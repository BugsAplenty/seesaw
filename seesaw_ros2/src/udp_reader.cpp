#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <mutex>

class UDPReader : public rclcpp::Node {
public:
  UDPReader() : Node("udp_reader") {
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    lidar_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", sensor_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    // Init Broadcaster to prevent Exit Code -11
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    setup_sockets();
    recv_thread_ = std::thread(&UDPReader::recv_loop, this);
    
    // Only timer is IMU. Lidar publishes itself automatically on zero-crossing.
    imuodom_timer_ = create_wall_timer(std::chrono::milliseconds(5), 
      std::bind(&UDPReader::publish_imuodom, this));
    
    RCLCPP_INFO(get_logger(), "🚀 UDPReader started.");
  }
  
  ~UDPReader() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (lidar_sock_ >= 0) close(lidar_sock_);
    if (imu_sock_ >= 0) close(imu_sock_);
  }
  
private:
  void setup_sockets() {
    int reuse = 1; int bufsize = 2*1024*1024; struct timeval tv = {0, 10000};
    
    lidar_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(lidar_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(lidar_sock_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    setsockopt(lidar_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in lidar_addr{}; lidar_addr.sin_family = AF_INET; lidar_addr.sin_addr.s_addr = INADDR_ANY; lidar_addr.sin_port = htons(12345);
    bind(lidar_sock_, (sockaddr*)&lidar_addr, sizeof(lidar_addr));
    
    imu_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(imu_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(imu_sock_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    setsockopt(imu_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in imu_addr{}; imu_addr.sin_family = AF_INET; imu_addr.sin_addr.s_addr = INADDR_ANY; imu_addr.sin_port = htons(12346);
    bind(imu_sock_, (sockaddr*)&imu_addr, sizeof(imu_addr));
  }
  
  void recv_loop() {
    char buf[1500]; sockaddr_in sender; socklen_t addrlen = sizeof(sender);
    while (running_) {
      fd_set fds; FD_ZERO(&fds);
      if (lidar_sock_ >= 0) FD_SET(lidar_sock_, &fds);
      if (imu_sock_ >= 0) FD_SET(imu_sock_, &fds);
      
      int max_fd = std::max(lidar_sock_, imu_sock_) + 1;
      struct timeval tv = {0, 5000};
      if (select(max_fd, &fds, NULL, NULL, &tv) <= 0) continue;
      
      if (FD_ISSET(lidar_sock_, &fds)) {
        ssize_t len = recvfrom(lidar_sock_, buf, sizeof(buf), 0, (sockaddr*)&sender, &addrlen);
        if (len > 0) parse_lidar(buf, len);
      }
      if (FD_ISSET(imu_sock_, &fds)) {
        ssize_t len = recvfrom(imu_sock_, buf, sizeof(buf), 0, (sockaddr*)&sender, &addrlen);
        if (len > 0) parse_imu(buf, len);
      }
    }
  }
  
  void parse_imu(const char* data, ssize_t len) {
    if (len != 28) return;
    uint32_t ts_ms; std::memcpy(&ts_ms, data, 4);
    float vals[6]; std::memcpy(vals, data + 4, 24);
    
    Eigen::Vector3f raw_accel{vals[0], vals[1], vals[2]};
    Eigen::Vector3f raw_gyro{vals[3], vals[4], vals[5]};

    // Auto Calibrate on boot
    if (!imu_calibrated_) {
      gyro_bias_ += raw_gyro;
      calib_samples_++;
      if (calib_samples_ == 1) RCLCPP_INFO(get_logger(), "IMU Calibrating: DO NOT MOVE ROBOT...");
      if (calib_samples_ >= 200) {
        gyro_bias_ /= 200.0f;
        imu_calibrated_ = true;
        RCLCPP_INFO(get_logger(), "✅ IMU Calibrated! Gyro Bias subtracted.");
      }
      return; 
    }
    raw_gyro -= gyro_bias_;

    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_ts_ = ts_ms / 1000.0;
    latest_accel_ = raw_accel;
    latest_gyro_ = raw_gyro;
    imu_data_valid_ = true;
  }

  void parse_lidar(const char* data, ssize_t len) {
    if (len < 20 || len % 20 != 0) return;
    int npoints = len / 20;
    bool sweep_complete = false;
    
    {
      std::lock_guard<std::mutex> lock(scans_mutex_);
      for (int i = 0; i < npoints; i++) {
        const char* pkt = data + i * 20;
        int32_t raw_phi, raw_angle, raw_dist;
        
        std::memcpy(&raw_phi, pkt + 4, 4);
        std::memcpy(&raw_angle, pkt + 8, 4);
        std::memcpy(&raw_dist, pkt + 12, 4);
        
        float phi = raw_phi / 100.0f;
        float angle = raw_angle / 100.0f;
        float dist = raw_dist / 1000.0f;
        
        if (dist > 0.01f && dist < 12.0f) {
          float abs_angle = fmodf(phi + angle, 360.0f);
          if (abs_angle < 0) abs_angle += 360.0f;
          
          if (abs_angle < 90.0f && last_angle_ > 270.0f) sweep_complete = true;
          last_angle_ = abs_angle;

          int bucket = static_cast<int>(abs_angle * 10.0f) % 3600;
          scans_[bucket] = std::min(scans_[bucket], dist);
          points_since_reset_++;
        }
      }
    }
    
    if (sweep_complete) publish_scan_sync();
  }

  void publish_scan_sync() {
    std::array<float, 3601> temp_scans;
    size_t temp_points;
    
    {
      std::lock_guard<std::mutex> lock(scans_mutex_);
      if (points_since_reset_ < 50) return;
      temp_scans = scans_;
      temp_points = points_since_reset_;
      scans_.fill(INFINITY);
      points_since_reset_ = 0;
    }
    
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "lidar_link";
    msg.angle_min = 0;
    msg.angle_max = 2 * M_PI;
    msg.angle_increment = 2 * M_PI / 3600;
    msg.time_increment = 0.1 / 3600.0;
    msg.scan_time = 0.1;
    msg.range_min = 0.01;
    msg.range_max = 12.0;
    msg.ranges = {temp_scans.begin(), temp_scans.end()};
    
    lidar_pub_->publish(msg);
  }

  void publish_imuodom() {
    Eigen::Vector3f accel, gyro;
    double ts;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (!imu_data_valid_) return;
      ts = latest_imu_ts_; accel = latest_accel_; gyro = latest_gyro_;
    }
    
    auto now = this->get_clock()->now();
    double dt = (prev_imu_ts_ > 0) ? (ts - prev_imu_ts_) : 0.005;
    prev_imu_ts_ = ts;
    
    roll_ += gyro.x() * dt;
    pitch_ += gyro.y() * dt;
    yaw_ += gyro.z() * dt;
    
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    q.normalize();
    
    // Broadcast Dynamic TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = 0.0; t.transform.translation.y = 0.0; t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z(); t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
    
    // Publish IMU
    auto imu = sensor_msgs::msg::Imu();
    imu.header.stamp = now;
    imu.header.frame_id = "imu_link";
    imu.orientation.x = q.x(); imu.orientation.y = q.y();
    imu.orientation.z = q.z(); imu.orientation.w = q.w();
    imu.angular_velocity.x = gyro.x(); imu.angular_velocity.y = gyro.y(); imu.angular_velocity.z = gyro.z();
    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  int lidar_sock_ = -1, imu_sock_ = -1;
  std::thread recv_thread_;
  std::atomic<bool> running_{true};
  
  rclcpp::TimerBase::SharedPtr imuodom_timer_;
  
  std::array<float, 3601> scans_{INFINITY};
  std::mutex scans_mutex_;
  size_t points_since_reset_ = 0;
  float last_angle_ = 0.0f;
  
  std::mutex imu_mutex_;
  double latest_imu_ts_ = 0, prev_imu_ts_ = 0;
  Eigen::Vector3f latest_accel_, latest_gyro_;
  std::atomic<bool> imu_data_valid_{false};
  
  bool imu_calibrated_ = false;
  int calib_samples_ = 0;
  Eigen::Vector3f gyro_bias_{0, 0, 0};
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPReader>());
  rclcpp::shutdown();
  return 0;
}
