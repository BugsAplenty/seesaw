// src/udp_reader.cpp - COMPLETE DROP-IN for your ESP32 Config.h ports
// PC binds: 12345(lidar)/12346(imu)  Filters ESP32 src:8888(lidar)/8889(imu)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cmath>
#include <rclcpp/qos.hpp>  // Add this include

class UDPReader : public rclcpp::Node {
public:
  UDPReader() : Node("udp_reader") {
    // REPLACE these 3 lines:
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();  // BEST_EFFORT, KEEP_LAST(5)
    lidar_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", sensor_qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", sensor_qos);
    setup_sockets();
    recv_thread_ = std::thread(&UDPReader::recv_loop, this);
    
    scan_timer_ = create_wall_timer(std::chrono::milliseconds(20), 
      std::bind(&UDPReader::publish_scan, this));
    imuodom_timer_ = create_wall_timer(std::chrono::milliseconds(5), 
      std::bind(&UDPReader::publish_imuodom, this));
    
    RCLCPP_INFO(get_logger(), "🚀 UDPReader: bind 12345(lidar)/12346(imu) ← ESP32 src 8888/8889");
  }
  
  ~UDPReader() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (lidar_sock_ >= 0) close(lidar_sock_);
    if (imu_sock_ >= 0) close(imu_sock_);
  }
  
private:
  void setup_sockets() {
    int reuse = 1;
    int bufsize = 2*1024*1024;
    struct timeval tv = {0, 10000};
    
    // LIDAR: bind ESP32 UDP_REMOTE_PORT_LIDAR = 12345
    lidar_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(lidar_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(lidar_sock_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    setsockopt(lidar_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    sockaddr_in lidar_addr{};
    lidar_addr.sin_family = AF_INET;
    lidar_addr.sin_addr.s_addr = INADDR_ANY;
    lidar_addr.sin_port = htons(12345);  // ← ESP32 UDP_REMOTE_PORT_LIDAR
    if (bind(lidar_sock_, (sockaddr*)&lidar_addr, sizeof(lidar_addr)) == 0) {
      RCLCPP_INFO(get_logger(), "✅ LIDAR bound port 12345 (ESP32 dest)");
    } else {
      RCLCPP_FATAL(get_logger(), "LIDAR bind 12345 failed: %s", strerror(errno));
      rclcpp::shutdown();
    }
    
    // IMU: bind ESP32 UDP_REMOTE_PORT_IMU = 12346
    imu_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(imu_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(imu_sock_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    setsockopt(imu_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    sockaddr_in imu_addr{};
    imu_addr.sin_family = AF_INET;
    imu_addr.sin_addr.s_addr = INADDR_ANY;
    imu_addr.sin_port = htons(12346);  // ← ESP32 UDP_REMOTE_PORT_IMU
    if (bind(imu_sock_, (sockaddr*)&imu_addr, sizeof(imu_addr)) == 0) {
      RCLCPP_INFO(get_logger(), "✅ IMU bound port 12346 (ESP32 dest)");
    } else {
      RCLCPP_FATAL(get_logger(), "IMU bind 12346 failed: %s", strerror(errno));
      rclcpp::shutdown();
    }
  }
  
  void recv_loop() {
    char buf[1500];
    sockaddr_in sender;
    socklen_t addrlen = sizeof(sender);
    
    while (running_) {
      fd_set fds;
      FD_ZERO(&fds);
      if (lidar_sock_ >= 0) FD_SET(lidar_sock_, &fds);
      if (imu_sock_ >= 0) FD_SET(imu_sock_, &fds);
      
      int max_fd = 0;
      if (lidar_sock_ >= 0) max_fd = std::max(max_fd, lidar_sock_);
      if (imu_sock_ >= 0) max_fd = std::max(max_fd, imu_sock_);
      max_fd++;
      
      struct timeval tv = {0, 5000};  // 5ms poll
      int activity = select(max_fd, &fds, NULL, NULL, &tv);
      
      if (activity < 0 && errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "select: %s", strerror(errno));
        continue;
      }
      
      // LIDAR socket (dest 12345) ← ESP32 src 8888
      if (lidar_sock_ >= 0 && FD_ISSET(lidar_sock_, &fds)) {
        ssize_t len = recvfrom(lidar_sock_, buf, sizeof(buf), 0, (sockaddr*)&sender, &addrlen);
        if (len > 0 && ntohs(sender.sin_port) == 8888) {  // ESP32 UDP_LOCAL_PORT_LIDAR
          parse_lidar(buf, len);
          lidar_packets_++;
        }
      }
      
      // IMU socket (dest 12346) ← ESP32 src 8889
      if (imu_sock_ >= 0 && FD_ISSET(imu_sock_, &fds)) {
        ssize_t len = recvfrom(imu_sock_, buf, sizeof(buf), 0, (sockaddr*)&sender, &addrlen);
        if (len > 0 && ntohs(sender.sin_port) == 8889) {  // ESP32 UDP_LOCAL_PORT_IMU
          parse_imu(buf, len);
          imu_packets_++;
        }
      }
      
      // Stats
      if ((lidar_packets_ + imu_packets_) % 1000 == 0) {
        RCLCPP_INFO(get_logger(), "Pkts: lidar=%zu imu=%zu", lidar_packets_, imu_packets_);
      }
    }
  }
  
  void parse_imu(const char* data, ssize_t len) {
    if (len != 28) return;
    
    uint32_t ts_be;
    std::memcpy(&ts_be, data, 4);
    uint32_t ts_ms = __builtin_bswap32(ts_be);
    
    float vals[6];
    std::memcpy(vals, data + 4, 24);
    for (auto& v : vals) {
      uint32_t bits;
      std::memcpy(&bits, &v, sizeof(bits));
      bits = __builtin_bswap32(bits);
      std::memcpy(&v, &bits, sizeof(v));
    }
    
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_ts_ = ts_ms / 1000.0;
    latest_accel_ = {vals[0], vals[1], vals[2]};
    latest_gyro_ = {vals[3], vals[4], vals[5]};
    imu_data_valid_ = true;
    imu_count_++;
    RCLCPP_DEBUG(get_logger(), "parse_imu: len=%zd ts=%.1f accel_z=%.2f gyro_z=%.2f", 
             len, latest_imu_ts_, latest_gyro_[2], latest_accel_[2]);
    
    if (imu_count_ % 1000 == 0) {
      RCLCPP_INFO(get_logger(), "IMU #%zu: Z=%.3f/%.3f", imu_count_, vals[2], vals[5]);
    }
  }
  
  void parse_lidar(const char* data, ssize_t len) {
    if (len < 20 || len % 20 != 0) return;
    
    int npoints = len / 20;
    std::lock_guard<std::mutex> lock(scans_mutex_);
    
    for (int i = 0; i < npoints; i++) {
      const char* pkt = data + i * 20;
      uint32_t raw_phi, raw_angle, raw_dist;
      std::memcpy(&raw_phi, pkt + 4, 4);
      std::memcpy(&raw_angle, pkt + 8, 4);
      std::memcpy(&raw_dist, pkt + 12, 4);
      uint8_t qual = pkt[16];
      
      if (qual < 10) continue;
      
      raw_phi = __builtin_bswap32(raw_phi);
      raw_angle = __builtin_bswap32(raw_angle);
      raw_dist = __builtin_bswap32(raw_dist);
      
      float phi = raw_phi / 100.0f;
      float angle = raw_angle / 100.0f;
      float dist = raw_dist / 1000.0f;
      
      if (dist > 0.01f) {
        float abs_angle = fmodf(phi + angle, 360.0f);
        int bucket = static_cast<int>(abs_angle * 10) % 3600;
        if (dist < scans_[bucket]) scans_[bucket] = dist;
        points_since_reset_++;
      }
    }
  }
  
  geometry_msgs::msg::Quaternion quat_from_yaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion msg;
    q.normalize();
    msg.x = q.x(); msg.y = q.y(); msg.z = q.z(); msg.w = q.w();
    return msg;
  }
  
  void publish_scan() {
    std::lock_guard<std::mutex> lock(scans_mutex_);
    if (points_since_reset_ < 100) return;
    
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "lidar_link";
    msg.angle_min = 0;
    msg.angle_max = 2 * M_PI;
    msg.angle_increment = 2 * M_PI / 3600;
    msg.range_min = 0.01;
    msg.range_max = 12.0;
    msg.ranges.assign(scans_.begin(), scans_.end());
    
    lidar_pub_->publish(msg);
    scans_.fill(INFINITY);
    points_since_reset_ = 0;
  }
  
  void publish_imuodom() {
    Eigen::Vector3f accel, gyro;
    double ts;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (!imu_data_valid_) return;
      ts = latest_imu_ts_;
      accel = latest_accel_;
      gyro = latest_gyro_;
    }
    
    auto now = this->get_clock()->now();
    double dt = (prev_imu_ts_ > 0) ? (ts - prev_imu_ts_) : 0.005;
    prev_imu_ts_ = ts;
    yaw_ += gyro.z() * dt;
    
    auto q = quat_from_yaw(yaw_);
    const double g = 9.81;
    
    // IMU msg
    auto imu = sensor_msgs::msg::Imu();
    imu.header.stamp = now;
    imu.header.frame_id = "imu_link";
    imu.orientation = q;
    std::fill(imu.orientation_covariance.begin(), imu.orientation_covariance.begin() + 9, 0.01);
    
    imu.linear_acceleration.x = accel.x() * g;
    imu.linear_acceleration.y = accel.y() * g;
    imu.linear_acceleration.z = accel.z() * g;
    std::fill(imu.linear_acceleration_covariance.begin(), imu.linear_acceleration_covariance.begin() + 9, 0.1);
    
    imu.angular_velocity.x = gyro.x();
    imu.angular_velocity.y = gyro.y();
    imu.angular_velocity.z = gyro.z();
    std::fill(imu.angular_velocity_covariance.begin(), imu.angular_velocity_covariance.begin() + 9, 0.01);
    
    imu_pub_->publish(imu);
    
    // Odom msg
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.orientation = q;
    
    odom.twist.twist.linear.x = accel.x() * g;
    odom.twist.twist.linear.y = accel.y() * g;
    odom.twist.twist.angular.z = gyro.z();
    
    odom_pub_->publish(odom);
  }
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // Sockets
  int lidar_sock_ = -1, imu_sock_ = -1;
  std::thread recv_thread_;
  std::atomic<bool> running_{true};
  
  // Timers
  rclcpp::TimerBase::SharedPtr scan_timer_, imuodom_timer_;
  
  // Lidar
  std::array<float, 3601> scans_{INFINITY};
  std::mutex scans_mutex_;
  size_t points_since_reset_ = 0;
  size_t lidar_packets_ = 0;
  
  // IMU
  std::mutex imu_mutex_;
  double latest_imu_ts_ = 0, prev_imu_ts_ = 0, yaw_ = 0;
  Eigen::Vector3f latest_accel_, latest_gyro_;
  std::atomic<bool> imu_data_valid_{false};
  uint64_t imu_count_ = 0, imu_packets_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UDPReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
