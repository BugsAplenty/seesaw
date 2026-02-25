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
#include <cmath>
#include <chrono>

class UDPReader : public rclcpp::Node {
public:
  UDPReader(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(node_name, options) {
    
    lidar_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    lidar_port_ = declare_parameter<int>("lidar_port", 12345);
    imu_port_ = declare_parameter<int>("imu_port", 12346);
    
    RCLCPP_INFO(get_logger(), "🚀 UDPReader: Lidar:%d IMU:%d", lidar_port_, imu_port_);
    
    // Setup lidar socket
    lidar_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in imu_addr = {0};
    setsockopt(lidar_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int bufsize = 1024*1024;
    setsockopt(lidar_sock_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    
    sockaddr_in lidar_addr{};
    lidar_addr.sin_family = AF_INET;
    lidar_addr.sin_addr.s_addr = INADDR_ANY;
    lidar_addr.sin_port = htons(lidar_port_);
    bind(lidar_sock_, (sockaddr*)&lidar_addr, sizeof(lidar_addr));
    
    // Non-blocking
    fcntl(lidar_sock_, F_SETFL, O_NONBLOCK);
    
    // Setup imu socket
    imu_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(imu_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    setsockopt(imu_sock_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    
    sockaddr_in imu_addr{};
    imu_addr.sin_family = AF_INET;
    imu_addr.sin_addr.s_addr = INADDR_ANY;
    imu_addr.sin_port = htons(imu_port_);
    bind(imu_sock_, (sockaddr*)&imu_addr, sizeof(imu_addr));
    
    fcntl(imu_sock_, F_SETFL, O_NONBLOCK);
    
    // Init scans
    scans_.fill(INFINITY);
    
    // Timers
    scan_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&UDPReader::publish_scan, this));
    imuodom_timer_ = create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&UDPReader::publish_imuodom, this));
    
    // Threads
    lidar_thread_ = std::thread(&UDPReader::recv_lidar_thread, this);
    imu_thread_ = std::thread(&UDPReader::recv_imu_thread, this);
  }
  
  ~UDPReader() {
    running_ = false;
    if (lidar_thread_.joinable()) lidar_thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();
    close(lidar_sock_);
    close(imu_sock_);
  }
  
private:
  void recv_lidar_thread() {
    char buffer[1500];
    sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);
    
    while (running_) {
      ssize_t len = recvfrom(lidar_sock_, buffer, sizeof(buffer), 0,
                            (sockaddr*)&sender_addr, &addr_len);
      if (len < 20) continue;
      
      // Parse packets (every 20 bytes)
      int np_points = len / 20;
      std::vector<uint32_t> packets(np_points * 5);
      memcpy(packets.data(), buffer, np_points * 20);
      
      std::vector<float> ts(np_points), phi(np_points), angle(np_points), dist(np_points);
      std::vector<uint8_t> qual(np_points);
      
      for (int i = 0; i < np_points; ++i) {
        ts[i] = packets[i*5 + 0] / 100.0f;
        phi[i] = static_cast<float>(packets[i*5 + 1]) / 100.0f;
        angle[i] = static_cast<float>(packets[i*5 + 2]) / 100.0f;
        dist[i] = static_cast<float>(packets[i*5 + 3]) / 1000.0f;
        qual[i] = buffer[16 + i*20];
      }
      
      // Filter valid points
      std::vector<int> valid_buckets;
      std::vector<float> valid_dist;
      for (int i = 0; i < np_points; ++i) {
        if (qual[i] >= 10 && dist[i] > 0.0f) {
          float valid_angle = fmodf(phi[i] + angle[i], 360.0f);
          int bucket = static_cast<int>(valid_angle * 10.0f) % 3600;
          valid_buckets.push_back(bucket);
          valid_dist.push_back(dist[i]);
        }
      }
      
      // Update min distances
      {
        std::lock_guard<std::mutex> lock(scans_mutex_);
        for (size_t j = 0; j < valid_buckets.size(); ++j) {
          int b = valid_buckets[j];
          if (valid_dist[j] < scans_[b]) {
            scans_[b] = valid_dist[j];
          }
        }
        points_since_reset_ += valid_buckets.size();
      }
      
      if (!lidar_logged_ && !valid_buckets.empty()) {
        char addr_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender_addr.sin_addr, addr_str, sizeof(addr_str));
        RCLCPP_INFO(get_logger(), "Lidar OK from %s:%d: %zu pts", addr_str, ntohs(sender_addr.sin_port), valid_buckets.size());
        lidar_logged_ = true;
      }
    }
  }
  
  // Add to private members:
uint64_t imu_packets_received_ = 0;
std::atomic<bool> imu_data_valid_{false};

// Replace recv_imu_thread():
void recv_imu_thread() {
    char buffer[128];  // Bigger buffer
    sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    auto log_period = std::chrono::steady_clock::now();

    while (running_) {
        ssize_t len = recvfrom(imu_sock_, buffer, sizeof(buffer), 0,
                            (sockaddr*)&sender_addr, &addr_len);
        
        if (len > 0) {
        imu_packets_received_++;
        
        // Debug log every 100 packets OR first packet
        auto now_t = std::chrono::steady_clock::now();
        if ((imu_packets_received_ % 100 == 0 || imu_packets_received_ == 1) ||
            std::chrono::duration_cast<std::chrono::seconds>(now_t - log_period).count() >= 5) {
            char addr_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &sender_addr.sin_addr, addr_str, sizeof(addr_str));
            RCLCPP_INFO(get_logger(), "IMU RAW #%zu: %zdB from %s:%d | first8=%02x%02x%02x%02x%02x%02x%02x%02x",
                        imu_packets_received_, len, addr_str, ntohs(sender_addr.sin_port),
                        (uint8_t)buffer[0], (uint8_t)buffer[1], (uint8_t)buffer[2], (uint8_t)buffer[3],
                        (uint8_t)buffer[4], (uint8_t)buffer[5], (uint8_t)buffer[6], (uint8_t)buffer[7]);
            log_period = now_t;
        }
        
        // Try parse (little-endian uint32_t ts + 6x float32)
        if (len >= 28) {
            uint32_t ts_ms;
            memcpy(&ts_ms, buffer + 0, 4);  // <I = little-endian uint32
            
            float floats[6];
            memcpy(floats, buffer + 4, 24);  // 6 floats = 24 bytes
            
            // Validate floats (not NaN/inf)
            bool valid = true;
            for (int i = 0; i < 6; ++i) {
            if (std::isnan(floats[i]) || std::isinf(floats[i])) {
                valid = false;
                break;
            }
            }
            
            if (valid) {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            latest_imu_ts_ = static_cast<double>(ts_ms) / 1000.0;
            latest_accel_ = Eigen::Vector3f(floats[0], floats[1], floats[2]);
            latest_gyro_ = Eigen::Vector3f(floats[3], floats[4], floats[5]);
            imu_data_valid_ = true;
            imu_count_++;
            
            RCLCPP_INFO_ONCE(get_logger(), "IMU PARSED: ts=%.3f accelZ=%.2f gyroZ=%.2f", 
                            latest_imu_ts_, floats[2], floats[5]);
            } else {
            RCLCPP_WARN(get_logger(), "IMU NaN/Inf detected");
            }
        } else {
            RCLCPP_DEBUG(get_logger(), "IMU short packet: %zdB", len);
        }
        }  // No else - non-blocking, don't log every empty recv
        
        // Yield CPU - CRITICAL for non-blocking loops [web:28]
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    }

  
  void publish_imuodom() {
    Eigen::Vector3f accel, gyro;
    double imu_ts;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (latest_imu_ts_ == 0.0) {
        RCLCPP_DEBUG(get_logger(), "No IMU data");
        return;
      }
      imu_ts = latest_imu_ts_;
      accel = latest_accel_;
      gyro = latest_gyro_;
    }
    
    auto now_stamp = now();
    const double g = 9.81;
    
    // Integrate yaw
    double dt = (prev_imu_ts_ > 0) ? (imu_ts - prev_imu_ts_) : 0.005;
    prev_imu_ts_ = imu_ts;
    yaw_ += gyro.z() * dt;
    
    auto q = quaternion_from_yaw(yaw_);
    
    // IMU message
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now_stamp;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation = q;
    for (int i = 0; i < 9; ++i) imu_msg.orientation_covariance[i] = 0.01;
    
    imu_msg.linear_acceleration.x = accel.x() * g;
    imu_msg.linear_acceleration.y = accel.y() * g;
    imu_msg.linear_acceleration.z = accel.z() * g;
    for (int i = 0; i < 9; ++i) imu_msg.linear_acceleration_covariance[i] = 0.1;
    for (int i = 9; i < 36; ++i) imu_msg.linear_acceleration_covariance[i] = 0.0;
    
    imu_msg.angular_velocity.x = gyro.x();
    imu_msg.angular_velocity.y = gyro.y();
    imu_msg.angular_velocity.z = gyro.z();
    for (int i = 0; i < 9; ++i) imu_msg.angular_velocity_covariance[i] = 0.01;
    for (int i = 9; i < 36; ++i) imu_msg.angular_velocity_covariance[i] = 0.0;
    
    imu_pub_->publish(imu_msg);
    
    // Odometry
    std::vector<double> pose_cov(36, 0.0);
    pose_cov[0] = 0.01; pose_cov[4] = 0.01; pose_cov[35] = 0.1;  // x,y,yaw
    
    std::vector<double> twist_cov(36, 0.0);
    twist_cov[0] = 0.1; twist_cov[5] = 0.1; twist_cov[35] = 0.05;  // ax,az,gz
    
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now_stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.orientation = q;
    odom_msg.pose.covariance = pose_cov;
    
    odom_msg.twist.twist.linear.x = accel.x() * g;
    odom_msg.twist.twist.linear.y = accel.y() * g;
    odom_msg.twist.twist.linear.z = accel.z() * g;
    odom_msg.twist.twist.angular.x = gyro.x();
    odom_msg.twist.twist.angular.y = gyro.y();
    odom_msg.twist.twist.angular.z = gyro.z();
    odom_msg.twist.covariance = twist_cov;
    
    odom_pub_->publish(odom_msg);
  }
  
  rclcpp::Time now() {
    return this->get_clock()->now();
  }
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // Sockets
  int lidar_sock_, imu_sock_;
  int lidar_port_, imu_port_;
  
  // State
  std::array<float, 3601> scans_;
  std::mutex scans_mutex_;
  size_t points_since_reset_ = 0;
  bool lidar_logged_ = false;
  
  double yaw_ = 0.0;
  double prev_imu_ts_ = 0.0;
  double latest_imu_ts_ = 0.0;
  Eigen::Vector3f latest_accel_, latest_gyro_;
  std::mutex imu_mutex_;
  uint64_t imu_count_ = 0;
  
  // Threads and timers
  std::atomic<bool> running_{true};
  std::thread lidar_thread_, imu_thread_;
  rclcpp::TimerBase::SharedPtr scan_timer_, imuodom_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPReader>("udp_reader"));
  rclcpp::shutdown();
  return 0;
}
