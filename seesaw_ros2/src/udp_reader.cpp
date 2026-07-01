#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class UDPReader : public rclcpp::Node
{
public:
  UDPReader()
  : Node("udp_reader")
  {
    lidar_port_ = declare_parameter<int>("lidar_port", 12345);
    recv_buffer_bytes_ = declare_parameter<int>("recv_buffer_bytes", 4 * 1024 * 1024);
    scan_bins_ = declare_parameter<int>("scan_bins", 3600);
    publish_ms_ = declare_parameter<int>("publish_ms", 100);

    auto sensor_qos = rclcpp::SensorDataQoS();
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float64>("/servo_angle", 10);

    scan_accum_.assign(static_cast<size_t>(scan_bins_), std::numeric_limits<float>::infinity());
    scan_publish_.assign(static_cast<size_t>(scan_bins_), std::numeric_limits<float>::infinity());

    open_socket(lidar_sock_, lidar_port_, "lidar");

    running_.store(true);
    recv_thread_ = std::thread(&UDPReader::recv_loop, this);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_ms_),
      std::bind(&UDPReader::publish_outputs, this));

    RCLCPP_INFO(get_logger(), "udp_reader up: listening on lidar_port=%d", lidar_port_);
  }

  ~UDPReader() override
  {
    running_.store(false);
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    close_socket(lidar_sock_, "lidar");
  }

private:
  void open_socket(int & sock, int port, const char * label)
  {
    sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      throw std::runtime_error(std::string("socket() failed for ") + label);
    }

    int reuse = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    ::setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buffer_bytes_, sizeof(recv_buffer_bytes_));

    int flags = ::fcntl(sock, F_GETFL, 0);
    if (flags >= 0) {
      ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close_socket(sock, label);
      throw std::runtime_error(std::string("bind() failed for ") + label + " port " + std::to_string(port));
    }
  }

  void close_socket(int & sock, const char * /*label*/)
  {
    if (sock >= 0) {
      ::close(sock);
      sock = -1;
    }
  }

  void recv_loop()
  {
    std::array<uint8_t, 2048> buf{};
    while (running_.load()) {
      fd_set readfds;
      FD_ZERO(&readfds);

      int max_fd = -1;
      if (lidar_sock_ >= 0) {
        FD_SET(lidar_sock_, &readfds);
        max_fd = std::max(max_fd, lidar_sock_);
      }

      if (max_fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 20000;

      const int rc = ::select(max_fd + 1, &readfds, nullptr, nullptr, &tv);
      if (rc <= 0) continue;

      if (lidar_sock_ >= 0 && FD_ISSET(lidar_sock_, &readfds)) {
        drain_socket(lidar_sock_, buf);
      }
    }
  }

  void drain_socket(int sock, std::array<uint8_t, 2048> & buf)
  {
    for (;;) {
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);

      const ssize_t len = ::recvfrom(
        sock, buf.data(), buf.size(), 0,
        reinterpret_cast<sockaddr *>(&sender), &sender_len);

      if (len < 0) {
        break; // EAGAIN or error
      }

      parse_lidar(buf.data(), static_cast<size_t>(len));
    }
  }

  void parse_lidar(const uint8_t * data, size_t len)
  {
    constexpr size_t kPointBytes = 16;
    
    // DEBUG: Print raw packet length
    RCLCPP_INFO(get_logger(), "Received UDP packet of length %zu bytes", len);

    if (len < kPointBytes || (len % kPointBytes) != 0) {
      RCLCPP_WARN(get_logger(), "Packet length %zu is not divisible by %zu. Throwing away.", len, kPointBytes);
      return;
    }

    const size_t count = len / kPointBytes;

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < count; ++i) {
      const uint8_t * p = data + i * kPointBytes;

      uint32_t raw_ts_us = 0;
      uint16_t servo_raw = 0;
      uint16_t angle_q6 = 0;
      uint16_t dist_mm = 0;
      uint8_t new_rot_flag = 0;
      uint8_t quality = 0;

      std::memcpy(&raw_ts_us, p + 0, 4);
      std::memcpy(&servo_raw, p + 4, 2);
      std::memcpy(&angle_q6, p + 6, 2);
      std::memcpy(&dist_mm, p + 8, 2);
      new_rot_flag = p[10];
      quality = p[11];

      // DEBUG: Print the first point of every packet
      if (i == 0) {
        RCLCPP_INFO(get_logger(), "First Point Raw: ts=%u, servo=%u, ang_q6=%u, dist_mm=%u, rot=%u, qual=%u",
                    raw_ts_us, servo_raw, angle_q6, dist_mm, new_rot_flag, quality);
      }

      // Update servo angle (degrees)
      if (servo_raw >= 511) {
        latest_servo_deg_ = 105.0f * static_cast<float>(servo_raw - 511) / static_cast<float>(1023 - 511);
      } else {
        latest_servo_deg_ = -105.0f * static_cast<float>(511 - servo_raw) / static_cast<float>(511);
      }

      // Update 2D lidar scan
      const float dist_m = static_cast<float>(dist_mm) * 0.001f;
      if (dist_m <= 0.0f || quality == 0) continue;

      const float az_deg = static_cast<float>(angle_q6) / 64.0f;
      const float az_rad = az_deg * (M_PI / 180.0f);
      
      int bin = static_cast<int>((az_rad / (2.0 * M_PI)) * static_cast<float>(scan_bins_));
      if (bin >= scan_bins_) bin = scan_bins_ - 1;
      if (bin >= 0) {
        auto & cell = scan_accum_[static_cast<size_t>(bin)];
        if (dist_m < cell) {
          cell = dist_m;
        }
      }
    }
  }

  void publish_outputs()
  {
    float servo_val = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      std::swap(scan_accum_, scan_publish_);
      std::fill(scan_accum_.begin(), scan_accum_.end(), std::numeric_limits<float>::infinity());
      servo_val = latest_servo_deg_;
    }

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = "lidar_link";
    msg.angle_min = 0.0f;
    msg.angle_max = 2.0f * M_PI;
    msg.angle_increment = (2.0f * M_PI) / static_cast<float>(scan_bins_);
    msg.time_increment = 0.0f;
    msg.scan_time = static_cast<float>(publish_ms_) / 1000.0f;
    msg.range_min = 0.05f;
    msg.range_max = 12.0f;
    msg.ranges = scan_publish_;
    scan_pub_->publish(std::move(msg));

    std_msgs::msg::Float64 servo_msg;
    servo_msg.data = servo_val;
    servo_pub_->publish(std::move(servo_msg));
  }

private:
  int lidar_port_{12345};
  int recv_buffer_bytes_{4 * 1024 * 1024};
  int scan_bins_{3600};
  int publish_ms_{100};

  int lidar_sock_{-1};
  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  std::mutex data_mutex_;
  std::vector<float> scan_accum_;
  std::vector<float> scan_publish_;
  float latest_servo_deg_{0.0f};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPReader>());
  rclcpp::shutdown();
  return 0;
}