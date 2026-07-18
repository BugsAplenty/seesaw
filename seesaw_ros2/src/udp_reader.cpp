#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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
    publish_ms_ = declare_parameter<int>("publish_ms", 50); // 20Hz publish rate

    auto sensor_qos = rclcpp::SensorDataQoS();
    raw_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/raw_points", sensor_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float64>("/servo_angle", 10);

    // Setup PointCloud2 fields (X, Y, Z, Intensity)
    point_fields_.resize(4);
    point_fields_[0].name = "x";       point_fields_[0].offset = 0;  point_fields_[0].datatype = sensor_msgs::msg::PointField::FLOAT32; point_fields_[0].count = 1;
    point_fields_[1].name = "y";       point_fields_[1].offset = 4;  point_fields_[1].datatype = sensor_msgs::msg::PointField::FLOAT32; point_fields_[1].count = 1;
    point_fields_[2].name = "z";       point_fields_[2].offset = 8;  point_fields_[2].datatype = sensor_msgs::msg::PointField::FLOAT32; point_fields_[2].count = 1;
    point_fields_[3].name = "intensity"; point_fields_[3].offset = 12; point_fields_[3].datatype = sensor_msgs::msg::PointField::FLOAT32; point_fields_[3].count = 1;

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
    if (sock < 0) throw std::runtime_error(std::string("socket() failed for ") + label);

    int reuse = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    ::setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buffer_bytes_, sizeof(recv_buffer_bytes_));

    int flags = ::fcntl(sock, F_GETFL, 0);
    if (flags >= 0) ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      close_socket(sock, label);
      throw std::runtime_error(std::string("bind() failed for ") + label + " port " + std::to_string(port));
    }
  }

  void close_socket(int & sock, const char *) {
    if (sock >= 0) { ::close(sock); sock = -1; }
  }

  void recv_loop()
  {
    std::array<uint8_t, 2048> buf{};
    while (running_.load()) {
      fd_set readfds;
      FD_ZERO(&readfds);
      if (lidar_sock_ >= 0) FD_SET(lidar_sock_, &readfds);
      else { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }

      timeval tv{};
      tv.tv_sec = 0; tv.tv_usec = 20000;

      const int rc = ::select(lidar_sock_ + 1, &readfds, nullptr, nullptr, &tv);
      if (rc <= 0) continue;
      if (FD_ISSET(lidar_sock_, &readfds)) drain_socket(lidar_sock_, buf);
    }
  }

  void drain_socket(int sock, std::array<uint8_t, 2048> & buf)
  {
    for (;;) {
      sockaddr_in sender{}; socklen_t sender_len = sizeof(sender);
      const ssize_t len = ::recvfrom(sock, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr *>(&sender), &sender_len);
      if (len < 0) break;
      parse_lidar(buf.data(), static_cast<size_t>(len));
    }
  }

  void parse_lidar(const uint8_t * data, size_t len)
  {
    constexpr size_t kPointBytes = 16;
    if (len < kPointBytes || (len % kPointBytes) != 0) return;

    const size_t count = len / kPointBytes;

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < count; ++i) {
      const uint8_t * p = data + i * kPointBytes;

      uint32_t raw_ts_us = 0;
      uint16_t servo_raw = 0, angle_q6 = 0, dist_mm = 0;
      uint8_t quality = 0;

      std::memcpy(&raw_ts_us, p + 0, 4);
      std::memcpy(&servo_raw, p + 4, 2);
      std::memcpy(&angle_q6, p + 6, 2);
      std::memcpy(&dist_mm, p + 8, 2);
      quality = p[11];

      // Update servo angle (degrees)
      if (servo_raw >= 511) {
        latest_servo_deg_ = 105.0f * static_cast<float>(servo_raw - 511) / 512.0f;
      } else {
        latest_servo_deg_ = -105.0f * static_cast<float>(511 - servo_raw) / 511.0f;
      }

      const float dist_m = static_cast<float>(dist_mm) * 0.001f;
      if (dist_m > 0.0f && quality > 0) {
        const float az_deg = static_cast<float>(angle_q6) / 64.0f;
        const float az_rad = az_deg * (M_PI / 180.0f);
        
        // Store raw 2D point (Z=0) and intensity
        live_points_.push_back({dist_m * std::cos(az_rad), dist_m * std::sin(az_rad), 0.0f, static_cast<float>(quality)});
      }
    }
  }

  void publish_outputs()
  {
    std::vector<float> pts_to_publish;
    float servo_val = 0.0;
    
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      // Flatten the vector of structs into a single vector of floats for fast memcpy
      pts_to_publish.resize(live_points_.size() * 4);
      if (!live_points_.empty()) {
        std::memcpy(pts_to_publish.data(), live_points_.data(), sizeof(float) * pts_to_publish.size());
      }
      live_points_.clear();
      servo_val = latest_servo_deg_;
    }

    // Publish Raw 2D PointCloud
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = "lidar_link";
    msg.height = 1;
    msg.fields = point_fields_;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.point_step = 16; // 4 floats * 4 bytes
    msg.width = static_cast<uint32_t>(pts_to_publish.size() / 4);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    if (!pts_to_publish.empty()) {
      std::memcpy(msg.data.data(), pts_to_publish.data(), msg.data.size());
    }
    raw_pub_->publish(std::move(msg));

    // Publish Servo Angle
    std_msgs::msg::Float64 servo_msg;
    servo_msg.data = servo_val;
    servo_pub_->publish(std::move(servo_msg));
  }

private:
  struct PointXYZI { float x; float y; float z; float intensity; };

  int lidar_port_{12345};
  int recv_buffer_bytes_{4 * 1024 * 1024};
  int publish_ms_{50};

  int lidar_sock_{-1};
  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  std::mutex data_mutex_;
  std::vector<PointXYZI> live_points_;
  float latest_servo_deg_{0.0f};

  std::vector<sensor_msgs::msg::PointField> point_fields_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;
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