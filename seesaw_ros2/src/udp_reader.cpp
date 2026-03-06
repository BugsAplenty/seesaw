#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <Eigen/Core>

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
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;

constexpr size_t kLidarPointBytes = 20;
constexpr size_t kImuPacketBytes = 28;
constexpr size_t kRecvBufferBytes = 2048;

inline float wrap_deg(float deg)
{
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

struct PointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
};

static_assert(sizeof(PointXYZI) == 16, "PointXYZI must stay tightly packed");
}  // namespace

class UDPReader : public rclcpp::Node
{
public:
  UDPReader()
  : Node("udp_reader")
  {
    lidar_port_ = declare_parameter<int>("lidar_port", 12345);
    imu_port_ = declare_parameter<int>("imu_port", 12346);
    recv_buffer_bytes_ = declare_parameter<int>("recv_buffer_bytes", 4 * 1024 * 1024);

    publish_cloud_ = declare_parameter<bool>("publish_cloud", true);
    publish_scan_ = declare_parameter<bool>("publish_scan", true);

    cloud_publish_ms_ = declare_parameter<int>("cloud_publish_ms", 80);
    imu_publish_ms_ = declare_parameter<int>("imu_publish_ms", 5);
    health_log_ms_ = declare_parameter<int>("health_log_ms", 2000);

    verbose_startup_ = declare_parameter<bool>("verbose_startup", true);
    verbose_socket_ = declare_parameter<bool>("verbose_socket", true);
    verbose_packets_ = declare_parameter<bool>("verbose_packets", false);
    verbose_point_geometry_ = declare_parameter<bool>("verbose_point_geometry", false);

    const auto max_points_param = declare_parameter<int64_t>("max_points_per_cloud", 120000);
    max_points_per_cloud_ = static_cast<size_t>(std::max<int64_t>(1000, max_points_param));

    min_range_ = static_cast<float>(declare_parameter<double>("min_range", 0.05));
    max_range_ = static_cast<float>(declare_parameter<double>("max_range", 12.0));
    quality_min_ = declare_parameter<int>("quality_min", 0);

    const auto scan_bins_param = declare_parameter<int64_t>("scan_bins", 3600);
    scan_bins_ = static_cast<int>(std::max<int64_t>(360, scan_bins_param));

    azimuth_offset_deg_ = static_cast<float>(declare_parameter<double>("azimuth_offset_deg", 0.0));
    tilt_zero_deg_ = static_cast<float>(declare_parameter<double>("tilt_zero_deg", 180.0));
    tilt_sign_ = static_cast<float>(declare_parameter<double>("tilt_sign", 1.0));

    const std::string tilt_axis = declare_parameter<std::string>("tilt_axis", "axis_y");
    tilt_about_x_ =
      (tilt_axis == "x" || tilt_axis == "X" ||
       tilt_axis == "axis_x" || tilt_axis == "roll");

    pivot_x_ = static_cast<float>(declare_parameter<double>("pivot_x", 0.0));
    pivot_y_ = static_cast<float>(declare_parameter<double>("pivot_y", 0.0));
    pivot_z_ = static_cast<float>(declare_parameter<double>("pivot_z", 0.0));

    zero_gyro_z_ = declare_parameter<bool>("zero_gyro_z", true);

    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "base_link");
    scan_frame_ = declare_parameter<std::string>("scan_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");

    auto sensor_qos = rclcpp::SensorDataQoS();

    if (publish_cloud_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points", sensor_qos);
    }
    if (publish_scan_) {
      scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", sensor_qos);
    }
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", sensor_qos);

    point_fields_.resize(4);
    point_fields_[0].name = "x";
    point_fields_[0].offset = 0;
    point_fields_[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_fields_[0].count = 1;

    point_fields_[1].name = "y";
    point_fields_[1].offset = 4;
    point_fields_[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_fields_[1].count = 1;

    point_fields_[2].name = "z";
    point_fields_[2].offset = 8;
    point_fields_[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_fields_[2].count = 1;

    point_fields_[3].name = "intensity";
    point_fields_[3].offset = 12;
    point_fields_[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_fields_[3].count = 1;

    live_points_.reserve(max_points_per_cloud_);
    publish_points_.reserve(max_points_per_cloud_);

    scan_accum_.assign(static_cast<size_t>(scan_bins_), std::numeric_limits<float>::infinity());
    scan_publish_.assign(static_cast<size_t>(scan_bins_), std::numeric_limits<float>::infinity());

    if (verbose_startup_) {
      RCLCPP_INFO(get_logger(), "=== UDPReader startup ===");
      RCLCPP_INFO(get_logger(), "lidar_port=%d imu_port=%d recv_buffer_bytes=%d",
        lidar_port_, imu_port_, recv_buffer_bytes_);
      RCLCPP_INFO(get_logger(), "publish_cloud=%s publish_scan=%s cloud_publish_ms=%d imu_publish_ms=%d",
        tf(publish_cloud_), tf(publish_scan_), cloud_publish_ms_, imu_publish_ms_);
      RCLCPP_INFO(get_logger(), "max_points_per_cloud=%zu min_range=%.3f max_range=%.3f quality_min=%d",
        max_points_per_cloud_, min_range_, max_range_, quality_min_);
      RCLCPP_INFO(get_logger(), "scan_bins=%d azimuth_offset_deg=%.3f", scan_bins_, azimuth_offset_deg_);
      RCLCPP_INFO(get_logger(), "tilt_axis=%s tilt_about_x=%s tilt_zero_deg=%.3f tilt_sign=%.3f",
        tilt_axis.c_str(), tf(tilt_about_x_), tilt_zero_deg_, tilt_sign_);
      RCLCPP_INFO(get_logger(), "pivot=(%.3f, %.3f, %.3f)", pivot_x_, pivot_y_, pivot_z_);
      RCLCPP_INFO(get_logger(), "frames: cloud=%s scan=%s imu=%s",
        cloud_frame_.c_str(), scan_frame_.c_str(), imu_frame_.c_str());
      RCLCPP_INFO(get_logger(), "verbose_startup=%s verbose_socket=%s verbose_packets=%s verbose_point_geometry=%s",
        tf(verbose_startup_), tf(verbose_socket_), tf(verbose_packets_), tf(verbose_point_geometry_));
    }

    open_socket(lidar_sock_, lidar_port_, "lidar");
    open_socket(imu_sock_, imu_port_, "imu");

    running_.store(true);
    recv_thread_ = std::thread(&UDPReader::recv_loop, this);

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(cloud_publish_ms_),
      std::bind(&UDPReader::publish_outputs, this));

    imu_timer_ = create_wall_timer(
      std::chrono::milliseconds(imu_publish_ms_),
      std::bind(&UDPReader::publish_latest_imu, this));

    health_timer_ = create_wall_timer(
      std::chrono::milliseconds(health_log_ms_),
      std::bind(&UDPReader::log_health, this));

    RCLCPP_INFO(
      get_logger(),
      "udp_reader up: lidar_port=%d imu_port=%d cloud=%s scan=%s tilt_axis=%s",
      lidar_port_,
      imu_port_,
      publish_cloud_ ? "on" : "off",
      publish_scan_ ? "on" : "off",
      tilt_about_x_ ? "x" : "y");
  }

  ~UDPReader() override
  {
    RCLCPP_WARN(get_logger(), "UDPReader shutting down");
    running_.store(false);
    close_socket(lidar_sock_, "lidar");
    close_socket(imu_sock_, "imu");
    if (recv_thread_.joinable()) {
      recv_thread_.join();
    }
    RCLCPP_WARN(get_logger(), "UDPReader shutdown complete");
  }

private:
  static const char * tf(bool v) { return v ? "true" : "false"; }

  void open_socket(int & sock, int port, const char * label)
  {
    sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      throw std::runtime_error(std::string("socket() failed for ") + label + " port " + std::to_string(port));
    }

    if (verbose_socket_) {
      RCLCPP_INFO(get_logger(), "[%s] socket() -> fd=%d", label, sock);
    }

    int reuse = 1;
    if (::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] setsockopt(SO_REUSEADDR) failed: %s", label, std::strerror(errno));
    } else if (verbose_socket_) {
      RCLCPP_INFO(get_logger(), "[%s] SO_REUSEADDR set", label);
    }

    if (::setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buffer_bytes_, sizeof(recv_buffer_bytes_)) < 0) {
      RCLCPP_WARN(get_logger(), "[%s] setsockopt(SO_RCVBUF=%d) failed: %s",
        label, recv_buffer_bytes_, std::strerror(errno));
    } else if (verbose_socket_) {
      RCLCPP_INFO(get_logger(), "[%s] SO_RCVBUF requested=%d", label, recv_buffer_bytes_);
    }

    int flags = ::fcntl(sock, F_GETFL, 0);
    if (flags >= 0) {
      if (::fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        RCLCPP_WARN(get_logger(), "[%s] fcntl(F_SETFL,O_NONBLOCK) failed: %s",
          label, std::strerror(errno));
      } else if (verbose_socket_) {
        RCLCPP_INFO(get_logger(), "[%s] socket set nonblocking", label);
      }
    } else {
      RCLCPP_WARN(get_logger(), "[%s] fcntl(F_GETFL) failed: %s", label, std::strerror(errno));
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      const std::string err = std::strerror(errno);
      close_socket(sock, label);
      throw std::runtime_error(std::string("bind() failed for ") + label + " port " +
                               std::to_string(port) + ": " + err);
    }

    if (verbose_socket_) {
      RCLCPP_INFO(get_logger(), "[%s] bind() ok on 0.0.0.0:%d fd=%d", label, port, sock);
    }
  }

  void close_socket(int & sock, const char * label)
  {
    if (sock >= 0) {
      if (verbose_socket_) {
        RCLCPP_INFO(get_logger(), "[%s] closing fd=%d", label, sock);
      }
      ::close(sock);
      sock = -1;
    }
  }

  void recv_loop()
  {
    std::array<uint8_t, kRecvBufferBytes> buf{};
    RCLCPP_INFO(get_logger(), "recv_loop started");

    while (running_.load()) {
      fd_set readfds;
      FD_ZERO(&readfds);

      int max_fd = -1;
      if (lidar_sock_ >= 0) {
        FD_SET(lidar_sock_, &readfds);
        max_fd = std::max(max_fd, lidar_sock_);
      }
      if (imu_sock_ >= 0) {
        FD_SET(imu_sock_, &readfds);
        max_fd = std::max(max_fd, imu_sock_);
      }

      if (max_fd < 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "recv_loop has no valid sockets; sleeping");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 20000;

      select_calls_.fetch_add(1, std::memory_order_relaxed);
      const int rc = ::select(max_fd + 1, &readfds, nullptr, nullptr, &tv);

      if (rc < 0) {
        select_errors_.fetch_add(1, std::memory_order_relaxed);
        if (errno == EINTR) {
          continue;
        }
        if (running_.load()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "select() failed: %s", std::strerror(errno));
        }
        continue;
      }

      if (rc == 0) {
        select_timeouts_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "recv_loop select timeout");
        continue;
      }

      if (lidar_sock_ >= 0 && FD_ISSET(lidar_sock_, &readfds)) {
        lidar_ready_events_.fetch_add(1, std::memory_order_relaxed);
        drain_socket(lidar_sock_, buf, true);
      }

      if (imu_sock_ >= 0 && FD_ISSET(imu_sock_, &readfds)) {
        imu_ready_events_.fetch_add(1, std::memory_order_relaxed);
        drain_socket(imu_sock_, buf, false);
      }
    }

    RCLCPP_INFO(get_logger(), "recv_loop exiting");
  }

  void drain_socket(int sock, std::array<uint8_t, kRecvBufferBytes> & buf, bool is_lidar)
  {
    const char * label = is_lidar ? "lidar" : "imu";

    for (;;) {
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);

      const ssize_t len = ::recvfrom(
        sock,
        buf.data(),
        buf.size(),
        0,
        reinterpret_cast<sockaddr *>(&sender),
        &sender_len);

      if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }

        recv_errors_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[%s] recvfrom() failed: %s", label, std::strerror(errno));
        break;
      }

      const uint16_t src_port = ntohs(sender.sin_port);
      char src_ip[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &sender.sin_addr, src_ip, sizeof(src_ip));

      last_packet_time_ns_.store(now().nanoseconds(), std::memory_order_relaxed);

      if (is_lidar) {
        lidar_packets_rx_.fetch_add(1, std::memory_order_relaxed);
        lidar_bytes_rx_.fetch_add(static_cast<uint64_t>(len), std::memory_order_relaxed);
        last_lidar_time_ns_.store(now().nanoseconds(), std::memory_order_relaxed);
      } else {
        imu_packets_rx_.fetch_add(1, std::memory_order_relaxed);
        imu_bytes_rx_.fetch_add(static_cast<uint64_t>(len), std::memory_order_relaxed);
        last_imu_time_ns_.store(now().nanoseconds(), std::memory_order_relaxed);
      }

      if (verbose_packets_) {
        RCLCPP_INFO(
          get_logger(),
          "[%s] recvfrom len=%zd from %s:%u fd=%d",
          label, len, src_ip, src_port, sock);
      } else {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[%s] receiving packets; latest len=%zd from %s:%u",
          label, len, src_ip, src_port);
      }

      if (is_lidar) {
        parse_lidar(buf.data(), static_cast<size_t>(len), src_ip, src_port);
      } else {
        parse_imu(buf.data(), static_cast<size_t>(len), src_ip, src_port);
      }
    }
  }

  void parse_imu(const uint8_t * data, size_t len, const char * src_ip, uint16_t src_port)
  {
    if (len != kImuPacketBytes) {
      imu_invalid_len_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[imu] invalid packet length=%zu expected=%zu from %s:%u",
        len, kImuPacketBytes, src_ip, src_port);
      return;
    }

    uint32_t ts_ms = 0;
    float vals[6]{};
    std::memcpy(&ts_ms, data, 4);
    std::memcpy(vals, data + 4, 24);

    const float raw_ax = vals[0];
    const float raw_ay = vals[1];
    const float raw_az = vals[2];
    const float raw_gx = vals[3];
    const float raw_gy = vals[4];
    const float raw_gz = vals[5];

    Eigen::Vector3f accel{raw_ax, raw_ay, -raw_az};
    Eigen::Vector3f gyro{raw_gx, raw_gy, raw_gz};

    if (zero_gyro_z_) {
      gyro.z() = 0.0f;
    }
    gyro.z() = -gyro.z();

    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      latest_imu_host_stamp_ = now();
      latest_imu_sensor_ms_ = ts_ms;
      latest_accel_ = accel;
      latest_gyro_ = gyro;
      imu_valid_ = true;
    }

    imu_packets_parsed_.fetch_add(1, std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[imu] parsed packets=%lu latest ts_ms=%u raw_accel=(%.4f, %.4f, %.4f) raw_gyro=(%.4f, %.4f, %.4f) fixed_accel=(%.4f, %.4f, %.4f) fixed_gyro=(%.4f, %.4f, %.4f)",
      static_cast<unsigned long>(imu_packets_parsed_.load(std::memory_order_relaxed)),
      ts_ms,
      raw_ax, raw_ay, raw_az,
      raw_gx, raw_gy, raw_gz,
      accel.x(), accel.y(), accel.z(),
      gyro.x(), gyro.y(), gyro.z());
  }

  void parse_lidar(const uint8_t * data, size_t len, const char * src_ip, uint16_t src_port)
  {
    if (len < kLidarPointBytes) {
      lidar_invalid_len_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[lidar] packet too short len=%zu from %s:%u",
        len, src_ip, src_port);
      return;
    }

    if ((len % kLidarPointBytes) != 0) {
      lidar_invalid_len_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[lidar] packet len=%zu not divisible by %zu from %s:%u",
        len, kLidarPointBytes, src_ip, src_port);
      return;
    }

    const size_t count = len / kLidarPointBytes;
    lidar_packets_parsed_.fetch_add(1, std::memory_order_relaxed);
    lidar_points_raw_.fetch_add(count, std::memory_order_relaxed);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      for (size_t i = 0; i < count; ++i) {
        const uint8_t * p = data + i * kLidarPointBytes;

        uint32_t raw_ts_us = 0;
        uint32_t raw_phi = 0;
        uint32_t raw_angle = 0;
        uint32_t raw_dist = 0;
        std::memcpy(&raw_ts_us, p + 0, 4);
        std::memcpy(&raw_phi, p + 4, 4);
        std::memcpy(&raw_angle, p + 8, 4);
        std::memcpy(&raw_dist, p + 12, 4);
        const uint8_t quality = p[16];

        if (quality < static_cast<uint8_t>(quality_min_)) {
          lidar_points_rejected_quality_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }

        const float dist_m = static_cast<float>(raw_dist) * 0.001f;
        if (dist_m < min_range_ || dist_m > max_range_) {
          lidar_points_rejected_range_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }

        const float phi_deg = static_cast<float>(raw_phi) * 0.01f;
        const float az_deg = static_cast<float>(raw_angle) * 0.01f + azimuth_offset_deg_;

        if (verbose_packets_ && i == 0) {
          RCLCPP_INFO(
            get_logger(),
            "[lidar] packet count=%zu first_point ts_us=%u phi_deg=%.2f az_deg=%.2f dist_m=%.3f quality=%u",
            count, raw_ts_us, phi_deg, az_deg, dist_m, static_cast<unsigned>(quality));
        }

        add_point_locked(phi_deg, az_deg, dist_m, static_cast<float>(quality));
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[lidar] parsed packets=%lu raw_points=%lu kept_points=%lu rej_range=%lu rej_quality=%lu live_points=%zu",
      static_cast<unsigned long>(lidar_packets_parsed_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_raw_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_kept_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_rejected_range_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_rejected_quality_.load(std::memory_order_relaxed)),
      live_points_.size());
  }

  void add_point_locked(float phi_deg, float az_deg, float dist_m, float intensity)
  {
    const float az = wrap_deg(az_deg) * kDegToRad;
    const float tilt = (phi_deg - tilt_zero_deg_) * tilt_sign_ * kDegToRad;

    const float ca = std::cos(az);
    const float sa = std::sin(az);
    const float ct = std::cos(tilt);
    const float st = std::sin(tilt);

    const float lx = dist_m * ca;
    const float ly = dist_m * sa;
    const float lz = 0.0f;

    float x = lx;
    float y = ly;
    float z = lz;

    if (tilt_about_x_) {
      y = ly * ct - lz * st;
      z = ly * st + lz * ct;
      x = lx;
    } else {
      x = lx * ct + lz * st;
      z = -lx * st + lz * ct;
      y = ly;
    }

    x += pivot_x_;
    y += pivot_y_;
    z += pivot_z_;

    if (verbose_point_geometry_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[point] phi=%.2f az=%.2f dist=%.3f -> xyz=(%.3f, %.3f, %.3f) intensity=%.1f",
        phi_deg, az_deg, dist_m, x, y, z, intensity);
    }

    if (live_points_.size() < max_points_per_cloud_) {
      live_points_.push_back(PointXYZI{x, y, z, intensity});
      lidar_points_kept_.fetch_add(1, std::memory_order_relaxed);
    } else {
      dropped_points_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Point buffer full; dropping points. max_points_per_cloud=%zu",
        max_points_per_cloud_);
    }

    if (publish_scan_) {
      const float planar_r = std::sqrt(x * x + y * y);
      if (planar_r < min_range_ || planar_r > max_range_) {
        scan_points_rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      float planar_angle = std::atan2(y, x);
      if (planar_angle < 0.0f) {
        planar_angle += kTwoPi;
      }

      int bin = static_cast<int>((planar_angle / kTwoPi) * static_cast<float>(scan_bins_));
      if (bin >= scan_bins_) {
        bin = scan_bins_ - 1;
      }
      if (bin >= 0) {
        auto & cell = scan_accum_[static_cast<size_t>(bin)];
        if (planar_r < cell) {
          cell = planar_r;
        }
      }
    }
  }

  void publish_latest_imu()
  {
    Eigen::Vector3f accel;
    Eigen::Vector3f gyro;
    bool valid = false;
    uint32_t sensor_ms = 0;

    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      valid = imu_valid_;
      if (valid) {
        accel = latest_accel_;
        gyro = latest_gyro_;
        sensor_ms = latest_imu_sensor_ms_;
      }
    }

    if (!valid) {
      imu_publish_skipped_no_data_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[imu_pub] skipping publish; no valid IMU data received yet");
      return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = imu_frame_;

    msg.linear_acceleration.x = accel.x();
    msg.linear_acceleration.y = accel.y();
    msg.linear_acceleration.z = accel.z();

    msg.angular_velocity.x = gyro.x();
    msg.angular_velocity.y = gyro.y();
    msg.angular_velocity.z = gyro.z();

    msg.orientation_covariance[0] = -1.0;

    imu_pub_->publish(std::move(msg));
    imu_published_.fetch_add(1, std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[imu_pub] published=%lu frame=%s sensor_ts_ms=%u accel=(%.4f, %.4f, %.4f) gyro=(%.4f, %.4f, %.4f) subs=%zu",
      static_cast<unsigned long>(imu_published_.load(std::memory_order_relaxed)),
      imu_frame_.c_str(),
      sensor_ms,
      accel.x(), accel.y(), accel.z(),
      gyro.x(), gyro.y(), gyro.z(),
      imu_pub_->get_subscription_count());
  }

  void publish_outputs()
  {
    if (!publish_cloud_ && !publish_scan_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "publish_outputs called but both publish_cloud and publish_scan are false");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      std::swap(live_points_, publish_points_);
      if (publish_scan_) {
        std::swap(scan_accum_, scan_publish_);
        std::fill(scan_accum_.begin(), scan_accum_.end(), std::numeric_limits<float>::infinity());
      }
    }

    if (publish_cloud_) {
      if (!publish_points_.empty()) {
        publish_cloud_msg(publish_points_);
      } else {
        cloud_publish_skipped_empty_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[cloud_pub] skipped publish; publish_points_ empty");
      }
    }

    if (publish_scan_) {
      publish_scan_msg(scan_publish_);
    }

    publish_points_.clear();

    const size_t dropped = dropped_points_.exchange(0, std::memory_order_relaxed);
    if (dropped > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropped %zu points because max_points_per_cloud was reached", dropped);
    }
  }

  void publish_cloud_msg(const std::vector<PointXYZI> & points)
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = cloud_frame_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(points.size());
    msg.fields = point_fields_;
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.point_step = sizeof(PointXYZI);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<size_t>(msg.row_step));

    std::memcpy(msg.data.data(), points.data(), msg.data.size());
    cloud_pub_->publish(std::move(msg));
    cloud_published_.fetch_add(1, std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[cloud_pub] published=%lu points=%zu frame=%s point_step=%u row_step=%u subs=%zu",
      static_cast<unsigned long>(cloud_published_.load(std::memory_order_relaxed)),
      points.size(),
      cloud_frame_.c_str(),
      static_cast<unsigned>(sizeof(PointXYZI)),
      static_cast<unsigned>(sizeof(PointXYZI) * points.size()),
      cloud_pub_->get_subscription_count());
  }

  void publish_scan_msg(const std::vector<float> & bins)
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = scan_frame_;
    msg.angle_min = 0.0f;
    msg.angle_max = kTwoPi;
    msg.angle_increment = kTwoPi / static_cast<float>(scan_bins_);
    msg.time_increment = 0.0f;
    msg.scan_time = static_cast<float>(cloud_publish_ms_) / 1000.0f;
    msg.range_min = min_range_;
    msg.range_max = max_range_;
    msg.ranges = bins;

    size_t finite_count = 0;
    float min_seen = std::numeric_limits<float>::infinity();
    float max_seen = 0.0f;
    for (float r : bins) {
      if (std::isfinite(r)) {
        ++finite_count;
        min_seen = std::min(min_seen, r);
        max_seen = std::max(max_seen, r);
      }
    }

    scan_pub_->publish(std::move(msg));
    scan_published_.fetch_add(1, std::memory_order_relaxed);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[scan_pub] published=%lu bins=%zu finite=%zu min=%.3f max=%.3f frame=%s subs=%zu",
      static_cast<unsigned long>(scan_published_.load(std::memory_order_relaxed)),
      bins.size(),
      finite_count,
      std::isfinite(min_seen) ? min_seen : -1.0f,
      finite_count > 0 ? max_seen : -1.0f,
      scan_frame_.c_str(),
      scan_pub_->get_subscription_count());
  }

  void log_health()
  {
    const auto now_ns = now().nanoseconds();
    const auto last_pkt_ns = last_packet_time_ns_.load(std::memory_order_relaxed);
    const auto last_imu_ns = last_imu_time_ns_.load(std::memory_order_relaxed);
    const auto last_lidar_ns = last_lidar_time_ns_.load(std::memory_order_relaxed);

    const double age_pkt_ms = (last_pkt_ns > 0) ? (now_ns - last_pkt_ns) / 1e6 : -1.0;
    const double age_imu_ms = (last_imu_ns > 0) ? (now_ns - last_imu_ns) / 1e6 : -1.0;
    const double age_lidar_ms = (last_lidar_ns > 0) ? (now_ns - last_lidar_ns) / 1e6 : -1.0;

    size_t live_points_size = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      live_points_size = live_points_.size();
    }

    bool imu_valid = false;
    uint32_t imu_sensor_ms = 0;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_valid = imu_valid_;
      imu_sensor_ms = latest_imu_sensor_ms_;
    }

    const size_t imu_subs = imu_pub_ ? imu_pub_->get_subscription_count() : 0;
    const size_t cloud_subs = cloud_pub_ ? cloud_pub_->get_subscription_count() : 0;
    const size_t scan_subs = scan_pub_ ? scan_pub_->get_subscription_count() : 0;

    RCLCPP_WARN(
      get_logger(),
      "[health] sockets lidar_fd=%d imu_fd=%d | select_calls=%lu timeouts=%lu errors=%lu recv_errors=%lu | "
      "rx imu_pkts=%lu imu_bytes=%lu lidar_pkts=%lu lidar_bytes=%lu | "
      "parse imu_ok=%lu imu_bad_len=%lu lidar_pkts=%lu lidar_raw_pts=%lu lidar_kept=%lu rej_range=%lu rej_quality=%lu lidar_bad_len=%lu | "
      "pub imu=%lu imu_skip=%lu cloud=%lu cloud_skip=%lu scan=%lu | "
      "subs imu=%zu cloud=%zu scan=%zu | "
      "ages last_pkt=%.1fms last_imu=%.1fms last_lidar=%.1fms | "
      "imu_valid=%s imu_sensor_ms=%u live_points=%zu scan_rej=%lu",
      lidar_sock_, imu_sock_,
      static_cast<unsigned long>(select_calls_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(select_timeouts_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(select_errors_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(recv_errors_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(imu_packets_rx_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(imu_bytes_rx_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_packets_rx_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_bytes_rx_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(imu_packets_parsed_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(imu_invalid_len_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_packets_parsed_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_raw_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_kept_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_rejected_range_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_points_rejected_quality_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(lidar_invalid_len_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(imu_published_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(imu_publish_skipped_no_data_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(cloud_published_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(cloud_publish_skipped_empty_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(scan_published_.load(std::memory_order_relaxed)),
      imu_subs, cloud_subs, scan_subs,
      age_pkt_ms, age_imu_ms, age_lidar_ms,
      tf(imu_valid), imu_sensor_ms, live_points_size,
      static_cast<unsigned long>(scan_points_rejected_.load(std::memory_order_relaxed)));
  }

private:
  int lidar_port_{12345};
  int imu_port_{12346};
  int recv_buffer_bytes_{4 * 1024 * 1024};

  bool publish_cloud_{true};
  bool publish_scan_{true};

  int cloud_publish_ms_{80};
  int imu_publish_ms_{5};
  int health_log_ms_{2000};

  bool verbose_startup_{true};
  bool verbose_socket_{true};
  bool verbose_packets_{false};
  bool verbose_point_geometry_{false};

  size_t max_points_per_cloud_{120000};

  float min_range_{0.05f};
  float max_range_{12.0f};
  int quality_min_{0};

  int scan_bins_{3600};
  float azimuth_offset_deg_{0.0f};

  float tilt_zero_deg_{180.0f};
  float tilt_sign_{1.0f};
  bool tilt_about_x_{false};

  float pivot_x_{0.0f};
  float pivot_y_{0.0f};
  float pivot_z_{0.0f};

  bool zero_gyro_z_{true};

  std::string cloud_frame_{"base_link"};
  std::string scan_frame_{"base_link"};
  std::string imu_frame_{"imu_link"};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  int lidar_sock_{-1};
  int imu_sock_{-1};
  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  std::mutex data_mutex_;
  std::vector<PointXYZI> live_points_;
  std::vector<PointXYZI> publish_points_;
  std::vector<float> scan_accum_;
  std::vector<float> scan_publish_;
  std::atomic<size_t> dropped_points_{0};

  std::mutex imu_mutex_;
  bool imu_valid_{false};
  rclcpp::Time latest_imu_host_stamp_{0, 0, RCL_ROS_TIME};
  uint32_t latest_imu_sensor_ms_{0};
  Eigen::Vector3f latest_accel_{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f latest_gyro_{0.0f, 0.0f, 0.0f};

  std::vector<sensor_msgs::msg::PointField> point_fields_;

  std::atomic<uint64_t> select_calls_{0};
  std::atomic<uint64_t> select_timeouts_{0};
  std::atomic<uint64_t> select_errors_{0};
  std::atomic<uint64_t> recv_errors_{0};

  std::atomic<uint64_t> lidar_ready_events_{0};
  std::atomic<uint64_t> imu_ready_events_{0};

  std::atomic<uint64_t> imu_packets_rx_{0};
  std::atomic<uint64_t> imu_bytes_rx_{0};
  std::atomic<uint64_t> imu_packets_parsed_{0};
  std::atomic<uint64_t> imu_invalid_len_{0};

  std::atomic<uint64_t> lidar_packets_rx_{0};
  std::atomic<uint64_t> lidar_bytes_rx_{0};
  std::atomic<uint64_t> lidar_packets_parsed_{0};
  std::atomic<uint64_t> lidar_invalid_len_{0};
  std::atomic<uint64_t> lidar_points_raw_{0};
  std::atomic<uint64_t> lidar_points_kept_{0};
  std::atomic<uint64_t> lidar_points_rejected_range_{0};
  std::atomic<uint64_t> lidar_points_rejected_quality_{0};

  std::atomic<uint64_t> imu_published_{0};
  std::atomic<uint64_t> imu_publish_skipped_no_data_{0};
  std::atomic<uint64_t> cloud_published_{0};
  std::atomic<uint64_t> cloud_publish_skipped_empty_{0};
  std::atomic<uint64_t> scan_published_{0};
  std::atomic<uint64_t> scan_points_rejected_{0};

  std::atomic<int64_t> last_packet_time_ns_{0};
  std::atomic<int64_t> last_imu_time_ns_{0};
  std::atomic<int64_t> last_lidar_time_ns_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPReader>());
  rclcpp::shutdown();
  return 0;
}
