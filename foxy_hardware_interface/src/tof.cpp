#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define FOXY_IMPLEMENTATION
#include "foxy/foxy.h"

class TofLaserScan : public rclcpp::Node
{
public:
  TofLaserScan()
  : Node("tof")
  {
    sensor_name_ =
      this->declare_parameter<std::string>("sensor_name", "tof0");

    frame_id_ =
      this->declare_parameter<std::string>("frame_id", "tof/front_link");

    topic_name_ =
      this->declare_parameter<std::string>("topic_name", "tof/front/range");

    publish_rate_hz_ =
      this->declare_parameter<double>("publish_rate", 30.0);

    range_min_m_ =
      this->declare_parameter<double>("range_min", 0.03);

    range_max_m_ =
      this->declare_parameter<double>("range_max", 2.0);

    validate_parameters();

    // Create the ROS resources before opening the hardware. This reduces the
    // amount of manual cleanup required if ROS resource creation throws.
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      topic_name_,
      rclcpp::SensorDataQoS());

    robot_ = robot_init();

    if (!robot_ok(&robot_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to initialize the robot description");

      throw std::runtime_error(
              "Failed to initialize the robot description");
    }

    robot_initialized_ = true;

    tof_ = tof_init_name(&robot_, sensor_name_.c_str());

    if (!tof_.ctx) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to initialize ToF device '%s'",
        sensor_name_.c_str());

      cleanup_hardware();

      throw std::runtime_error(
              "Failed to initialize ToF device '" + sensor_name_ + "'");
    }

    tof_initialized_ = true;

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));

    try {
      timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&TofLaserScan::timer_callback, this));
    } catch (...) {
      cleanup_hardware();
      throw;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing ToF device '%s' on '%s' with frame '%s' at %.1f Hz",
      sensor_name_.c_str(),
      topic_name_.c_str(),
      frame_id_.c_str(),
      publish_rate_hz_);
  }

  ~TofLaserScan() override
  {
    timer_.reset();
    cleanup_hardware();
  }

private:
  void validate_parameters() const
  {
    if (sensor_name_.empty()) {
      throw std::invalid_argument("sensor_name must not be empty");
    }

    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }

    if (topic_name_.empty()) {
      throw std::invalid_argument("topic_name must not be empty");
    }

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument(
              "publish_rate must be a positive finite value");
    }

    if (!std::isfinite(range_min_m_) || range_min_m_ < 0.0) {
      throw std::invalid_argument(
              "range_min must be a non-negative finite value");
    }

    if (!std::isfinite(range_max_m_) || range_max_m_ <= range_min_m_) {
      throw std::invalid_argument(
              "range_max must be finite and greater than range_min");
    }
  }

  void timer_callback()
  {
    // Timestamp immediately before the blocking hardware read. LaserScan
    // defines this as the acquisition time of the first ray.
    const auto acquisition_time = this->now();

    const int distance_mm = tof_read_mm(tof_);

    if (distance_mm < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to read ToF device '%s': error %d",
        sensor_name_.c_str(),
        distance_mm);

      // Do not publish an apparently valid scan for a driver-level failure.
      return;
    }

    const float distance_m =
      static_cast<float>(distance_mm) * 0.001F;

    sensor_msgs::msg::LaserScan message;

    message.header.stamp = acquisition_time;
    message.header.frame_id = frame_id_;

    // The VL53L0X provides one central measurement rather than an angular
    // sweep. Do not copy the same measurement across a fabricated field of
    // view.
    message.angle_min = 0.0F;
    message.angle_max = 0.0F;
    message.angle_increment = 0.0F;

    message.time_increment = 0.0F;
    message.scan_time =
      static_cast<float>(1.0 / publish_rate_hz_);

    message.range_min = static_cast<float>(range_min_m_);
    message.range_max = static_cast<float>(range_max_m_);

    if (distance_m < message.range_min) {
      // Common LaserScan convention for an object closer than the minimum
      // measurable distance.
      message.ranges.push_back(
        -std::numeric_limits<float>::infinity());
    } else if (distance_m > message.range_max) {
      // Common LaserScan convention for no return inside the usable range.
      message.ranges.push_back(
        std::numeric_limits<float>::infinity());
    } else {
      message.ranges.push_back(distance_m);
    }

    // The device does not provide an intensity measurement, so
    // message.intensities remains empty.
    publisher_->publish(message);
  }

  void cleanup_hardware() noexcept
  {
    if (tof_initialized_) {
      tof_deinit(&robot_, &tof_);
      tof_initialized_ = false;
    }

    if (robot_initialized_) {
      robot_deinit(&robot_);
      robot_initialized_ = false;
    }
  }

  std::string sensor_name_;
  std::string frame_id_;
  std::string topic_name_;

  double publish_rate_hz_{30.0};
  double range_min_m_{0.03};
  double range_max_m_{2.0};

  robot_t robot_{};
  tof_t tof_{};

  bool robot_initialized_{false};
  bool tof_initialized_{false};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int return_code = 0;

  try {
    rclcpp::spin(std::make_shared<TofLaserScan>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("tof"),
      "ToF node terminated during initialization: %s",
      exception.what());

    return_code = 1;
  }

  rclcpp::shutdown();
  return return_code;
}
