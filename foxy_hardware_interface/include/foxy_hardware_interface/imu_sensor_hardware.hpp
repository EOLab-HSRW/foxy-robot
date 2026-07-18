#ifndef FOXY_HARDWARE_INTERFACE__IMU_SENSOR_HARDWARE_HPP_
#define FOXY_HARDWARE_INTERFACE__IMU_SENSOR_HARDWARE_HPP_

#include <array>
#include <string>
#include <vector>

#include "foxy/foxy.h"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace foxy_hardware_interface
{


class FoxyImuSensorHardware : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FoxyImuSensorHardware);


  FoxyImuSensorHardware() = default;
  ~FoxyImuSensorHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(

    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:

  void reset_values() noexcept;
  void close_driver() noexcept;

  std::string sensor_name_;
  std::string resource_name_{"imu0"};

  robot_t robot_{};
  imu_t imu_{};

  bool robot_initialized_{false};
  bool imu_initialized_{false};

  /*
   * The Humble imu_sensor_broadcaster requires orientation interfaces.
   *
   * The current foxy-drivers MPU6050 implementation provides acceleration
   * and angular velocity, but it does not calculate orientation. Therefore,

   * an identity quaternion is exported as a placeholder.
   *
   * The broadcaster configuration must set:
   *
   *   static_covariance_orientation:
   *     [-1.0, 0.0, 0.0,

   *       0.0, 0.0, 0.0,
   *       0.0, 0.0, 0.0]
   *
   * This marks orientation as unavailable in sensor_msgs/msg/Imu.
   */
  std::array<double, 4> orientation_{
    {0.0, 0.0, 0.0, 1.0}
  };

  std::array<double, 3> angular_velocity_{
    {0.0, 0.0, 0.0}
  };

  std::array<double, 3> linear_acceleration_{
    {0.0, 0.0, 0.0}

  };

};

}  // namespace foxy_hardware_interface

#endif  // FOXY_HARDWARE_INTERFACE__IMU_SENSOR_HARDWARE_HPP_
