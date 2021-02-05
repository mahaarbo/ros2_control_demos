#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_MULTI_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"

using hardware_interface::return_type;

namespace ros2_control_demo_hardware
{
class RRBotSystemMultiInterfaceHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemMultiInterfaceHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type start() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type stop() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type write() override;

  /**
   * Check and change behavior depending on the command interface
   * that will be claimed. Under position control we set the
   * position directly, and under velocity, or acceleration we
   * integrate once or twice respectively.
   * 
   * \param key string identifying interface to be checked
   * \return return_type::OK if the resource can be claimed or 
   * return_type::ERROR if the hardware_interface is unable to 
   * accept the change.
   */
  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  return_type accept_command_resource_claim(const std::vector<std::string> & interfaces) override;

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the commands for the simulated robot
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_accelerations_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_lvl_t : std::uint8_t
  {
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2
  };

  std::vector<integration_lvl_t> control_lvl_;

};

}  // namespace ros2_control_demo_hardware
#endif // ROS2_CONTROL_DEMO_HARDWARE__RRBOT_MULTI_INTERFACE_HPP_