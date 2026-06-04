#ifndef ARM_ROBOT_HARDWARE_HPP_
#define ARM_ROBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "robot_description/visibility_control.h"

namespace robot_description
{
class ArmRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmRobotHardware)

  ROBOT_DESCRIPTION_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROBOT_DESCRIPTION_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOT_DESCRIPTION_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOT_DESCRIPTION_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_DESCRIPTION_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_DESCRIPTION_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROBOT_DESCRIPTION_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 机械臂 ros2_control 硬件接口：前 real_joint_count_ 个关节走真实话题反馈。
  double initial_position_for_joint(const hardware_interface::ComponentInfo & joint) const;
  bool joint_has_interface(
    const std::vector<hardware_interface::InterfaceInfo> & interfaces,
    const std::string & name) const;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> last_positions_;

  std::shared_ptr<rclcpp::Node> node_;
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fb_subscriber_ = nullptr;
  realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64MultiArray>>
    received_fb_msg_ptr_{nullptr};
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    realtime_cmd_publisher_ = nullptr;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> cmd_publisher_ = nullptr;

  std::string command_topic_ = "arm_position_cmd";
  std::string state_topic_ = "arm_position_fb";
  std::size_t real_joint_count_ = 5;
};
}  // namespace robot_description

#endif  // ARM_ROBOT_HARDWARE_HPP_
