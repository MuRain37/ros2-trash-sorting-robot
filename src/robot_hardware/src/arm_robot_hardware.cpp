#include "robot_hardware/arm_robot_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "arm_position_cmd";
constexpr auto DEFAULT_STATE_TOPIC = "arm_position_fb";
constexpr std::size_t DEFAULT_REAL_JOINT_COUNT = 5;
}  // namespace

namespace robot_hardware
{
hardware_interface::CallbackReturn ArmRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_topic_ = DEFAULT_COMMAND_TOPIC;
  state_topic_ = DEFAULT_STATE_TOPIC;
  real_joint_count_ = DEFAULT_REAL_JOINT_COUNT;

  const auto command_topic = info_.hardware_parameters.find("command_topic");
  if (command_topic != info_.hardware_parameters.end() && !command_topic->second.empty())
  {
    command_topic_ = command_topic->second;
  }

  const auto state_topic = info_.hardware_parameters.find("state_topic");
  if (state_topic != info_.hardware_parameters.end() && !state_topic->second.empty())
  {
    state_topic_ = state_topic->second;
  }

  const auto real_joint_count = info_.hardware_parameters.find("real_joint_count");
  if (real_joint_count != info_.hardware_parameters.end() && !real_joint_count->second.empty())
  {
    try
    {
      real_joint_count_ = static_cast<std::size_t>(std::stoul(real_joint_count->second));
    }
    catch (const std::exception &)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArmRobotHardware"),
        "Invalid real_joint_count value '%s'.",
        real_joint_count->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (info_.joints.empty())
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArmRobotHardware"), "No arm joints are configured.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (real_joint_count_ > info_.joints.size())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("ArmRobotHardware"),
      "real_joint_count (%zu) is greater than configured joints (%zu).",
      real_joint_count_, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  last_positions_.resize(info_.joints.size(), 0.0);

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    if (!joint_has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArmRobotHardware"),
        "Joint '%s' must expose a position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (
      !joint_has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !joint_has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArmRobotHardware"),
        "Joint '%s' must expose position and velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const double initial_position = initial_position_for_joint(joint);
    hw_positions_[i] = initial_position;
    last_positions_[i] = initial_position;
    hw_commands_[i] = initial_position;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArmRobotHardware"),
    "Configured %zu arm joints (%zu real joints), command_topic=%s, state_topic=%s",
    info_.joints.size(), real_joint_count_, command_topic_.c_str(), state_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArmRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmRobotHardware"), "Activating arm hardware interface.");

  for (std::size_t i = 0; i < hw_positions_.size(); ++i)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
    }
    if (std::isnan(hw_velocities_[i]))
    {
      hw_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = hw_positions_[i];
    }
    last_positions_[i] = hw_positions_[i];
  }

  subscriber_is_active_ = true;
  node_ = std::make_shared<rclcpp::Node>("arm_hardware_node");
  received_fb_msg_ptr_.set(nullptr);

  fb_subscriber_ =
    node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    state_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(node_->get_logger(), "Ignoring arm feedback while inactive.");
        return;
      }
      received_fb_msg_ptr_.set(std::move(msg));
    });

  cmd_publisher_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    command_topic_, rclcpp::SystemDefaultsQoS());
  realtime_cmd_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    cmd_publisher_);

  RCLCPP_INFO(rclcpp::get_logger("ArmRobotHardware"), "Arm hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmRobotHardware"), "Deactivating arm hardware interface.");
  subscriber_is_active_ = false;
  fb_subscriber_.reset();
  cmd_publisher_.reset();
  realtime_cmd_publisher_.reset();
  received_fb_msg_ptr_.set(nullptr);
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (node_)
  {
    rclcpp::spin_some(node_);
  }

  std::shared_ptr<std_msgs::msg::Float64MultiArray> fb_msg;
  received_fb_msg_ptr_.get(fb_msg);
  if (fb_msg)
  {
    // 真实关节使用硬件桥接节点反馈；数量不足时只更新已有部分。
    const std::size_t feedback_count =
      std::min(real_joint_count_, static_cast<std::size_t>(fb_msg->data.size()));
    if (feedback_count < real_joint_count_)
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ArmRobotHardware"),
        *node_->get_clock(),
        5000,
        "Arm feedback has %zu values, expected %zu.",
        fb_msg->data.size(), real_joint_count_);
    }

    const double dt = period.seconds();
    for (std::size_t i = 0; i < feedback_count; ++i)
    {
      const double position = fb_msg->data[i];
      // 反馈只有位置，速度由两次位置差估算。
      hw_velocities_[i] = dt > 0.0 ? (position - last_positions_[i]) / dt : 0.0;
      hw_positions_[i] = position;
      last_positions_[i] = position;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  // 虚拟末端关节没有真实硬件，直接跟随控制器命令更新状态。
  for (std::size_t i = real_joint_count_; i < hw_commands_.size(); ++i)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      hw_velocities_[i] = dt > 0.0 ? (hw_commands_[i] - hw_positions_[i]) / dt : 0.0;
      hw_positions_[i] = hw_commands_[i];
      last_positions_[i] = hw_positions_[i];
    }
  }

  if (realtime_cmd_publisher_ && realtime_cmd_publisher_->trylock())
  {
    auto & cmd_msg = realtime_cmd_publisher_->msg_;
    cmd_msg.data.resize(real_joint_count_);
    // 只发送真实关节命令，虚拟关节留给 MoveIt 约束使用。
    for (std::size_t i = 0; i < real_joint_count_; ++i)
    {
      cmd_msg.data[i] = hw_commands_[i];
    }
    realtime_cmd_publisher_->unlockAndPublish();
  }

  return hardware_interface::return_type::OK;
}

double ArmRobotHardware::initial_position_for_joint(
  const hardware_interface::ComponentInfo & joint) const
{
  for (const auto & state_interface : joint.state_interfaces)
  {
    if (state_interface.name != hardware_interface::HW_IF_POSITION)
    {
      continue;
    }

    if (state_interface.initial_value.empty())
    {
      return 0.0;
    }

    try
    {
      return std::stod(state_interface.initial_value);
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ArmRobotHardware"),
        "Invalid initial_value '%s' for joint '%s'. Using 0.0.",
        state_interface.initial_value.c_str(), joint.name.c_str());
      return 0.0;
    }
  }
  return 0.0;
}

bool ArmRobotHardware::joint_has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & name) const
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & interface_info)
    {
      return interface_info.name == name;
    });
}
}  // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(
  robot_hardware::ArmRobotHardware, hardware_interface::SystemInterface)
