#include "robot_hardware/diff_robot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "diff_test_cmd";
constexpr auto DEFAULT_STATE_TOPIC = "diff_test_fb";

}  // namespace

namespace robot_hardware
{
hardware_interface::CallbackReturn DiffRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 四轮底盘固定使用 4 个轮关节。
  if (info_.joints.size() != 4)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffRobotHardware"),
      "The number of joints is set not correctly, please check your urdf. 4 is expected, but now it is %zu.", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(
      rclcpp::get_logger("DiffRobotHardware"), "Found 4 joints successfully!");
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // 每个轮关节需要速度命令、位置状态和速度状态接口。
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffRobotHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffRobotHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffRobotHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffRobotHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffRobotHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Activating ...please wait...");

  // 激活时初始化状态缓存。
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
  subscriber_is_active_ = true;

  this->node_ = std::make_shared<rclcpp::Node>("hardware_node");

  std_msgs::msg::Float64MultiArray empty_int16array;
  for(std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    empty_int16array.data.push_back(0.0);
  }
  received_fb_msg_ptr_.set(std::make_shared<std_msgs::msg::Float64MultiArray>(empty_int16array));

  // 订阅硬件桥接节点发布的轮子反馈。
  fb_subscriber_ =
    this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    DEFAULT_STATE_TOPIC, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(
        this->node_->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      received_fb_msg_ptr_.set(std::move(msg));
    });

  // 发布控制器写入的轮速命令。
  cmd_publisher = this->node_->create_publisher<std_msgs::msg::Float64MultiArray>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_cmd_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(cmd_publisher);

  RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Deactivating ...please wait...");

  subscriber_is_active_ = false;
  fb_subscriber_.reset();
  received_fb_msg_ptr_.set(nullptr);

  RCLCPP_INFO(rclcpp::get_logger("DiffRobotHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::shared_ptr<std_msgs::msg::Float64MultiArray> fb_msg;
  received_fb_msg_ptr_.get(fb_msg);
  rclcpp::spin_some(this->node_);

  for (std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    if (i < hw_velocities_.size())
    {
      // 反馈话题给出轮速；ros2_control 的 position 状态用周期积分得到。
      hw_velocities_[i] = fb_msg->data[i];
      hw_positions_[i] += period.seconds() * hw_velocities_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (realtime_cmd_publisher_->trylock())
  {
    auto & cmd_msg = realtime_cmd_publisher_->msg_;
    cmd_msg.data.resize(hw_commands_.size());
    // 控制器写入的速度命令直接转发给 Python 硬件桥接节点。
    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      cmd_msg.data[i] = hw_commands_[i];
    }
    realtime_cmd_publisher_->unlockAndPublish();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_hardware::DiffRobotHardware, hardware_interface::SystemInterface)
