/**
 *****************************Copyright (c) 2023  ZJU****************************
 * @file      : diff_robot_hardware.cpp
 * @brief     : 四轮差速机器人硬件接口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0    2023-07-18       Hao Lion        1. 基于 diff_test_system 移植
 *  V2.0.0    2026-04-16       MuRain37        2. 修改为四轮差速驱动
 *******************************************************************************
 */

#include "robot_description/diff_robot_hardware.hpp"

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
constexpr auto DEFAULT_STATE_TOPIC = "diff_test_cmd";

}  // namespace

namespace robot_description
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

  // 四轮机器人需要 4 个 joint
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
    // Each joint has one velocity command interface and two state interfaces (position, velocity)
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

  // set some default values
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

  // create realtime publisher
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
  // Open-loop mode: directly use commands as velocities (no external feedback needed)
  // Left wheel velocity (applied to both front_left and rear_left)
  double left_velocity = hw_commands_[0];
  // Right wheel velocity (applied to both front_right and rear_right)
  double right_velocity = hw_commands_[1];

  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("DiffRobotHardware"), *this->node_->get_clock(), 1000,
    "read() - left_vel: %.3f, right_vel: %.3f", left_velocity, right_velocity);

  hw_velocities_[0] = left_velocity;   // front_left_wheel_joint
  hw_velocities_[1] = right_velocity;  // front_right_wheel_joint
  hw_velocities_[2] = left_velocity;  // rear_left_wheel_joint
  hw_velocities_[3] = right_velocity; // rear_right_wheel_joint

  // integrate velocity to position
  for (std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] += period.seconds() * hw_velocities_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Open-loop mode: nothing to publish, commands are directly used in read()
  return hardware_interface::return_type::OK;
}

}  // namespace robot_description

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_description::DiffRobotHardware, hardware_interface::SystemInterface)