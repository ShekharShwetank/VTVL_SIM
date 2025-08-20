// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vtvl_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
#define VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vtvl_msgs/msg/detail/vehicle_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vtvl_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleState_mass
{
public:
  explicit Init_VehicleState_mass(::vtvl_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  ::vtvl_msgs::msg::VehicleState mass(::vtvl_msgs::msg::VehicleState::_mass_type arg)
  {
    msg_.mass = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vtvl_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_angular_velocity
{
public:
  explicit Init_VehicleState_angular_velocity(::vtvl_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_mass angular_velocity(::vtvl_msgs::msg::VehicleState::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_VehicleState_mass(msg_);
  }

private:
  ::vtvl_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_attitude
{
public:
  explicit Init_VehicleState_attitude(::vtvl_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_angular_velocity attitude(::vtvl_msgs::msg::VehicleState::_attitude_type arg)
  {
    msg_.attitude = std::move(arg);
    return Init_VehicleState_angular_velocity(msg_);
  }

private:
  ::vtvl_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_velocity
{
public:
  explicit Init_VehicleState_velocity(::vtvl_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_attitude velocity(::vtvl_msgs::msg::VehicleState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_VehicleState_attitude(msg_);
  }

private:
  ::vtvl_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_position
{
public:
  explicit Init_VehicleState_position(::vtvl_msgs::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_velocity position(::vtvl_msgs::msg::VehicleState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_VehicleState_velocity(msg_);
  }

private:
  ::vtvl_msgs::msg::VehicleState msg_;
};

class Init_VehicleState_header
{
public:
  Init_VehicleState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleState_position header(::vtvl_msgs::msg::VehicleState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VehicleState_position(msg_);
  }

private:
  ::vtvl_msgs::msg::VehicleState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vtvl_msgs::msg::VehicleState>()
{
  return vtvl_msgs::msg::builder::Init_VehicleState_header();
}

}  // namespace vtvl_msgs

#endif  // VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
