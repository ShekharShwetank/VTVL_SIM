// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vtvl_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__TRAITS_HPP_
#define VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vtvl_msgs/msg/detail/vehicle_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace vtvl_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: attitude
  {
    out << "attitude: ";
    to_flow_style_yaml(msg.attitude, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: mass
  {
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: attitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attitude:\n";
    to_block_style_yaml(msg.attitude, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }

  // member: mass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace vtvl_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vtvl_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vtvl_msgs::msg::VehicleState & msg,
  std::ostream & out, size_t indentation = 0)
{
  vtvl_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vtvl_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vtvl_msgs::msg::VehicleState & msg)
{
  return vtvl_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vtvl_msgs::msg::VehicleState>()
{
  return "vtvl_msgs::msg::VehicleState";
}

template<>
inline const char * name<vtvl_msgs::msg::VehicleState>()
{
  return "vtvl_msgs/msg/VehicleState";
}

template<>
struct has_fixed_size<vtvl_msgs::msg::VehicleState>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<vtvl_msgs::msg::VehicleState>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<vtvl_msgs::msg::VehicleState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__TRAITS_HPP_
