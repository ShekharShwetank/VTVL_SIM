// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vtvl_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
#define VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/VehicleState in the package vtvl_msgs.
/**
  * VehicleState.msg: rocket state vector (positions, velocities, attitude, rates)
 */
typedef struct vtvl_msgs__msg__VehicleState
{
  /// Header to track timestamp and frame ID
  std_msgs__msg__Header header;
  /// Translational State in Inertial Frame
  /// Inertial position (x, y, z) in meters
  geometry_msgs__msg__Point position;
  /// Inertial velocity (vx, vy, vz) in m/s
  geometry_msgs__msg__Vector3 velocity;
  /// Rotational State
  /// Orientation as a quaternion (x, y, z, w)
  geometry_msgs__msg__Quaternion attitude;
  /// Body frame angular rates (wx, wy, wz) in rad/s
  geometry_msgs__msg__Vector3 angular_velocity;
  /// Mass Properties
  /// Current total mass in kg
  double mass;
} vtvl_msgs__msg__VehicleState;

// Struct for a sequence of vtvl_msgs__msg__VehicleState.
typedef struct vtvl_msgs__msg__VehicleState__Sequence
{
  vtvl_msgs__msg__VehicleState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vtvl_msgs__msg__VehicleState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VTVL_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
