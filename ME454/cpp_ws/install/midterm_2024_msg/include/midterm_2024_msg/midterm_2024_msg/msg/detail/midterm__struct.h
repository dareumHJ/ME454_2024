// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from midterm_2024_msg:msg/Midterm.idl
// generated code does not contain a copyright notice

#ifndef MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__STRUCT_H_
#define MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/Midterm in the package midterm_2024_msg.
typedef struct midterm_2024_msg__msg__Midterm
{
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Quaternion quaternion;
} midterm_2024_msg__msg__Midterm;

// Struct for a sequence of midterm_2024_msg__msg__Midterm.
typedef struct midterm_2024_msg__msg__Midterm__Sequence
{
  midterm_2024_msg__msg__Midterm * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} midterm_2024_msg__msg__Midterm__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__STRUCT_H_
