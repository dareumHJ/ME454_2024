// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from midterm_2024_msg:msg/Midterm.idl
// generated code does not contain a copyright notice

#ifndef MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__TRAITS_HPP_
#define MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "midterm_2024_msg/msg/detail/midterm__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace midterm_2024_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Midterm & msg,
  std::ostream & out)
{
  out << "{";
  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: quaternion
  {
    out << "quaternion: ";
    to_flow_style_yaml(msg.quaternion, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Midterm & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: quaternion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion:\n";
    to_block_style_yaml(msg.quaternion, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Midterm & msg, bool use_flow_style = false)
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

}  // namespace midterm_2024_msg

namespace rosidl_generator_traits
{

[[deprecated("use midterm_2024_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const midterm_2024_msg::msg::Midterm & msg,
  std::ostream & out, size_t indentation = 0)
{
  midterm_2024_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use midterm_2024_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const midterm_2024_msg::msg::Midterm & msg)
{
  return midterm_2024_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<midterm_2024_msg::msg::Midterm>()
{
  return "midterm_2024_msg::msg::Midterm";
}

template<>
inline const char * name<midterm_2024_msg::msg::Midterm>()
{
  return "midterm_2024_msg/msg/Midterm";
}

template<>
struct has_fixed_size<midterm_2024_msg::msg::Midterm>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<midterm_2024_msg::msg::Midterm>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<midterm_2024_msg::msg::Midterm>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__TRAITS_HPP_
