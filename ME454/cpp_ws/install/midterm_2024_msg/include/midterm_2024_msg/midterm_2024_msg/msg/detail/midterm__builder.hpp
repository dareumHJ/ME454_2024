// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from midterm_2024_msg:msg/Midterm.idl
// generated code does not contain a copyright notice

#ifndef MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__BUILDER_HPP_
#define MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "midterm_2024_msg/msg/detail/midterm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace midterm_2024_msg
{

namespace msg
{

namespace builder
{

class Init_Midterm_quaternion
{
public:
  explicit Init_Midterm_quaternion(::midterm_2024_msg::msg::Midterm & msg)
  : msg_(msg)
  {}
  ::midterm_2024_msg::msg::Midterm quaternion(::midterm_2024_msg::msg::Midterm::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return std::move(msg_);
  }

private:
  ::midterm_2024_msg::msg::Midterm msg_;
};

class Init_Midterm_position
{
public:
  explicit Init_Midterm_position(::midterm_2024_msg::msg::Midterm & msg)
  : msg_(msg)
  {}
  Init_Midterm_quaternion position(::midterm_2024_msg::msg::Midterm::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Midterm_quaternion(msg_);
  }

private:
  ::midterm_2024_msg::msg::Midterm msg_;
};

class Init_Midterm_angular_velocity
{
public:
  Init_Midterm_angular_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Midterm_position angular_velocity(::midterm_2024_msg::msg::Midterm::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_Midterm_position(msg_);
  }

private:
  ::midterm_2024_msg::msg::Midterm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::midterm_2024_msg::msg::Midterm>()
{
  return midterm_2024_msg::msg::builder::Init_Midterm_angular_velocity();
}

}  // namespace midterm_2024_msg

#endif  // MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__BUILDER_HPP_
