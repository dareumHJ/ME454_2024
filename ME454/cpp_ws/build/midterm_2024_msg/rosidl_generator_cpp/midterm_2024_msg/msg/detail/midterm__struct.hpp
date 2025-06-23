// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from midterm_2024_msg:msg/Midterm.idl
// generated code does not contain a copyright notice

#ifndef MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__STRUCT_HPP_
#define MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__midterm_2024_msg__msg__Midterm __attribute__((deprecated))
#else
# define DEPRECATED__midterm_2024_msg__msg__Midterm __declspec(deprecated)
#endif

namespace midterm_2024_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Midterm_
{
  using Type = Midterm_<ContainerAllocator>;

  explicit Midterm_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : angular_velocity(_init),
    position(_init),
    quaternion(_init)
  {
    (void)_init;
  }

  explicit Midterm_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : angular_velocity(_alloc, _init),
    position(_alloc, _init),
    quaternion(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _quaternion_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _quaternion_type quaternion;

  // setters for named parameter idiom
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__quaternion(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    midterm_2024_msg::msg::Midterm_<ContainerAllocator> *;
  using ConstRawPtr =
    const midterm_2024_msg::msg::Midterm_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      midterm_2024_msg::msg::Midterm_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      midterm_2024_msg::msg::Midterm_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__midterm_2024_msg__msg__Midterm
    std::shared_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__midterm_2024_msg__msg__Midterm
    std::shared_ptr<midterm_2024_msg::msg::Midterm_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Midterm_ & other) const
  {
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->quaternion != other.quaternion) {
      return false;
    }
    return true;
  }
  bool operator!=(const Midterm_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Midterm_

// alias to use template instance with default allocator
using Midterm =
  midterm_2024_msg::msg::Midterm_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace midterm_2024_msg

#endif  // MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__STRUCT_HPP_
