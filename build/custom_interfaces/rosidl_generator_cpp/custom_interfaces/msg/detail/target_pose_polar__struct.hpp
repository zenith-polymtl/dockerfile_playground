// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/TargetPosePolar.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__TargetPosePolar __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__TargetPosePolar __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetPosePolar_
{
  using Type = TargetPosePolar_<ContainerAllocator>;

  explicit TargetPosePolar_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0.0f;
      this->theta = 0.0f;
      this->z = 0.0f;
      this->relative = false;
      this->v_r = 0.0f;
      this->v_theta = 0.0f;
    }
  }

  explicit TargetPosePolar_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0.0f;
      this->theta = 0.0f;
      this->z = 0.0f;
      this->relative = false;
      this->v_r = 0.0f;
      this->v_theta = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _r_type =
    float;
  _r_type r;
  using _theta_type =
    float;
  _theta_type theta;
  using _z_type =
    float;
  _z_type z;
  using _relative_type =
    bool;
  _relative_type relative;
  using _v_r_type =
    float;
  _v_r_type v_r;
  using _v_theta_type =
    float;
  _v_theta_type v_theta;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__theta(
    const float & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__relative(
    const bool & _arg)
  {
    this->relative = _arg;
    return *this;
  }
  Type & set__v_r(
    const float & _arg)
  {
    this->v_r = _arg;
    return *this;
  }
  Type & set__v_theta(
    const float & _arg)
  {
    this->v_theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::TargetPosePolar_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::TargetPosePolar_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::TargetPosePolar_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::TargetPosePolar_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__TargetPosePolar
    std::shared_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__TargetPosePolar
    std::shared_ptr<custom_interfaces::msg::TargetPosePolar_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetPosePolar_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->relative != other.relative) {
      return false;
    }
    if (this->v_r != other.v_r) {
      return false;
    }
    if (this->v_theta != other.v_theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetPosePolar_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetPosePolar_

// alias to use template instance with default allocator
using TargetPosePolar =
  custom_interfaces::msg::TargetPosePolar_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__STRUCT_HPP_
