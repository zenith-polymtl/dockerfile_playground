// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/TargetPosePolar.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/target_pose_polar__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetPosePolar_v_theta
{
public:
  explicit Init_TargetPosePolar_v_theta(::custom_interfaces::msg::TargetPosePolar & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::TargetPosePolar v_theta(::custom_interfaces::msg::TargetPosePolar::_v_theta_type arg)
  {
    msg_.v_theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

class Init_TargetPosePolar_v_r
{
public:
  explicit Init_TargetPosePolar_v_r(::custom_interfaces::msg::TargetPosePolar & msg)
  : msg_(msg)
  {}
  Init_TargetPosePolar_v_theta v_r(::custom_interfaces::msg::TargetPosePolar::_v_r_type arg)
  {
    msg_.v_r = std::move(arg);
    return Init_TargetPosePolar_v_theta(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

class Init_TargetPosePolar_relative
{
public:
  explicit Init_TargetPosePolar_relative(::custom_interfaces::msg::TargetPosePolar & msg)
  : msg_(msg)
  {}
  Init_TargetPosePolar_v_r relative(::custom_interfaces::msg::TargetPosePolar::_relative_type arg)
  {
    msg_.relative = std::move(arg);
    return Init_TargetPosePolar_v_r(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

class Init_TargetPosePolar_z
{
public:
  explicit Init_TargetPosePolar_z(::custom_interfaces::msg::TargetPosePolar & msg)
  : msg_(msg)
  {}
  Init_TargetPosePolar_relative z(::custom_interfaces::msg::TargetPosePolar::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_TargetPosePolar_relative(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

class Init_TargetPosePolar_theta
{
public:
  explicit Init_TargetPosePolar_theta(::custom_interfaces::msg::TargetPosePolar & msg)
  : msg_(msg)
  {}
  Init_TargetPosePolar_z theta(::custom_interfaces::msg::TargetPosePolar::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_TargetPosePolar_z(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

class Init_TargetPosePolar_r
{
public:
  explicit Init_TargetPosePolar_r(::custom_interfaces::msg::TargetPosePolar & msg)
  : msg_(msg)
  {}
  Init_TargetPosePolar_theta r(::custom_interfaces::msg::TargetPosePolar::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_TargetPosePolar_theta(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

class Init_TargetPosePolar_header
{
public:
  Init_TargetPosePolar_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetPosePolar_r header(::custom_interfaces::msg::TargetPosePolar::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TargetPosePolar_r(msg_);
  }

private:
  ::custom_interfaces::msg::TargetPosePolar msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::TargetPosePolar>()
{
  return custom_interfaces::msg::builder::Init_TargetPosePolar_header();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__BUILDER_HPP_
