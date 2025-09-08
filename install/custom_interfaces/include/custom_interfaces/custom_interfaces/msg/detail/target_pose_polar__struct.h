// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/TargetPosePolar.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__STRUCT_H_

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

/// Struct defined in msg/TargetPosePolar in the package custom_interfaces.
typedef struct custom_interfaces__msg__TargetPosePolar
{
  std_msgs__msg__Header header;
  float r;
  float theta;
  float z;
  bool relative;
  float v_r;
  float v_theta;
} custom_interfaces__msg__TargetPosePolar;

// Struct for a sequence of custom_interfaces__msg__TargetPosePolar.
typedef struct custom_interfaces__msg__TargetPosePolar__Sequence
{
  custom_interfaces__msg__TargetPosePolar * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__TargetPosePolar__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__TARGET_POSE_POLAR__STRUCT_H_
