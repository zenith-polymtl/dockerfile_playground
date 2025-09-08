// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/TargetPosePolar.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/target_pose_polar__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
custom_interfaces__msg__TargetPosePolar__init(custom_interfaces__msg__TargetPosePolar * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_interfaces__msg__TargetPosePolar__fini(msg);
    return false;
  }
  // r
  // theta
  // z
  // relative
  // v_r
  // v_theta
  return true;
}

void
custom_interfaces__msg__TargetPosePolar__fini(custom_interfaces__msg__TargetPosePolar * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // r
  // theta
  // z
  // relative
  // v_r
  // v_theta
}

bool
custom_interfaces__msg__TargetPosePolar__are_equal(const custom_interfaces__msg__TargetPosePolar * lhs, const custom_interfaces__msg__TargetPosePolar * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // relative
  if (lhs->relative != rhs->relative) {
    return false;
  }
  // v_r
  if (lhs->v_r != rhs->v_r) {
    return false;
  }
  // v_theta
  if (lhs->v_theta != rhs->v_theta) {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__TargetPosePolar__copy(
  const custom_interfaces__msg__TargetPosePolar * input,
  custom_interfaces__msg__TargetPosePolar * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // r
  output->r = input->r;
  // theta
  output->theta = input->theta;
  // z
  output->z = input->z;
  // relative
  output->relative = input->relative;
  // v_r
  output->v_r = input->v_r;
  // v_theta
  output->v_theta = input->v_theta;
  return true;
}

custom_interfaces__msg__TargetPosePolar *
custom_interfaces__msg__TargetPosePolar__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__TargetPosePolar * msg = (custom_interfaces__msg__TargetPosePolar *)allocator.allocate(sizeof(custom_interfaces__msg__TargetPosePolar), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__TargetPosePolar));
  bool success = custom_interfaces__msg__TargetPosePolar__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__TargetPosePolar__destroy(custom_interfaces__msg__TargetPosePolar * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__TargetPosePolar__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__TargetPosePolar__Sequence__init(custom_interfaces__msg__TargetPosePolar__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__TargetPosePolar * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__TargetPosePolar *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__TargetPosePolar), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__TargetPosePolar__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__TargetPosePolar__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interfaces__msg__TargetPosePolar__Sequence__fini(custom_interfaces__msg__TargetPosePolar__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interfaces__msg__TargetPosePolar__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interfaces__msg__TargetPosePolar__Sequence *
custom_interfaces__msg__TargetPosePolar__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__TargetPosePolar__Sequence * array = (custom_interfaces__msg__TargetPosePolar__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__TargetPosePolar__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__TargetPosePolar__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__TargetPosePolar__Sequence__destroy(custom_interfaces__msg__TargetPosePolar__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__TargetPosePolar__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__TargetPosePolar__Sequence__are_equal(const custom_interfaces__msg__TargetPosePolar__Sequence * lhs, const custom_interfaces__msg__TargetPosePolar__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__TargetPosePolar__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__TargetPosePolar__Sequence__copy(
  const custom_interfaces__msg__TargetPosePolar__Sequence * input,
  custom_interfaces__msg__TargetPosePolar__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__TargetPosePolar);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__TargetPosePolar * data =
      (custom_interfaces__msg__TargetPosePolar *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__TargetPosePolar__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__TargetPosePolar__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__TargetPosePolar__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
