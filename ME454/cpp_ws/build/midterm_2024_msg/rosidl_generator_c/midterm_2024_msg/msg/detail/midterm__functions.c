// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from midterm_2024_msg:msg/Midterm.idl
// generated code does not contain a copyright notice
#include "midterm_2024_msg/msg/detail/midterm__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `angular_velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `quaternion`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
midterm_2024_msg__msg__Midterm__init(midterm_2024_msg__msg__Midterm * msg)
{
  if (!msg) {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    midterm_2024_msg__msg__Midterm__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    midterm_2024_msg__msg__Midterm__fini(msg);
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__init(&msg->quaternion)) {
    midterm_2024_msg__msg__Midterm__fini(msg);
    return false;
  }
  return true;
}

void
midterm_2024_msg__msg__Midterm__fini(midterm_2024_msg__msg__Midterm * msg)
{
  if (!msg) {
    return;
  }
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // quaternion
  geometry_msgs__msg__Quaternion__fini(&msg->quaternion);
}

bool
midterm_2024_msg__msg__Midterm__are_equal(const midterm_2024_msg__msg__Midterm * lhs, const midterm_2024_msg__msg__Midterm * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->quaternion), &(rhs->quaternion)))
  {
    return false;
  }
  return true;
}

bool
midterm_2024_msg__msg__Midterm__copy(
  const midterm_2024_msg__msg__Midterm * input,
  midterm_2024_msg__msg__Midterm * output)
{
  if (!input || !output) {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // quaternion
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->quaternion), &(output->quaternion)))
  {
    return false;
  }
  return true;
}

midterm_2024_msg__msg__Midterm *
midterm_2024_msg__msg__Midterm__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  midterm_2024_msg__msg__Midterm * msg = (midterm_2024_msg__msg__Midterm *)allocator.allocate(sizeof(midterm_2024_msg__msg__Midterm), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(midterm_2024_msg__msg__Midterm));
  bool success = midterm_2024_msg__msg__Midterm__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
midterm_2024_msg__msg__Midterm__destroy(midterm_2024_msg__msg__Midterm * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    midterm_2024_msg__msg__Midterm__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
midterm_2024_msg__msg__Midterm__Sequence__init(midterm_2024_msg__msg__Midterm__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  midterm_2024_msg__msg__Midterm * data = NULL;

  if (size) {
    data = (midterm_2024_msg__msg__Midterm *)allocator.zero_allocate(size, sizeof(midterm_2024_msg__msg__Midterm), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = midterm_2024_msg__msg__Midterm__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        midterm_2024_msg__msg__Midterm__fini(&data[i - 1]);
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
midterm_2024_msg__msg__Midterm__Sequence__fini(midterm_2024_msg__msg__Midterm__Sequence * array)
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
      midterm_2024_msg__msg__Midterm__fini(&array->data[i]);
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

midterm_2024_msg__msg__Midterm__Sequence *
midterm_2024_msg__msg__Midterm__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  midterm_2024_msg__msg__Midterm__Sequence * array = (midterm_2024_msg__msg__Midterm__Sequence *)allocator.allocate(sizeof(midterm_2024_msg__msg__Midterm__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = midterm_2024_msg__msg__Midterm__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
midterm_2024_msg__msg__Midterm__Sequence__destroy(midterm_2024_msg__msg__Midterm__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    midterm_2024_msg__msg__Midterm__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
midterm_2024_msg__msg__Midterm__Sequence__are_equal(const midterm_2024_msg__msg__Midterm__Sequence * lhs, const midterm_2024_msg__msg__Midterm__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!midterm_2024_msg__msg__Midterm__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
midterm_2024_msg__msg__Midterm__Sequence__copy(
  const midterm_2024_msg__msg__Midterm__Sequence * input,
  midterm_2024_msg__msg__Midterm__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(midterm_2024_msg__msg__Midterm);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    midterm_2024_msg__msg__Midterm * data =
      (midterm_2024_msg__msg__Midterm *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!midterm_2024_msg__msg__Midterm__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          midterm_2024_msg__msg__Midterm__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!midterm_2024_msg__msg__Midterm__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
