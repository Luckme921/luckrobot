// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from orbbec_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice
#include "orbbec_camera_msgs/msg/detail/imu_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
orbbec_camera_msgs__msg__IMUInfo__init(orbbec_camera_msgs__msg__IMUInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    orbbec_camera_msgs__msg__IMUInfo__fini(msg);
    return false;
  }
  // noise_density
  // random_walk
  // reference_temperature
  // bias
  // gravity
  // scale_misalignment
  // temperature_slope
  return true;
}

void
orbbec_camera_msgs__msg__IMUInfo__fini(orbbec_camera_msgs__msg__IMUInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // noise_density
  // random_walk
  // reference_temperature
  // bias
  // gravity
  // scale_misalignment
  // temperature_slope
}

bool
orbbec_camera_msgs__msg__IMUInfo__are_equal(const orbbec_camera_msgs__msg__IMUInfo * lhs, const orbbec_camera_msgs__msg__IMUInfo * rhs)
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
  // noise_density
  if (lhs->noise_density != rhs->noise_density) {
    return false;
  }
  // random_walk
  if (lhs->random_walk != rhs->random_walk) {
    return false;
  }
  // reference_temperature
  if (lhs->reference_temperature != rhs->reference_temperature) {
    return false;
  }
  // bias
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->bias[i] != rhs->bias[i]) {
      return false;
    }
  }
  // gravity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gravity[i] != rhs->gravity[i]) {
      return false;
    }
  }
  // scale_misalignment
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->scale_misalignment[i] != rhs->scale_misalignment[i]) {
      return false;
    }
  }
  // temperature_slope
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->temperature_slope[i] != rhs->temperature_slope[i]) {
      return false;
    }
  }
  return true;
}

bool
orbbec_camera_msgs__msg__IMUInfo__copy(
  const orbbec_camera_msgs__msg__IMUInfo * input,
  orbbec_camera_msgs__msg__IMUInfo * output)
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
  // noise_density
  output->noise_density = input->noise_density;
  // random_walk
  output->random_walk = input->random_walk;
  // reference_temperature
  output->reference_temperature = input->reference_temperature;
  // bias
  for (size_t i = 0; i < 3; ++i) {
    output->bias[i] = input->bias[i];
  }
  // gravity
  for (size_t i = 0; i < 3; ++i) {
    output->gravity[i] = input->gravity[i];
  }
  // scale_misalignment
  for (size_t i = 0; i < 9; ++i) {
    output->scale_misalignment[i] = input->scale_misalignment[i];
  }
  // temperature_slope
  for (size_t i = 0; i < 9; ++i) {
    output->temperature_slope[i] = input->temperature_slope[i];
  }
  return true;
}

orbbec_camera_msgs__msg__IMUInfo *
orbbec_camera_msgs__msg__IMUInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orbbec_camera_msgs__msg__IMUInfo * msg = (orbbec_camera_msgs__msg__IMUInfo *)allocator.allocate(sizeof(orbbec_camera_msgs__msg__IMUInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orbbec_camera_msgs__msg__IMUInfo));
  bool success = orbbec_camera_msgs__msg__IMUInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orbbec_camera_msgs__msg__IMUInfo__destroy(orbbec_camera_msgs__msg__IMUInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orbbec_camera_msgs__msg__IMUInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orbbec_camera_msgs__msg__IMUInfo__Sequence__init(orbbec_camera_msgs__msg__IMUInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orbbec_camera_msgs__msg__IMUInfo * data = NULL;

  if (size) {
    data = (orbbec_camera_msgs__msg__IMUInfo *)allocator.zero_allocate(size, sizeof(orbbec_camera_msgs__msg__IMUInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orbbec_camera_msgs__msg__IMUInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orbbec_camera_msgs__msg__IMUInfo__fini(&data[i - 1]);
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
orbbec_camera_msgs__msg__IMUInfo__Sequence__fini(orbbec_camera_msgs__msg__IMUInfo__Sequence * array)
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
      orbbec_camera_msgs__msg__IMUInfo__fini(&array->data[i]);
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

orbbec_camera_msgs__msg__IMUInfo__Sequence *
orbbec_camera_msgs__msg__IMUInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orbbec_camera_msgs__msg__IMUInfo__Sequence * array = (orbbec_camera_msgs__msg__IMUInfo__Sequence *)allocator.allocate(sizeof(orbbec_camera_msgs__msg__IMUInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orbbec_camera_msgs__msg__IMUInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orbbec_camera_msgs__msg__IMUInfo__Sequence__destroy(orbbec_camera_msgs__msg__IMUInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orbbec_camera_msgs__msg__IMUInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orbbec_camera_msgs__msg__IMUInfo__Sequence__are_equal(const orbbec_camera_msgs__msg__IMUInfo__Sequence * lhs, const orbbec_camera_msgs__msg__IMUInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orbbec_camera_msgs__msg__IMUInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orbbec_camera_msgs__msg__IMUInfo__Sequence__copy(
  const orbbec_camera_msgs__msg__IMUInfo__Sequence * input,
  orbbec_camera_msgs__msg__IMUInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orbbec_camera_msgs__msg__IMUInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orbbec_camera_msgs__msg__IMUInfo * data =
      (orbbec_camera_msgs__msg__IMUInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orbbec_camera_msgs__msg__IMUInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orbbec_camera_msgs__msg__IMUInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orbbec_camera_msgs__msg__IMUInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
