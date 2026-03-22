// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
// generated code does not contain a copyright notice
#include "orbbec_camera_msgs/msg/detail/device_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `connection_type`
#include "rosidl_runtime_c/string_functions.h"

bool
orbbec_camera_msgs__msg__DeviceStatus__init(orbbec_camera_msgs__msg__DeviceStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    orbbec_camera_msgs__msg__DeviceStatus__fini(msg);
    return false;
  }
  // color_frame_rate_cur
  // color_frame_rate_avg
  // color_frame_rate_min
  // color_frame_rate_max
  // color_delay_ms_cur
  // color_delay_ms_avg
  // color_delay_ms_min
  // color_delay_ms_max
  // depth_frame_rate_cur
  // depth_frame_rate_avg
  // depth_frame_rate_min
  // depth_frame_rate_max
  // depth_delay_ms_cur
  // depth_delay_ms_avg
  // depth_delay_ms_min
  // depth_delay_ms_max
  // device_online
  // connection_type
  if (!rosidl_runtime_c__String__init(&msg->connection_type)) {
    orbbec_camera_msgs__msg__DeviceStatus__fini(msg);
    return false;
  }
  // customer_calibration_ready
  // calibration_from_factory
  // calibration_from_launch_param
  return true;
}

void
orbbec_camera_msgs__msg__DeviceStatus__fini(orbbec_camera_msgs__msg__DeviceStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // color_frame_rate_cur
  // color_frame_rate_avg
  // color_frame_rate_min
  // color_frame_rate_max
  // color_delay_ms_cur
  // color_delay_ms_avg
  // color_delay_ms_min
  // color_delay_ms_max
  // depth_frame_rate_cur
  // depth_frame_rate_avg
  // depth_frame_rate_min
  // depth_frame_rate_max
  // depth_delay_ms_cur
  // depth_delay_ms_avg
  // depth_delay_ms_min
  // depth_delay_ms_max
  // device_online
  // connection_type
  rosidl_runtime_c__String__fini(&msg->connection_type);
  // customer_calibration_ready
  // calibration_from_factory
  // calibration_from_launch_param
}

bool
orbbec_camera_msgs__msg__DeviceStatus__are_equal(const orbbec_camera_msgs__msg__DeviceStatus * lhs, const orbbec_camera_msgs__msg__DeviceStatus * rhs)
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
  // color_frame_rate_cur
  if (lhs->color_frame_rate_cur != rhs->color_frame_rate_cur) {
    return false;
  }
  // color_frame_rate_avg
  if (lhs->color_frame_rate_avg != rhs->color_frame_rate_avg) {
    return false;
  }
  // color_frame_rate_min
  if (lhs->color_frame_rate_min != rhs->color_frame_rate_min) {
    return false;
  }
  // color_frame_rate_max
  if (lhs->color_frame_rate_max != rhs->color_frame_rate_max) {
    return false;
  }
  // color_delay_ms_cur
  if (lhs->color_delay_ms_cur != rhs->color_delay_ms_cur) {
    return false;
  }
  // color_delay_ms_avg
  if (lhs->color_delay_ms_avg != rhs->color_delay_ms_avg) {
    return false;
  }
  // color_delay_ms_min
  if (lhs->color_delay_ms_min != rhs->color_delay_ms_min) {
    return false;
  }
  // color_delay_ms_max
  if (lhs->color_delay_ms_max != rhs->color_delay_ms_max) {
    return false;
  }
  // depth_frame_rate_cur
  if (lhs->depth_frame_rate_cur != rhs->depth_frame_rate_cur) {
    return false;
  }
  // depth_frame_rate_avg
  if (lhs->depth_frame_rate_avg != rhs->depth_frame_rate_avg) {
    return false;
  }
  // depth_frame_rate_min
  if (lhs->depth_frame_rate_min != rhs->depth_frame_rate_min) {
    return false;
  }
  // depth_frame_rate_max
  if (lhs->depth_frame_rate_max != rhs->depth_frame_rate_max) {
    return false;
  }
  // depth_delay_ms_cur
  if (lhs->depth_delay_ms_cur != rhs->depth_delay_ms_cur) {
    return false;
  }
  // depth_delay_ms_avg
  if (lhs->depth_delay_ms_avg != rhs->depth_delay_ms_avg) {
    return false;
  }
  // depth_delay_ms_min
  if (lhs->depth_delay_ms_min != rhs->depth_delay_ms_min) {
    return false;
  }
  // depth_delay_ms_max
  if (lhs->depth_delay_ms_max != rhs->depth_delay_ms_max) {
    return false;
  }
  // device_online
  if (lhs->device_online != rhs->device_online) {
    return false;
  }
  // connection_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->connection_type), &(rhs->connection_type)))
  {
    return false;
  }
  // customer_calibration_ready
  if (lhs->customer_calibration_ready != rhs->customer_calibration_ready) {
    return false;
  }
  // calibration_from_factory
  if (lhs->calibration_from_factory != rhs->calibration_from_factory) {
    return false;
  }
  // calibration_from_launch_param
  if (lhs->calibration_from_launch_param != rhs->calibration_from_launch_param) {
    return false;
  }
  return true;
}

bool
orbbec_camera_msgs__msg__DeviceStatus__copy(
  const orbbec_camera_msgs__msg__DeviceStatus * input,
  orbbec_camera_msgs__msg__DeviceStatus * output)
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
  // color_frame_rate_cur
  output->color_frame_rate_cur = input->color_frame_rate_cur;
  // color_frame_rate_avg
  output->color_frame_rate_avg = input->color_frame_rate_avg;
  // color_frame_rate_min
  output->color_frame_rate_min = input->color_frame_rate_min;
  // color_frame_rate_max
  output->color_frame_rate_max = input->color_frame_rate_max;
  // color_delay_ms_cur
  output->color_delay_ms_cur = input->color_delay_ms_cur;
  // color_delay_ms_avg
  output->color_delay_ms_avg = input->color_delay_ms_avg;
  // color_delay_ms_min
  output->color_delay_ms_min = input->color_delay_ms_min;
  // color_delay_ms_max
  output->color_delay_ms_max = input->color_delay_ms_max;
  // depth_frame_rate_cur
  output->depth_frame_rate_cur = input->depth_frame_rate_cur;
  // depth_frame_rate_avg
  output->depth_frame_rate_avg = input->depth_frame_rate_avg;
  // depth_frame_rate_min
  output->depth_frame_rate_min = input->depth_frame_rate_min;
  // depth_frame_rate_max
  output->depth_frame_rate_max = input->depth_frame_rate_max;
  // depth_delay_ms_cur
  output->depth_delay_ms_cur = input->depth_delay_ms_cur;
  // depth_delay_ms_avg
  output->depth_delay_ms_avg = input->depth_delay_ms_avg;
  // depth_delay_ms_min
  output->depth_delay_ms_min = input->depth_delay_ms_min;
  // depth_delay_ms_max
  output->depth_delay_ms_max = input->depth_delay_ms_max;
  // device_online
  output->device_online = input->device_online;
  // connection_type
  if (!rosidl_runtime_c__String__copy(
      &(input->connection_type), &(output->connection_type)))
  {
    return false;
  }
  // customer_calibration_ready
  output->customer_calibration_ready = input->customer_calibration_ready;
  // calibration_from_factory
  output->calibration_from_factory = input->calibration_from_factory;
  // calibration_from_launch_param
  output->calibration_from_launch_param = input->calibration_from_launch_param;
  return true;
}

orbbec_camera_msgs__msg__DeviceStatus *
orbbec_camera_msgs__msg__DeviceStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orbbec_camera_msgs__msg__DeviceStatus * msg = (orbbec_camera_msgs__msg__DeviceStatus *)allocator.allocate(sizeof(orbbec_camera_msgs__msg__DeviceStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orbbec_camera_msgs__msg__DeviceStatus));
  bool success = orbbec_camera_msgs__msg__DeviceStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orbbec_camera_msgs__msg__DeviceStatus__destroy(orbbec_camera_msgs__msg__DeviceStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orbbec_camera_msgs__msg__DeviceStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orbbec_camera_msgs__msg__DeviceStatus__Sequence__init(orbbec_camera_msgs__msg__DeviceStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orbbec_camera_msgs__msg__DeviceStatus * data = NULL;

  if (size) {
    data = (orbbec_camera_msgs__msg__DeviceStatus *)allocator.zero_allocate(size, sizeof(orbbec_camera_msgs__msg__DeviceStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orbbec_camera_msgs__msg__DeviceStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orbbec_camera_msgs__msg__DeviceStatus__fini(&data[i - 1]);
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
orbbec_camera_msgs__msg__DeviceStatus__Sequence__fini(orbbec_camera_msgs__msg__DeviceStatus__Sequence * array)
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
      orbbec_camera_msgs__msg__DeviceStatus__fini(&array->data[i]);
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

orbbec_camera_msgs__msg__DeviceStatus__Sequence *
orbbec_camera_msgs__msg__DeviceStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orbbec_camera_msgs__msg__DeviceStatus__Sequence * array = (orbbec_camera_msgs__msg__DeviceStatus__Sequence *)allocator.allocate(sizeof(orbbec_camera_msgs__msg__DeviceStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orbbec_camera_msgs__msg__DeviceStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orbbec_camera_msgs__msg__DeviceStatus__Sequence__destroy(orbbec_camera_msgs__msg__DeviceStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orbbec_camera_msgs__msg__DeviceStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orbbec_camera_msgs__msg__DeviceStatus__Sequence__are_equal(const orbbec_camera_msgs__msg__DeviceStatus__Sequence * lhs, const orbbec_camera_msgs__msg__DeviceStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orbbec_camera_msgs__msg__DeviceStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orbbec_camera_msgs__msg__DeviceStatus__Sequence__copy(
  const orbbec_camera_msgs__msg__DeviceStatus__Sequence * input,
  orbbec_camera_msgs__msg__DeviceStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orbbec_camera_msgs__msg__DeviceStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orbbec_camera_msgs__msg__DeviceStatus * data =
      (orbbec_camera_msgs__msg__DeviceStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orbbec_camera_msgs__msg__DeviceStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orbbec_camera_msgs__msg__DeviceStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orbbec_camera_msgs__msg__DeviceStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
