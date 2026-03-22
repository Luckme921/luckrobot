// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__STRUCT_H_

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
// Member 'connection_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DeviceStatus in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__msg__DeviceStatus
{
  std_msgs__msg__Header header;
  /// --- Color stream ---
  double color_frame_rate_cur;
  double color_frame_rate_avg;
  double color_frame_rate_min;
  double color_frame_rate_max;
  double color_delay_ms_cur;
  double color_delay_ms_avg;
  double color_delay_ms_min;
  double color_delay_ms_max;
  /// --- Depth stream ---
  double depth_frame_rate_cur;
  double depth_frame_rate_avg;
  double depth_frame_rate_min;
  double depth_frame_rate_max;
  double depth_delay_ms_cur;
  double depth_delay_ms_avg;
  double depth_delay_ms_min;
  double depth_delay_ms_max;
  /// --- Device info ---
  bool device_online;
  /// e.g. "USB2.0", "USB3.0", "GigE"
  rosidl_runtime_c__String connection_type;
  /// --- Calibration status ---
  bool customer_calibration_ready;
  bool calibration_from_factory;
  bool calibration_from_launch_param;
} orbbec_camera_msgs__msg__DeviceStatus;

// Struct for a sequence of orbbec_camera_msgs__msg__DeviceStatus.
typedef struct orbbec_camera_msgs__msg__DeviceStatus__Sequence
{
  orbbec_camera_msgs__msg__DeviceStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__msg__DeviceStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__STRUCT_H_
