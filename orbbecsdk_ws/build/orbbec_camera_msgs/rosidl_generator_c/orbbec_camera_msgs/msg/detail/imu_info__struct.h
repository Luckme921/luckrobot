// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__STRUCT_H_

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

/// Struct defined in msg/IMUInfo in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__msg__IMUInfo
{
  std_msgs__msg__Header header;
  double noise_density;
  double random_walk;
  double reference_temperature;
  double bias[3];
  double gravity[3];
  double scale_misalignment[9];
  double temperature_slope[9];
} orbbec_camera_msgs__msg__IMUInfo;

// Struct for a sequence of orbbec_camera_msgs__msg__IMUInfo.
typedef struct orbbec_camera_msgs__msg__IMUInfo__Sequence
{
  orbbec_camera_msgs__msg__IMUInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__msg__IMUInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__STRUCT_H_
