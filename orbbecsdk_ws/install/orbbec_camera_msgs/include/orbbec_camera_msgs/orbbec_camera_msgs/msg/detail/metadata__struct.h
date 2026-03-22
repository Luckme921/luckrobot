// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__METADATA__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__METADATA__STRUCT_H_

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
// Member 'json_data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Metadata in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__msg__Metadata
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String json_data;
} orbbec_camera_msgs__msg__Metadata;

// Struct for a sequence of orbbec_camera_msgs__msg__Metadata.
typedef struct orbbec_camera_msgs__msg__Metadata__Sequence
{
  orbbec_camera_msgs__msg__Metadata * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__msg__Metadata__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__METADATA__STRUCT_H_
