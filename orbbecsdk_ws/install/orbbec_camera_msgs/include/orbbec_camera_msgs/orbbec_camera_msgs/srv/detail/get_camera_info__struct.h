// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:srv/GetCameraInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetCameraInfo in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__GetCameraInfo_Request
{
  uint8_t structure_needs_at_least_one_member;
} orbbec_camera_msgs__srv__GetCameraInfo_Request;

// Struct for a sequence of orbbec_camera_msgs__srv__GetCameraInfo_Request.
typedef struct orbbec_camera_msgs__srv__GetCameraInfo_Request__Sequence
{
  orbbec_camera_msgs__srv__GetCameraInfo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__GetCameraInfo_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"

/// Struct defined in srv/GetCameraInfo in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__GetCameraInfo_Response
{
  sensor_msgs__msg__CameraInfo info;
} orbbec_camera_msgs__srv__GetCameraInfo_Response;

// Struct for a sequence of orbbec_camera_msgs__srv__GetCameraInfo_Response.
typedef struct orbbec_camera_msgs__srv__GetCameraInfo_Response__Sequence
{
  orbbec_camera_msgs__srv__GetCameraInfo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__GetCameraInfo_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__STRUCT_H_
