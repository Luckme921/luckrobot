// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:srv/GetUserCalibParams.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_USER_CALIB_PARAMS__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_USER_CALIB_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetUserCalibParams in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__GetUserCalibParams_Request
{
  uint8_t structure_needs_at_least_one_member;
} orbbec_camera_msgs__srv__GetUserCalibParams_Request;

// Struct for a sequence of orbbec_camera_msgs__srv__GetUserCalibParams_Request.
typedef struct orbbec_camera_msgs__srv__GetUserCalibParams_Request__Sequence
{
  orbbec_camera_msgs__srv__GetUserCalibParams_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__GetUserCalibParams_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetUserCalibParams in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__GetUserCalibParams_Response
{
  /// Intrinsic camera matrix for the raw (distorted) images
  double k[9];
  /// The distortion parameters
  double d[8];
  /// Extrinsic rotation matrix
  double rotation[9];
  /// Extrinsic translation vector
  double translation[3];
  bool success;
  rosidl_runtime_c__String message;
} orbbec_camera_msgs__srv__GetUserCalibParams_Response;

// Struct for a sequence of orbbec_camera_msgs__srv__GetUserCalibParams_Response.
typedef struct orbbec_camera_msgs__srv__GetUserCalibParams_Response__Sequence
{
  orbbec_camera_msgs__srv__GetUserCalibParams_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__GetUserCalibParams_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_USER_CALIB_PARAMS__STRUCT_H_
