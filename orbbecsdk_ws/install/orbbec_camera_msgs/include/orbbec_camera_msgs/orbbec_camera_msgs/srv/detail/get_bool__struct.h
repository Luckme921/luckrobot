// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:srv/GetBool.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_BOOL__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_BOOL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetBool in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__GetBool_Request
{
  uint8_t structure_needs_at_least_one_member;
} orbbec_camera_msgs__srv__GetBool_Request;

// Struct for a sequence of orbbec_camera_msgs__srv__GetBool_Request.
typedef struct orbbec_camera_msgs__srv__GetBool_Request__Sequence
{
  orbbec_camera_msgs__srv__GetBool_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__GetBool_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetBool in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__GetBool_Response
{
  bool data;
  bool success;
  rosidl_runtime_c__String message;
} orbbec_camera_msgs__srv__GetBool_Response;

// Struct for a sequence of orbbec_camera_msgs__srv__GetBool_Response.
typedef struct orbbec_camera_msgs__srv__GetBool_Response__Sequence
{
  orbbec_camera_msgs__srv__GetBool_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__GetBool_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_BOOL__STRUCT_H_
