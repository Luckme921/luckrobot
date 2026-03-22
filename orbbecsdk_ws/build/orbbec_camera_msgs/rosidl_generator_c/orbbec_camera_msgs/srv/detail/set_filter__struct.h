// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orbbec_camera_msgs:srv/SetFilter.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_FILTER__STRUCT_H_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_FILTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filter_name'
#include "rosidl_runtime_c/string.h"
// Member 'filter_param'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/SetFilter in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__SetFilter_Request
{
  rosidl_runtime_c__String filter_name;
  bool filter_enable;
  rosidl_runtime_c__float__Sequence filter_param;
} orbbec_camera_msgs__srv__SetFilter_Request;

// Struct for a sequence of orbbec_camera_msgs__srv__SetFilter_Request.
typedef struct orbbec_camera_msgs__srv__SetFilter_Request__Sequence
{
  orbbec_camera_msgs__srv__SetFilter_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__SetFilter_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetFilter in the package orbbec_camera_msgs.
typedef struct orbbec_camera_msgs__srv__SetFilter_Response
{
  bool success;
  rosidl_runtime_c__String message;
} orbbec_camera_msgs__srv__SetFilter_Response;

// Struct for a sequence of orbbec_camera_msgs__srv__SetFilter_Response.
typedef struct orbbec_camera_msgs__srv__SetFilter_Response__Sequence
{
  orbbec_camera_msgs__srv__SetFilter_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orbbec_camera_msgs__srv__SetFilter_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_FILTER__STRUCT_H_
