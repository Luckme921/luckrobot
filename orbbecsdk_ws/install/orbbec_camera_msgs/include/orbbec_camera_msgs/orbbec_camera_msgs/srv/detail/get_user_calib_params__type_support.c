// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from orbbec_camera_msgs:srv/GetUserCalibParams.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "orbbec_camera_msgs/srv/detail/get_user_calib_params__rosidl_typesupport_introspection_c.h"
#include "orbbec_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "orbbec_camera_msgs/srv/detail/get_user_calib_params__functions.h"
#include "orbbec_camera_msgs/srv/detail/get_user_calib_params__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  orbbec_camera_msgs__srv__GetUserCalibParams_Request__init(message_memory);
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_fini_function(void * message_memory)
{
  orbbec_camera_msgs__srv__GetUserCalibParams_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_members = {
  "orbbec_camera_msgs__srv",  // message namespace
  "GetUserCalibParams_Request",  // message name
  1,  // number of fields
  sizeof(orbbec_camera_msgs__srv__GetUserCalibParams_Request),
  orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_member_array,  // message members
  orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_type_support_handle = {
  0,
  &orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_orbbec_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams_Request)() {
  if (!orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_type_support_handle.typesupport_identifier) {
    orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &orbbec_camera_msgs__srv__GetUserCalibParams_Request__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "orbbec_camera_msgs/srv/detail/get_user_calib_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "orbbec_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "orbbec_camera_msgs/srv/detail/get_user_calib_params__functions.h"
// already included above
// #include "orbbec_camera_msgs/srv/detail/get_user_calib_params__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  orbbec_camera_msgs__srv__GetUserCalibParams_Response__init(message_memory);
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_fini_function(void * message_memory)
{
  orbbec_camera_msgs__srv__GetUserCalibParams_Response__fini(message_memory);
}

size_t orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__k(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__k(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__k(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__k(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__k(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__k(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__k(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__d(
  const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__d(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__d(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__d(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__d(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__rotation(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__rotation(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__rotation(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__rotation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__rotation(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__rotation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__rotation(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__translation(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__translation(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__translation(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__translation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__translation(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__translation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__translation(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_member_array[6] = {
  {
    "k",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Response, k),  // bytes offset in struct
    NULL,  // default value
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__k,  // size() function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__k,  // get_const(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__k,  // get(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__k,  // fetch(index, &value) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__k,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Response, d),  // bytes offset in struct
    NULL,  // default value
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__d,  // size() function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__d,  // get_const(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__d,  // get(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__d,  // fetch(index, &value) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__d,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rotation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Response, rotation),  // bytes offset in struct
    NULL,  // default value
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__rotation,  // size() function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__rotation,  // get_const(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__rotation,  // get(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__rotation,  // fetch(index, &value) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__rotation,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "translation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Response, translation),  // bytes offset in struct
    NULL,  // default value
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__size_function__GetUserCalibParams_Response__translation,  // size() function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_const_function__GetUserCalibParams_Response__translation,  // get_const(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__get_function__GetUserCalibParams_Response__translation,  // get(index) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__fetch_function__GetUserCalibParams_Response__translation,  // fetch(index, &value) function pointer
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__assign_function__GetUserCalibParams_Response__translation,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__srv__GetUserCalibParams_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_members = {
  "orbbec_camera_msgs__srv",  // message namespace
  "GetUserCalibParams_Response",  // message name
  6,  // number of fields
  sizeof(orbbec_camera_msgs__srv__GetUserCalibParams_Response),
  orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_member_array,  // message members
  orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_type_support_handle = {
  0,
  &orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_orbbec_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams_Response)() {
  if (!orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_type_support_handle.typesupport_identifier) {
    orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &orbbec_camera_msgs__srv__GetUserCalibParams_Response__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "orbbec_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "orbbec_camera_msgs/srv/detail/get_user_calib_params__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_members = {
  "orbbec_camera_msgs__srv",  // service namespace
  "GetUserCalibParams",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_Request_message_type_support_handle,
  NULL  // response message
  // orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_Response_message_type_support_handle
};

static rosidl_service_type_support_t orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_type_support_handle = {
  0,
  &orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_orbbec_camera_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams)() {
  if (!orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_type_support_handle.typesupport_identifier) {
    orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, srv, GetUserCalibParams_Response)()->data;
  }

  return &orbbec_camera_msgs__srv__detail__get_user_calib_params__rosidl_typesupport_introspection_c__GetUserCalibParams_service_type_support_handle;
}
