// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from orbbec_camera_msgs:msg/Extrinsics.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "orbbec_camera_msgs/msg/detail/extrinsics__rosidl_typesupport_introspection_c.h"
#include "orbbec_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "orbbec_camera_msgs/msg/detail/extrinsics__functions.h"
#include "orbbec_camera_msgs/msg/detail/extrinsics__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  orbbec_camera_msgs__msg__Extrinsics__init(message_memory);
}

void orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_fini_function(void * message_memory)
{
  orbbec_camera_msgs__msg__Extrinsics__fini(message_memory);
}

size_t orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__size_function__Extrinsics__rotation(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_const_function__Extrinsics__rotation(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_function__Extrinsics__rotation(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__fetch_function__Extrinsics__rotation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_const_function__Extrinsics__rotation(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__assign_function__Extrinsics__rotation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_function__Extrinsics__rotation(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__size_function__Extrinsics__translation(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_const_function__Extrinsics__translation(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_function__Extrinsics__translation(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__fetch_function__Extrinsics__translation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_const_function__Extrinsics__translation(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__assign_function__Extrinsics__translation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_function__Extrinsics__translation(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs__msg__Extrinsics, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
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
    offsetof(orbbec_camera_msgs__msg__Extrinsics, rotation),  // bytes offset in struct
    NULL,  // default value
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__size_function__Extrinsics__rotation,  // size() function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_const_function__Extrinsics__rotation,  // get_const(index) function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_function__Extrinsics__rotation,  // get(index) function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__fetch_function__Extrinsics__rotation,  // fetch(index, &value) function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__assign_function__Extrinsics__rotation,  // assign(index, value) function pointer
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
    offsetof(orbbec_camera_msgs__msg__Extrinsics, translation),  // bytes offset in struct
    NULL,  // default value
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__size_function__Extrinsics__translation,  // size() function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_const_function__Extrinsics__translation,  // get_const(index) function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__get_function__Extrinsics__translation,  // get(index) function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__fetch_function__Extrinsics__translation,  // fetch(index, &value) function pointer
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__assign_function__Extrinsics__translation,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_members = {
  "orbbec_camera_msgs__msg",  // message namespace
  "Extrinsics",  // message name
  3,  // number of fields
  sizeof(orbbec_camera_msgs__msg__Extrinsics),
  orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_member_array,  // message members
  orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_init_function,  // function to initialize message memory (memory has to be allocated)
  orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_type_support_handle = {
  0,
  &orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_orbbec_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, orbbec_camera_msgs, msg, Extrinsics)() {
  orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_type_support_handle.typesupport_identifier) {
    orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &orbbec_camera_msgs__msg__Extrinsics__rosidl_typesupport_introspection_c__Extrinsics_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
