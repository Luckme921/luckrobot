// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from livox_ros_driver2:msg/CustomMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "livox_ros_driver2/msg/detail/custom_msg__rosidl_typesupport_introspection_c.h"
#include "livox_ros_driver2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "livox_ros_driver2/msg/detail/custom_msg__functions.h"
#include "livox_ros_driver2/msg/detail/custom_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "livox_ros_driver2/msg/custom_point.h"
// Member `points`
#include "livox_ros_driver2/msg/detail/custom_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  livox_ros_driver2__msg__CustomMsg__init(message_memory);
}

void livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_fini_function(void * message_memory)
{
  livox_ros_driver2__msg__CustomMsg__fini(message_memory);
}

size_t livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__size_function__CustomMsg__rsvd(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomMsg__rsvd(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomMsg__rsvd(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__fetch_function__CustomMsg__rsvd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomMsg__rsvd(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__assign_function__CustomMsg__rsvd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomMsg__rsvd(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__size_function__CustomMsg__points(
  const void * untyped_member)
{
  const livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (const livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomMsg__points(
  const void * untyped_member, size_t index)
{
  const livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (const livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomMsg__points(
  void * untyped_member, size_t index)
{
  livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__fetch_function__CustomMsg__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const livox_ros_driver2__msg__CustomPoint * item =
    ((const livox_ros_driver2__msg__CustomPoint *)
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomMsg__points(untyped_member, index));
  livox_ros_driver2__msg__CustomPoint * value =
    (livox_ros_driver2__msg__CustomPoint *)(untyped_value);
  *value = *item;
}

void livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__assign_function__CustomMsg__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  livox_ros_driver2__msg__CustomPoint * item =
    ((livox_ros_driver2__msg__CustomPoint *)
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomMsg__points(untyped_member, index));
  const livox_ros_driver2__msg__CustomPoint * value =
    (const livox_ros_driver2__msg__CustomPoint *)(untyped_value);
  *item = *value;
}

bool livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__resize_function__CustomMsg__points(
  void * untyped_member, size_t size)
{
  livox_ros_driver2__msg__CustomPoint__Sequence * member =
    (livox_ros_driver2__msg__CustomPoint__Sequence *)(untyped_member);
  livox_ros_driver2__msg__CustomPoint__Sequence__fini(member);
  return livox_ros_driver2__msg__CustomPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timebase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, timebase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, point_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lidar_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, lidar_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rsvd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, rsvd),  // bytes offset in struct
    NULL,  // default value
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__size_function__CustomMsg__rsvd,  // size() function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomMsg__rsvd,  // get_const(index) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomMsg__rsvd,  // get(index) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__fetch_function__CustomMsg__rsvd,  // fetch(index, &value) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__assign_function__CustomMsg__rsvd,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(livox_ros_driver2__msg__CustomMsg, points),  // bytes offset in struct
    NULL,  // default value
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__size_function__CustomMsg__points,  // size() function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_const_function__CustomMsg__points,  // get_const(index) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__get_function__CustomMsg__points,  // get(index) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__fetch_function__CustomMsg__points,  // fetch(index, &value) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__assign_function__CustomMsg__points,  // assign(index, value) function pointer
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__resize_function__CustomMsg__points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_members = {
  "livox_ros_driver2__msg",  // message namespace
  "CustomMsg",  // message name
  6,  // number of fields
  sizeof(livox_ros_driver2__msg__CustomMsg),
  livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array,  // message members
  livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle = {
  0,
  &livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_livox_ros_driver2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, livox_ros_driver2, msg, CustomMsg)() {
  livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, livox_ros_driver2, msg, CustomPoint)();
  if (!livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle.typesupport_identifier) {
    livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &livox_ros_driver2__msg__CustomMsg__rosidl_typesupport_introspection_c__CustomMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
