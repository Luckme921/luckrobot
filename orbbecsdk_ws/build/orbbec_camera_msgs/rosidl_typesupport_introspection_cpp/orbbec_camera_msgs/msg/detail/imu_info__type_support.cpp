// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from orbbec_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "orbbec_camera_msgs/msg/detail/imu_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace orbbec_camera_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void IMUInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) orbbec_camera_msgs::msg::IMUInfo(_init);
}

void IMUInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<orbbec_camera_msgs::msg::IMUInfo *>(message_memory);
  typed_message->~IMUInfo();
}

size_t size_function__IMUInfo__bias(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUInfo__bias(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUInfo__bias(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUInfo__bias(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUInfo__bias(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUInfo__bias(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUInfo__bias(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__IMUInfo__gravity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__IMUInfo__gravity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUInfo__gravity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUInfo__gravity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUInfo__gravity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUInfo__gravity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUInfo__gravity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__IMUInfo__scale_misalignment(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__IMUInfo__scale_misalignment(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUInfo__scale_misalignment(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUInfo__scale_misalignment(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUInfo__scale_misalignment(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUInfo__scale_misalignment(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUInfo__scale_misalignment(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__IMUInfo__temperature_slope(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__IMUInfo__temperature_slope(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__IMUInfo__temperature_slope(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__IMUInfo__temperature_slope(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__IMUInfo__temperature_slope(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__IMUInfo__temperature_slope(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__IMUInfo__temperature_slope(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember IMUInfo_message_member_array[8] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "noise_density",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, noise_density),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "random_walk",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, random_walk),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "reference_temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, reference_temperature),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bias",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, bias),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUInfo__bias,  // size() function pointer
    get_const_function__IMUInfo__bias,  // get_const(index) function pointer
    get_function__IMUInfo__bias,  // get(index) function pointer
    fetch_function__IMUInfo__bias,  // fetch(index, &value) function pointer
    assign_function__IMUInfo__bias,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gravity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, gravity),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUInfo__gravity,  // size() function pointer
    get_const_function__IMUInfo__gravity,  // get_const(index) function pointer
    get_function__IMUInfo__gravity,  // get(index) function pointer
    fetch_function__IMUInfo__gravity,  // fetch(index, &value) function pointer
    assign_function__IMUInfo__gravity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "scale_misalignment",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, scale_misalignment),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUInfo__scale_misalignment,  // size() function pointer
    get_const_function__IMUInfo__scale_misalignment,  // get_const(index) function pointer
    get_function__IMUInfo__scale_misalignment,  // get(index) function pointer
    fetch_function__IMUInfo__scale_misalignment,  // fetch(index, &value) function pointer
    assign_function__IMUInfo__scale_misalignment,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "temperature_slope",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::msg::IMUInfo, temperature_slope),  // bytes offset in struct
    nullptr,  // default value
    size_function__IMUInfo__temperature_slope,  // size() function pointer
    get_const_function__IMUInfo__temperature_slope,  // get_const(index) function pointer
    get_function__IMUInfo__temperature_slope,  // get(index) function pointer
    fetch_function__IMUInfo__temperature_slope,  // fetch(index, &value) function pointer
    assign_function__IMUInfo__temperature_slope,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers IMUInfo_message_members = {
  "orbbec_camera_msgs::msg",  // message namespace
  "IMUInfo",  // message name
  8,  // number of fields
  sizeof(orbbec_camera_msgs::msg::IMUInfo),
  IMUInfo_message_member_array,  // message members
  IMUInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t IMUInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &IMUInfo_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace orbbec_camera_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<orbbec_camera_msgs::msg::IMUInfo>()
{
  return &::orbbec_camera_msgs::msg::rosidl_typesupport_introspection_cpp::IMUInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, orbbec_camera_msgs, msg, IMUInfo)() {
  return &::orbbec_camera_msgs::msg::rosidl_typesupport_introspection_cpp::IMUInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
