// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from orbbec_camera_msgs:srv/SetUserCalibParams.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "orbbec_camera_msgs/srv/detail/set_user_calib_params__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace orbbec_camera_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetUserCalibParams_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) orbbec_camera_msgs::srv::SetUserCalibParams_Request(_init);
}

void SetUserCalibParams_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<orbbec_camera_msgs::srv::SetUserCalibParams_Request *>(message_memory);
  typed_message->~SetUserCalibParams_Request();
}

size_t size_function__SetUserCalibParams_Request__k(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__SetUserCalibParams_Request__k(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__SetUserCalibParams_Request__k(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetUserCalibParams_Request__k(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SetUserCalibParams_Request__k(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SetUserCalibParams_Request__k(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SetUserCalibParams_Request__k(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__SetUserCalibParams_Request__d(const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * get_const_function__SetUserCalibParams_Request__d(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 8> *>(untyped_member);
  return &member[index];
}

void * get_function__SetUserCalibParams_Request__d(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 8> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetUserCalibParams_Request__d(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SetUserCalibParams_Request__d(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SetUserCalibParams_Request__d(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SetUserCalibParams_Request__d(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__SetUserCalibParams_Request__rotation(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__SetUserCalibParams_Request__rotation(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__SetUserCalibParams_Request__rotation(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetUserCalibParams_Request__rotation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SetUserCalibParams_Request__rotation(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SetUserCalibParams_Request__rotation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SetUserCalibParams_Request__rotation(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__SetUserCalibParams_Request__translation(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__SetUserCalibParams_Request__translation(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__SetUserCalibParams_Request__translation(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetUserCalibParams_Request__translation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SetUserCalibParams_Request__translation(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SetUserCalibParams_Request__translation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SetUserCalibParams_Request__translation(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetUserCalibParams_Request_message_member_array[4] = {
  {
    "k",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::srv::SetUserCalibParams_Request, k),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetUserCalibParams_Request__k,  // size() function pointer
    get_const_function__SetUserCalibParams_Request__k,  // get_const(index) function pointer
    get_function__SetUserCalibParams_Request__k,  // get(index) function pointer
    fetch_function__SetUserCalibParams_Request__k,  // fetch(index, &value) function pointer
    assign_function__SetUserCalibParams_Request__k,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::srv::SetUserCalibParams_Request, d),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetUserCalibParams_Request__d,  // size() function pointer
    get_const_function__SetUserCalibParams_Request__d,  // get_const(index) function pointer
    get_function__SetUserCalibParams_Request__d,  // get(index) function pointer
    fetch_function__SetUserCalibParams_Request__d,  // fetch(index, &value) function pointer
    assign_function__SetUserCalibParams_Request__d,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rotation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::srv::SetUserCalibParams_Request, rotation),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetUserCalibParams_Request__rotation,  // size() function pointer
    get_const_function__SetUserCalibParams_Request__rotation,  // get_const(index) function pointer
    get_function__SetUserCalibParams_Request__rotation,  // get(index) function pointer
    fetch_function__SetUserCalibParams_Request__rotation,  // fetch(index, &value) function pointer
    assign_function__SetUserCalibParams_Request__rotation,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "translation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::srv::SetUserCalibParams_Request, translation),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetUserCalibParams_Request__translation,  // size() function pointer
    get_const_function__SetUserCalibParams_Request__translation,  // get_const(index) function pointer
    get_function__SetUserCalibParams_Request__translation,  // get(index) function pointer
    fetch_function__SetUserCalibParams_Request__translation,  // fetch(index, &value) function pointer
    assign_function__SetUserCalibParams_Request__translation,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetUserCalibParams_Request_message_members = {
  "orbbec_camera_msgs::srv",  // message namespace
  "SetUserCalibParams_Request",  // message name
  4,  // number of fields
  sizeof(orbbec_camera_msgs::srv::SetUserCalibParams_Request),
  SetUserCalibParams_Request_message_member_array,  // message members
  SetUserCalibParams_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetUserCalibParams_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetUserCalibParams_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetUserCalibParams_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace orbbec_camera_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<orbbec_camera_msgs::srv::SetUserCalibParams_Request>()
{
  return &::orbbec_camera_msgs::srv::rosidl_typesupport_introspection_cpp::SetUserCalibParams_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, orbbec_camera_msgs, srv, SetUserCalibParams_Request)() {
  return &::orbbec_camera_msgs::srv::rosidl_typesupport_introspection_cpp::SetUserCalibParams_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "orbbec_camera_msgs/srv/detail/set_user_calib_params__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace orbbec_camera_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetUserCalibParams_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) orbbec_camera_msgs::srv::SetUserCalibParams_Response(_init);
}

void SetUserCalibParams_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<orbbec_camera_msgs::srv::SetUserCalibParams_Response *>(message_memory);
  typed_message->~SetUserCalibParams_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetUserCalibParams_Response_message_member_array[2] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::srv::SetUserCalibParams_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(orbbec_camera_msgs::srv::SetUserCalibParams_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetUserCalibParams_Response_message_members = {
  "orbbec_camera_msgs::srv",  // message namespace
  "SetUserCalibParams_Response",  // message name
  2,  // number of fields
  sizeof(orbbec_camera_msgs::srv::SetUserCalibParams_Response),
  SetUserCalibParams_Response_message_member_array,  // message members
  SetUserCalibParams_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetUserCalibParams_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetUserCalibParams_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetUserCalibParams_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace orbbec_camera_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<orbbec_camera_msgs::srv::SetUserCalibParams_Response>()
{
  return &::orbbec_camera_msgs::srv::rosidl_typesupport_introspection_cpp::SetUserCalibParams_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, orbbec_camera_msgs, srv, SetUserCalibParams_Response)() {
  return &::orbbec_camera_msgs::srv::rosidl_typesupport_introspection_cpp::SetUserCalibParams_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "orbbec_camera_msgs/srv/detail/set_user_calib_params__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace orbbec_camera_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SetUserCalibParams_service_members = {
  "orbbec_camera_msgs::srv",  // service namespace
  "SetUserCalibParams",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<orbbec_camera_msgs::srv::SetUserCalibParams>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SetUserCalibParams_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetUserCalibParams_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace orbbec_camera_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<orbbec_camera_msgs::srv::SetUserCalibParams>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::orbbec_camera_msgs::srv::rosidl_typesupport_introspection_cpp::SetUserCalibParams_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::orbbec_camera_msgs::srv::SetUserCalibParams_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::orbbec_camera_msgs::srv::SetUserCalibParams_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, orbbec_camera_msgs, srv, SetUserCalibParams)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<orbbec_camera_msgs::srv::SetUserCalibParams>();
}

#ifdef __cplusplus
}
#endif
