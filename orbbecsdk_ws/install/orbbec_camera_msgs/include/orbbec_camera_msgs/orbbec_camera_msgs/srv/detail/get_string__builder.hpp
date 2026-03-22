// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:srv/GetString.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_STRING__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_STRING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/srv/detail/get_string__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::GetString_Request>()
{
  return ::orbbec_camera_msgs::srv::GetString_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace orbbec_camera_msgs


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_GetString_Response_message
{
public:
  explicit Init_GetString_Response_message(::orbbec_camera_msgs::srv::GetString_Response & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::srv::GetString_Response message(::orbbec_camera_msgs::srv::GetString_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetString_Response msg_;
};

class Init_GetString_Response_success
{
public:
  explicit Init_GetString_Response_success(::orbbec_camera_msgs::srv::GetString_Response & msg)
  : msg_(msg)
  {}
  Init_GetString_Response_message success(::orbbec_camera_msgs::srv::GetString_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetString_Response_message(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetString_Response msg_;
};

class Init_GetString_Response_data
{
public:
  Init_GetString_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetString_Response_success data(::orbbec_camera_msgs::srv::GetString_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_GetString_Response_success(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetString_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::GetString_Response>()
{
  return orbbec_camera_msgs::srv::builder::Init_GetString_Response_data();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_STRING__BUILDER_HPP_
