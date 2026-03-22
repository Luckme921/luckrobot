// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:srv/SetFilter.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_FILTER__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_FILTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/srv/detail/set_filter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_SetFilter_Request_filter_param
{
public:
  explicit Init_SetFilter_Request_filter_param(::orbbec_camera_msgs::srv::SetFilter_Request & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::srv::SetFilter_Request filter_param(::orbbec_camera_msgs::srv::SetFilter_Request::_filter_param_type arg)
  {
    msg_.filter_param = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetFilter_Request msg_;
};

class Init_SetFilter_Request_filter_enable
{
public:
  explicit Init_SetFilter_Request_filter_enable(::orbbec_camera_msgs::srv::SetFilter_Request & msg)
  : msg_(msg)
  {}
  Init_SetFilter_Request_filter_param filter_enable(::orbbec_camera_msgs::srv::SetFilter_Request::_filter_enable_type arg)
  {
    msg_.filter_enable = std::move(arg);
    return Init_SetFilter_Request_filter_param(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetFilter_Request msg_;
};

class Init_SetFilter_Request_filter_name
{
public:
  Init_SetFilter_Request_filter_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetFilter_Request_filter_enable filter_name(::orbbec_camera_msgs::srv::SetFilter_Request::_filter_name_type arg)
  {
    msg_.filter_name = std::move(arg);
    return Init_SetFilter_Request_filter_enable(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetFilter_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::SetFilter_Request>()
{
  return orbbec_camera_msgs::srv::builder::Init_SetFilter_Request_filter_name();
}

}  // namespace orbbec_camera_msgs


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_SetFilter_Response_message
{
public:
  explicit Init_SetFilter_Response_message(::orbbec_camera_msgs::srv::SetFilter_Response & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::srv::SetFilter_Response message(::orbbec_camera_msgs::srv::SetFilter_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetFilter_Response msg_;
};

class Init_SetFilter_Response_success
{
public:
  Init_SetFilter_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetFilter_Response_message success(::orbbec_camera_msgs::srv::SetFilter_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetFilter_Response_message(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetFilter_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::SetFilter_Response>()
{
  return orbbec_camera_msgs::srv::builder::Init_SetFilter_Response_success();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_FILTER__BUILDER_HPP_
