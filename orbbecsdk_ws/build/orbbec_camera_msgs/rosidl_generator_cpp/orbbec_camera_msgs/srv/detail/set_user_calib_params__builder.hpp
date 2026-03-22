// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:srv/SetUserCalibParams.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_USER_CALIB_PARAMS__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_USER_CALIB_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/srv/detail/set_user_calib_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_SetUserCalibParams_Request_translation
{
public:
  explicit Init_SetUserCalibParams_Request_translation(::orbbec_camera_msgs::srv::SetUserCalibParams_Request & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Request translation(::orbbec_camera_msgs::srv::SetUserCalibParams_Request::_translation_type arg)
  {
    msg_.translation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Request msg_;
};

class Init_SetUserCalibParams_Request_rotation
{
public:
  explicit Init_SetUserCalibParams_Request_rotation(::orbbec_camera_msgs::srv::SetUserCalibParams_Request & msg)
  : msg_(msg)
  {}
  Init_SetUserCalibParams_Request_translation rotation(::orbbec_camera_msgs::srv::SetUserCalibParams_Request::_rotation_type arg)
  {
    msg_.rotation = std::move(arg);
    return Init_SetUserCalibParams_Request_translation(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Request msg_;
};

class Init_SetUserCalibParams_Request_d
{
public:
  explicit Init_SetUserCalibParams_Request_d(::orbbec_camera_msgs::srv::SetUserCalibParams_Request & msg)
  : msg_(msg)
  {}
  Init_SetUserCalibParams_Request_rotation d(::orbbec_camera_msgs::srv::SetUserCalibParams_Request::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_SetUserCalibParams_Request_rotation(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Request msg_;
};

class Init_SetUserCalibParams_Request_k
{
public:
  Init_SetUserCalibParams_Request_k()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetUserCalibParams_Request_d k(::orbbec_camera_msgs::srv::SetUserCalibParams_Request::_k_type arg)
  {
    msg_.k = std::move(arg);
    return Init_SetUserCalibParams_Request_d(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::SetUserCalibParams_Request>()
{
  return orbbec_camera_msgs::srv::builder::Init_SetUserCalibParams_Request_k();
}

}  // namespace orbbec_camera_msgs


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_SetUserCalibParams_Response_message
{
public:
  explicit Init_SetUserCalibParams_Response_message(::orbbec_camera_msgs::srv::SetUserCalibParams_Response & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Response message(::orbbec_camera_msgs::srv::SetUserCalibParams_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Response msg_;
};

class Init_SetUserCalibParams_Response_success
{
public:
  Init_SetUserCalibParams_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetUserCalibParams_Response_message success(::orbbec_camera_msgs::srv::SetUserCalibParams_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetUserCalibParams_Response_message(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::SetUserCalibParams_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::SetUserCalibParams_Response>()
{
  return orbbec_camera_msgs::srv::builder::Init_SetUserCalibParams_Response_success();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__SET_USER_CALIB_PARAMS__BUILDER_HPP_
