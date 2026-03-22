// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:srv/GetUserCalibParams.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_USER_CALIB_PARAMS__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_USER_CALIB_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/srv/detail/get_user_calib_params__struct.hpp"
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
auto build<::orbbec_camera_msgs::srv::GetUserCalibParams_Request>()
{
  return ::orbbec_camera_msgs::srv::GetUserCalibParams_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace orbbec_camera_msgs


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_GetUserCalibParams_Response_message
{
public:
  explicit Init_GetUserCalibParams_Response_message(::orbbec_camera_msgs::srv::GetUserCalibParams_Response & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response message(::orbbec_camera_msgs::srv::GetUserCalibParams_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response msg_;
};

class Init_GetUserCalibParams_Response_success
{
public:
  explicit Init_GetUserCalibParams_Response_success(::orbbec_camera_msgs::srv::GetUserCalibParams_Response & msg)
  : msg_(msg)
  {}
  Init_GetUserCalibParams_Response_message success(::orbbec_camera_msgs::srv::GetUserCalibParams_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetUserCalibParams_Response_message(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response msg_;
};

class Init_GetUserCalibParams_Response_translation
{
public:
  explicit Init_GetUserCalibParams_Response_translation(::orbbec_camera_msgs::srv::GetUserCalibParams_Response & msg)
  : msg_(msg)
  {}
  Init_GetUserCalibParams_Response_success translation(::orbbec_camera_msgs::srv::GetUserCalibParams_Response::_translation_type arg)
  {
    msg_.translation = std::move(arg);
    return Init_GetUserCalibParams_Response_success(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response msg_;
};

class Init_GetUserCalibParams_Response_rotation
{
public:
  explicit Init_GetUserCalibParams_Response_rotation(::orbbec_camera_msgs::srv::GetUserCalibParams_Response & msg)
  : msg_(msg)
  {}
  Init_GetUserCalibParams_Response_translation rotation(::orbbec_camera_msgs::srv::GetUserCalibParams_Response::_rotation_type arg)
  {
    msg_.rotation = std::move(arg);
    return Init_GetUserCalibParams_Response_translation(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response msg_;
};

class Init_GetUserCalibParams_Response_d
{
public:
  explicit Init_GetUserCalibParams_Response_d(::orbbec_camera_msgs::srv::GetUserCalibParams_Response & msg)
  : msg_(msg)
  {}
  Init_GetUserCalibParams_Response_rotation d(::orbbec_camera_msgs::srv::GetUserCalibParams_Response::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_GetUserCalibParams_Response_rotation(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response msg_;
};

class Init_GetUserCalibParams_Response_k
{
public:
  Init_GetUserCalibParams_Response_k()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetUserCalibParams_Response_d k(::orbbec_camera_msgs::srv::GetUserCalibParams_Response::_k_type arg)
  {
    msg_.k = std::move(arg);
    return Init_GetUserCalibParams_Response_d(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetUserCalibParams_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::GetUserCalibParams_Response>()
{
  return orbbec_camera_msgs::srv::builder::Init_GetUserCalibParams_Response_k();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_USER_CALIB_PARAMS__BUILDER_HPP_
