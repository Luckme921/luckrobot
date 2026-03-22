// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:srv/GetCameraInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/srv/detail/get_camera_info__struct.hpp"
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
auto build<::orbbec_camera_msgs::srv::GetCameraInfo_Request>()
{
  return ::orbbec_camera_msgs::srv::GetCameraInfo_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace orbbec_camera_msgs


namespace orbbec_camera_msgs
{

namespace srv
{

namespace builder
{

class Init_GetCameraInfo_Response_info
{
public:
  Init_GetCameraInfo_Response_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::orbbec_camera_msgs::srv::GetCameraInfo_Response info(::orbbec_camera_msgs::srv::GetCameraInfo_Response::_info_type arg)
  {
    msg_.info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::srv::GetCameraInfo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::srv::GetCameraInfo_Response>()
{
  return orbbec_camera_msgs::srv::builder::Init_GetCameraInfo_Response_info();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__BUILDER_HPP_
