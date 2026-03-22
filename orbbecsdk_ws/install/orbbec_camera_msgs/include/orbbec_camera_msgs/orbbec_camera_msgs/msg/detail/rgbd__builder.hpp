// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:msg/RGBD.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__RGBD__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__RGBD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/msg/detail/rgbd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace msg
{

namespace builder
{

class Init_RGBD_depth
{
public:
  explicit Init_RGBD_depth(::orbbec_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::msg::RGBD depth(::orbbec_camera_msgs::msg::RGBD::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_rgb
{
public:
  explicit Init_RGBD_rgb(::orbbec_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  Init_RGBD_depth rgb(::orbbec_camera_msgs::msg::RGBD::_rgb_type arg)
  {
    msg_.rgb = std::move(arg);
    return Init_RGBD_depth(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_depth_camera_info
{
public:
  explicit Init_RGBD_depth_camera_info(::orbbec_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  Init_RGBD_rgb depth_camera_info(::orbbec_camera_msgs::msg::RGBD::_depth_camera_info_type arg)
  {
    msg_.depth_camera_info = std::move(arg);
    return Init_RGBD_rgb(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_rgb_camera_info
{
public:
  explicit Init_RGBD_rgb_camera_info(::orbbec_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  Init_RGBD_depth_camera_info rgb_camera_info(::orbbec_camera_msgs::msg::RGBD::_rgb_camera_info_type arg)
  {
    msg_.rgb_camera_info = std::move(arg);
    return Init_RGBD_depth_camera_info(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_header
{
public:
  Init_RGBD_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RGBD_rgb_camera_info header(::orbbec_camera_msgs::msg::RGBD::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RGBD_rgb_camera_info(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::RGBD msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::msg::RGBD>()
{
  return orbbec_camera_msgs::msg::builder::Init_RGBD_header();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__RGBD__BUILDER_HPP_
