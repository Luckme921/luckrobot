// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/msg/detail/device_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace msg
{

namespace builder
{

class Init_DeviceStatus_calibration_from_launch_param
{
public:
  explicit Init_DeviceStatus_calibration_from_launch_param(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::msg::DeviceStatus calibration_from_launch_param(::orbbec_camera_msgs::msg::DeviceStatus::_calibration_from_launch_param_type arg)
  {
    msg_.calibration_from_launch_param = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_calibration_from_factory
{
public:
  explicit Init_DeviceStatus_calibration_from_factory(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_calibration_from_launch_param calibration_from_factory(::orbbec_camera_msgs::msg::DeviceStatus::_calibration_from_factory_type arg)
  {
    msg_.calibration_from_factory = std::move(arg);
    return Init_DeviceStatus_calibration_from_launch_param(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_customer_calibration_ready
{
public:
  explicit Init_DeviceStatus_customer_calibration_ready(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_calibration_from_factory customer_calibration_ready(::orbbec_camera_msgs::msg::DeviceStatus::_customer_calibration_ready_type arg)
  {
    msg_.customer_calibration_ready = std::move(arg);
    return Init_DeviceStatus_calibration_from_factory(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_connection_type
{
public:
  explicit Init_DeviceStatus_connection_type(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_customer_calibration_ready connection_type(::orbbec_camera_msgs::msg::DeviceStatus::_connection_type_type arg)
  {
    msg_.connection_type = std::move(arg);
    return Init_DeviceStatus_customer_calibration_ready(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_device_online
{
public:
  explicit Init_DeviceStatus_device_online(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_connection_type device_online(::orbbec_camera_msgs::msg::DeviceStatus::_device_online_type arg)
  {
    msg_.device_online = std::move(arg);
    return Init_DeviceStatus_connection_type(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_delay_ms_max
{
public:
  explicit Init_DeviceStatus_depth_delay_ms_max(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_device_online depth_delay_ms_max(::orbbec_camera_msgs::msg::DeviceStatus::_depth_delay_ms_max_type arg)
  {
    msg_.depth_delay_ms_max = std::move(arg);
    return Init_DeviceStatus_device_online(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_delay_ms_min
{
public:
  explicit Init_DeviceStatus_depth_delay_ms_min(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_delay_ms_max depth_delay_ms_min(::orbbec_camera_msgs::msg::DeviceStatus::_depth_delay_ms_min_type arg)
  {
    msg_.depth_delay_ms_min = std::move(arg);
    return Init_DeviceStatus_depth_delay_ms_max(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_delay_ms_avg
{
public:
  explicit Init_DeviceStatus_depth_delay_ms_avg(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_delay_ms_min depth_delay_ms_avg(::orbbec_camera_msgs::msg::DeviceStatus::_depth_delay_ms_avg_type arg)
  {
    msg_.depth_delay_ms_avg = std::move(arg);
    return Init_DeviceStatus_depth_delay_ms_min(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_delay_ms_cur
{
public:
  explicit Init_DeviceStatus_depth_delay_ms_cur(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_delay_ms_avg depth_delay_ms_cur(::orbbec_camera_msgs::msg::DeviceStatus::_depth_delay_ms_cur_type arg)
  {
    msg_.depth_delay_ms_cur = std::move(arg);
    return Init_DeviceStatus_depth_delay_ms_avg(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_frame_rate_max
{
public:
  explicit Init_DeviceStatus_depth_frame_rate_max(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_delay_ms_cur depth_frame_rate_max(::orbbec_camera_msgs::msg::DeviceStatus::_depth_frame_rate_max_type arg)
  {
    msg_.depth_frame_rate_max = std::move(arg);
    return Init_DeviceStatus_depth_delay_ms_cur(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_frame_rate_min
{
public:
  explicit Init_DeviceStatus_depth_frame_rate_min(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_frame_rate_max depth_frame_rate_min(::orbbec_camera_msgs::msg::DeviceStatus::_depth_frame_rate_min_type arg)
  {
    msg_.depth_frame_rate_min = std::move(arg);
    return Init_DeviceStatus_depth_frame_rate_max(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_frame_rate_avg
{
public:
  explicit Init_DeviceStatus_depth_frame_rate_avg(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_frame_rate_min depth_frame_rate_avg(::orbbec_camera_msgs::msg::DeviceStatus::_depth_frame_rate_avg_type arg)
  {
    msg_.depth_frame_rate_avg = std::move(arg);
    return Init_DeviceStatus_depth_frame_rate_min(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_depth_frame_rate_cur
{
public:
  explicit Init_DeviceStatus_depth_frame_rate_cur(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_frame_rate_avg depth_frame_rate_cur(::orbbec_camera_msgs::msg::DeviceStatus::_depth_frame_rate_cur_type arg)
  {
    msg_.depth_frame_rate_cur = std::move(arg);
    return Init_DeviceStatus_depth_frame_rate_avg(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_delay_ms_max
{
public:
  explicit Init_DeviceStatus_color_delay_ms_max(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_depth_frame_rate_cur color_delay_ms_max(::orbbec_camera_msgs::msg::DeviceStatus::_color_delay_ms_max_type arg)
  {
    msg_.color_delay_ms_max = std::move(arg);
    return Init_DeviceStatus_depth_frame_rate_cur(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_delay_ms_min
{
public:
  explicit Init_DeviceStatus_color_delay_ms_min(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_delay_ms_max color_delay_ms_min(::orbbec_camera_msgs::msg::DeviceStatus::_color_delay_ms_min_type arg)
  {
    msg_.color_delay_ms_min = std::move(arg);
    return Init_DeviceStatus_color_delay_ms_max(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_delay_ms_avg
{
public:
  explicit Init_DeviceStatus_color_delay_ms_avg(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_delay_ms_min color_delay_ms_avg(::orbbec_camera_msgs::msg::DeviceStatus::_color_delay_ms_avg_type arg)
  {
    msg_.color_delay_ms_avg = std::move(arg);
    return Init_DeviceStatus_color_delay_ms_min(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_delay_ms_cur
{
public:
  explicit Init_DeviceStatus_color_delay_ms_cur(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_delay_ms_avg color_delay_ms_cur(::orbbec_camera_msgs::msg::DeviceStatus::_color_delay_ms_cur_type arg)
  {
    msg_.color_delay_ms_cur = std::move(arg);
    return Init_DeviceStatus_color_delay_ms_avg(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_frame_rate_max
{
public:
  explicit Init_DeviceStatus_color_frame_rate_max(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_delay_ms_cur color_frame_rate_max(::orbbec_camera_msgs::msg::DeviceStatus::_color_frame_rate_max_type arg)
  {
    msg_.color_frame_rate_max = std::move(arg);
    return Init_DeviceStatus_color_delay_ms_cur(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_frame_rate_min
{
public:
  explicit Init_DeviceStatus_color_frame_rate_min(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_frame_rate_max color_frame_rate_min(::orbbec_camera_msgs::msg::DeviceStatus::_color_frame_rate_min_type arg)
  {
    msg_.color_frame_rate_min = std::move(arg);
    return Init_DeviceStatus_color_frame_rate_max(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_frame_rate_avg
{
public:
  explicit Init_DeviceStatus_color_frame_rate_avg(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_frame_rate_min color_frame_rate_avg(::orbbec_camera_msgs::msg::DeviceStatus::_color_frame_rate_avg_type arg)
  {
    msg_.color_frame_rate_avg = std::move(arg);
    return Init_DeviceStatus_color_frame_rate_min(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_color_frame_rate_cur
{
public:
  explicit Init_DeviceStatus_color_frame_rate_cur(::orbbec_camera_msgs::msg::DeviceStatus & msg)
  : msg_(msg)
  {}
  Init_DeviceStatus_color_frame_rate_avg color_frame_rate_cur(::orbbec_camera_msgs::msg::DeviceStatus::_color_frame_rate_cur_type arg)
  {
    msg_.color_frame_rate_cur = std::move(arg);
    return Init_DeviceStatus_color_frame_rate_avg(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

class Init_DeviceStatus_header
{
public:
  Init_DeviceStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DeviceStatus_color_frame_rate_cur header(::orbbec_camera_msgs::msg::DeviceStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DeviceStatus_color_frame_rate_cur(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::DeviceStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::msg::DeviceStatus>()
{
  return orbbec_camera_msgs::msg::builder::Init_DeviceStatus_header();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__BUILDER_HPP_
