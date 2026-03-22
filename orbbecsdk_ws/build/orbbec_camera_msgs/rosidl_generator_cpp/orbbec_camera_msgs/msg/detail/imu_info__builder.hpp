// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/msg/detail/imu_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace msg
{

namespace builder
{

class Init_IMUInfo_temperature_slope
{
public:
  explicit Init_IMUInfo_temperature_slope(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::msg::IMUInfo temperature_slope(::orbbec_camera_msgs::msg::IMUInfo::_temperature_slope_type arg)
  {
    msg_.temperature_slope = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_scale_misalignment
{
public:
  explicit Init_IMUInfo_scale_misalignment(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  Init_IMUInfo_temperature_slope scale_misalignment(::orbbec_camera_msgs::msg::IMUInfo::_scale_misalignment_type arg)
  {
    msg_.scale_misalignment = std::move(arg);
    return Init_IMUInfo_temperature_slope(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_gravity
{
public:
  explicit Init_IMUInfo_gravity(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  Init_IMUInfo_scale_misalignment gravity(::orbbec_camera_msgs::msg::IMUInfo::_gravity_type arg)
  {
    msg_.gravity = std::move(arg);
    return Init_IMUInfo_scale_misalignment(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_bias
{
public:
  explicit Init_IMUInfo_bias(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  Init_IMUInfo_gravity bias(::orbbec_camera_msgs::msg::IMUInfo::_bias_type arg)
  {
    msg_.bias = std::move(arg);
    return Init_IMUInfo_gravity(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_reference_temperature
{
public:
  explicit Init_IMUInfo_reference_temperature(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  Init_IMUInfo_bias reference_temperature(::orbbec_camera_msgs::msg::IMUInfo::_reference_temperature_type arg)
  {
    msg_.reference_temperature = std::move(arg);
    return Init_IMUInfo_bias(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_random_walk
{
public:
  explicit Init_IMUInfo_random_walk(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  Init_IMUInfo_reference_temperature random_walk(::orbbec_camera_msgs::msg::IMUInfo::_random_walk_type arg)
  {
    msg_.random_walk = std::move(arg);
    return Init_IMUInfo_reference_temperature(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_noise_density
{
public:
  explicit Init_IMUInfo_noise_density(::orbbec_camera_msgs::msg::IMUInfo & msg)
  : msg_(msg)
  {}
  Init_IMUInfo_random_walk noise_density(::orbbec_camera_msgs::msg::IMUInfo::_noise_density_type arg)
  {
    msg_.noise_density = std::move(arg);
    return Init_IMUInfo_random_walk(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

class Init_IMUInfo_header
{
public:
  Init_IMUInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMUInfo_noise_density header(::orbbec_camera_msgs::msg::IMUInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_IMUInfo_noise_density(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::IMUInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::msg::IMUInfo>()
{
  return orbbec_camera_msgs::msg::builder::Init_IMUInfo_header();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__BUILDER_HPP_
