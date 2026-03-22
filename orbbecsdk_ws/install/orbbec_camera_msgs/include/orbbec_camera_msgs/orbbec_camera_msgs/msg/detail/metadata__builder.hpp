// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orbbec_camera_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orbbec_camera_msgs/msg/detail/metadata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orbbec_camera_msgs
{

namespace msg
{

namespace builder
{

class Init_Metadata_json_data
{
public:
  explicit Init_Metadata_json_data(::orbbec_camera_msgs::msg::Metadata & msg)
  : msg_(msg)
  {}
  ::orbbec_camera_msgs::msg::Metadata json_data(::orbbec_camera_msgs::msg::Metadata::_json_data_type arg)
  {
    msg_.json_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::Metadata msg_;
};

class Init_Metadata_header
{
public:
  Init_Metadata_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Metadata_json_data header(::orbbec_camera_msgs::msg::Metadata::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Metadata_json_data(msg_);
  }

private:
  ::orbbec_camera_msgs::msg::Metadata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orbbec_camera_msgs::msg::Metadata>()
{
  return orbbec_camera_msgs::msg::builder::Init_Metadata_header();
}

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_
