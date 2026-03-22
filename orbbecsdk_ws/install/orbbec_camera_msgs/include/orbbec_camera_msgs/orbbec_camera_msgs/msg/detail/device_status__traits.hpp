// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__TRAITS_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "orbbec_camera_msgs/msg/detail/device_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace orbbec_camera_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DeviceStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: color_frame_rate_cur
  {
    out << "color_frame_rate_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_cur, out);
    out << ", ";
  }

  // member: color_frame_rate_avg
  {
    out << "color_frame_rate_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_avg, out);
    out << ", ";
  }

  // member: color_frame_rate_min
  {
    out << "color_frame_rate_min: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_min, out);
    out << ", ";
  }

  // member: color_frame_rate_max
  {
    out << "color_frame_rate_max: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_max, out);
    out << ", ";
  }

  // member: color_delay_ms_cur
  {
    out << "color_delay_ms_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_cur, out);
    out << ", ";
  }

  // member: color_delay_ms_avg
  {
    out << "color_delay_ms_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_avg, out);
    out << ", ";
  }

  // member: color_delay_ms_min
  {
    out << "color_delay_ms_min: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_min, out);
    out << ", ";
  }

  // member: color_delay_ms_max
  {
    out << "color_delay_ms_max: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_max, out);
    out << ", ";
  }

  // member: depth_frame_rate_cur
  {
    out << "depth_frame_rate_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_cur, out);
    out << ", ";
  }

  // member: depth_frame_rate_avg
  {
    out << "depth_frame_rate_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_avg, out);
    out << ", ";
  }

  // member: depth_frame_rate_min
  {
    out << "depth_frame_rate_min: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_min, out);
    out << ", ";
  }

  // member: depth_frame_rate_max
  {
    out << "depth_frame_rate_max: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_max, out);
    out << ", ";
  }

  // member: depth_delay_ms_cur
  {
    out << "depth_delay_ms_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_cur, out);
    out << ", ";
  }

  // member: depth_delay_ms_avg
  {
    out << "depth_delay_ms_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_avg, out);
    out << ", ";
  }

  // member: depth_delay_ms_min
  {
    out << "depth_delay_ms_min: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_min, out);
    out << ", ";
  }

  // member: depth_delay_ms_max
  {
    out << "depth_delay_ms_max: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_max, out);
    out << ", ";
  }

  // member: device_online
  {
    out << "device_online: ";
    rosidl_generator_traits::value_to_yaml(msg.device_online, out);
    out << ", ";
  }

  // member: connection_type
  {
    out << "connection_type: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_type, out);
    out << ", ";
  }

  // member: customer_calibration_ready
  {
    out << "customer_calibration_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.customer_calibration_ready, out);
    out << ", ";
  }

  // member: calibration_from_factory
  {
    out << "calibration_from_factory: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_from_factory, out);
    out << ", ";
  }

  // member: calibration_from_launch_param
  {
    out << "calibration_from_launch_param: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_from_launch_param, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DeviceStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: color_frame_rate_cur
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_frame_rate_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_cur, out);
    out << "\n";
  }

  // member: color_frame_rate_avg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_frame_rate_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_avg, out);
    out << "\n";
  }

  // member: color_frame_rate_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_frame_rate_min: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_min, out);
    out << "\n";
  }

  // member: color_frame_rate_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_frame_rate_max: ";
    rosidl_generator_traits::value_to_yaml(msg.color_frame_rate_max, out);
    out << "\n";
  }

  // member: color_delay_ms_cur
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_delay_ms_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_cur, out);
    out << "\n";
  }

  // member: color_delay_ms_avg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_delay_ms_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_avg, out);
    out << "\n";
  }

  // member: color_delay_ms_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_delay_ms_min: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_min, out);
    out << "\n";
  }

  // member: color_delay_ms_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_delay_ms_max: ";
    rosidl_generator_traits::value_to_yaml(msg.color_delay_ms_max, out);
    out << "\n";
  }

  // member: depth_frame_rate_cur
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_frame_rate_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_cur, out);
    out << "\n";
  }

  // member: depth_frame_rate_avg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_frame_rate_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_avg, out);
    out << "\n";
  }

  // member: depth_frame_rate_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_frame_rate_min: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_min, out);
    out << "\n";
  }

  // member: depth_frame_rate_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_frame_rate_max: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_frame_rate_max, out);
    out << "\n";
  }

  // member: depth_delay_ms_cur
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_delay_ms_cur: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_cur, out);
    out << "\n";
  }

  // member: depth_delay_ms_avg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_delay_ms_avg: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_avg, out);
    out << "\n";
  }

  // member: depth_delay_ms_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_delay_ms_min: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_min, out);
    out << "\n";
  }

  // member: depth_delay_ms_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_delay_ms_max: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_delay_ms_max, out);
    out << "\n";
  }

  // member: device_online
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_online: ";
    rosidl_generator_traits::value_to_yaml(msg.device_online, out);
    out << "\n";
  }

  // member: connection_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_type: ";
    rosidl_generator_traits::value_to_yaml(msg.connection_type, out);
    out << "\n";
  }

  // member: customer_calibration_ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "customer_calibration_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.customer_calibration_ready, out);
    out << "\n";
  }

  // member: calibration_from_factory
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_from_factory: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_from_factory, out);
    out << "\n";
  }

  // member: calibration_from_launch_param
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibration_from_launch_param: ";
    rosidl_generator_traits::value_to_yaml(msg.calibration_from_launch_param, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DeviceStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace orbbec_camera_msgs

namespace rosidl_generator_traits
{

[[deprecated("use orbbec_camera_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const orbbec_camera_msgs::msg::DeviceStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  orbbec_camera_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orbbec_camera_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const orbbec_camera_msgs::msg::DeviceStatus & msg)
{
  return orbbec_camera_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<orbbec_camera_msgs::msg::DeviceStatus>()
{
  return "orbbec_camera_msgs::msg::DeviceStatus";
}

template<>
inline const char * name<orbbec_camera_msgs::msg::DeviceStatus>()
{
  return "orbbec_camera_msgs/msg/DeviceStatus";
}

template<>
struct has_fixed_size<orbbec_camera_msgs::msg::DeviceStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<orbbec_camera_msgs::msg::DeviceStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<orbbec_camera_msgs::msg::DeviceStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__TRAITS_HPP_
