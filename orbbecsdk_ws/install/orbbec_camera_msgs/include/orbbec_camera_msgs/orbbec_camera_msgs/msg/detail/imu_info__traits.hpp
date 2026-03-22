// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from orbbec_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__TRAITS_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "orbbec_camera_msgs/msg/detail/imu_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace orbbec_camera_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const IMUInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: noise_density
  {
    out << "noise_density: ";
    rosidl_generator_traits::value_to_yaml(msg.noise_density, out);
    out << ", ";
  }

  // member: random_walk
  {
    out << "random_walk: ";
    rosidl_generator_traits::value_to_yaml(msg.random_walk, out);
    out << ", ";
  }

  // member: reference_temperature
  {
    out << "reference_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_temperature, out);
    out << ", ";
  }

  // member: bias
  {
    if (msg.bias.size() == 0) {
      out << "bias: []";
    } else {
      out << "bias: [";
      size_t pending_items = msg.bias.size();
      for (auto item : msg.bias) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gravity
  {
    if (msg.gravity.size() == 0) {
      out << "gravity: []";
    } else {
      out << "gravity: [";
      size_t pending_items = msg.gravity.size();
      for (auto item : msg.gravity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: scale_misalignment
  {
    if (msg.scale_misalignment.size() == 0) {
      out << "scale_misalignment: []";
    } else {
      out << "scale_misalignment: [";
      size_t pending_items = msg.scale_misalignment.size();
      for (auto item : msg.scale_misalignment) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: temperature_slope
  {
    if (msg.temperature_slope.size() == 0) {
      out << "temperature_slope: []";
    } else {
      out << "temperature_slope: [";
      size_t pending_items = msg.temperature_slope.size();
      for (auto item : msg.temperature_slope) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IMUInfo & msg,
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

  // member: noise_density
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "noise_density: ";
    rosidl_generator_traits::value_to_yaml(msg.noise_density, out);
    out << "\n";
  }

  // member: random_walk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "random_walk: ";
    rosidl_generator_traits::value_to_yaml(msg.random_walk, out);
    out << "\n";
  }

  // member: reference_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reference_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.reference_temperature, out);
    out << "\n";
  }

  // member: bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bias.size() == 0) {
      out << "bias: []\n";
    } else {
      out << "bias:\n";
      for (auto item : msg.bias) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gravity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gravity.size() == 0) {
      out << "gravity: []\n";
    } else {
      out << "gravity:\n";
      for (auto item : msg.gravity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: scale_misalignment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scale_misalignment.size() == 0) {
      out << "scale_misalignment: []\n";
    } else {
      out << "scale_misalignment:\n";
      for (auto item : msg.scale_misalignment) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: temperature_slope
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.temperature_slope.size() == 0) {
      out << "temperature_slope: []\n";
    } else {
      out << "temperature_slope:\n";
      for (auto item : msg.temperature_slope) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IMUInfo & msg, bool use_flow_style = false)
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
  const orbbec_camera_msgs::msg::IMUInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  orbbec_camera_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orbbec_camera_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const orbbec_camera_msgs::msg::IMUInfo & msg)
{
  return orbbec_camera_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<orbbec_camera_msgs::msg::IMUInfo>()
{
  return "orbbec_camera_msgs::msg::IMUInfo";
}

template<>
inline const char * name<orbbec_camera_msgs::msg::IMUInfo>()
{
  return "orbbec_camera_msgs/msg/IMUInfo";
}

template<>
struct has_fixed_size<orbbec_camera_msgs::msg::IMUInfo>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<orbbec_camera_msgs::msg::IMUInfo>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<orbbec_camera_msgs::msg::IMUInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__TRAITS_HPP_
