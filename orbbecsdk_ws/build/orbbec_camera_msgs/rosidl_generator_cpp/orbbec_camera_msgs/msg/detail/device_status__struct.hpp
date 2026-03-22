// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__STRUCT_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__orbbec_camera_msgs__msg__DeviceStatus __attribute__((deprecated))
#else
# define DEPRECATED__orbbec_camera_msgs__msg__DeviceStatus __declspec(deprecated)
#endif

namespace orbbec_camera_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DeviceStatus_
{
  using Type = DeviceStatus_<ContainerAllocator>;

  explicit DeviceStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color_frame_rate_cur = 0.0;
      this->color_frame_rate_avg = 0.0;
      this->color_frame_rate_min = 0.0;
      this->color_frame_rate_max = 0.0;
      this->color_delay_ms_cur = 0.0;
      this->color_delay_ms_avg = 0.0;
      this->color_delay_ms_min = 0.0;
      this->color_delay_ms_max = 0.0;
      this->depth_frame_rate_cur = 0.0;
      this->depth_frame_rate_avg = 0.0;
      this->depth_frame_rate_min = 0.0;
      this->depth_frame_rate_max = 0.0;
      this->depth_delay_ms_cur = 0.0;
      this->depth_delay_ms_avg = 0.0;
      this->depth_delay_ms_min = 0.0;
      this->depth_delay_ms_max = 0.0;
      this->device_online = false;
      this->connection_type = "";
      this->customer_calibration_ready = false;
      this->calibration_from_factory = false;
      this->calibration_from_launch_param = false;
    }
  }

  explicit DeviceStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    connection_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color_frame_rate_cur = 0.0;
      this->color_frame_rate_avg = 0.0;
      this->color_frame_rate_min = 0.0;
      this->color_frame_rate_max = 0.0;
      this->color_delay_ms_cur = 0.0;
      this->color_delay_ms_avg = 0.0;
      this->color_delay_ms_min = 0.0;
      this->color_delay_ms_max = 0.0;
      this->depth_frame_rate_cur = 0.0;
      this->depth_frame_rate_avg = 0.0;
      this->depth_frame_rate_min = 0.0;
      this->depth_frame_rate_max = 0.0;
      this->depth_delay_ms_cur = 0.0;
      this->depth_delay_ms_avg = 0.0;
      this->depth_delay_ms_min = 0.0;
      this->depth_delay_ms_max = 0.0;
      this->device_online = false;
      this->connection_type = "";
      this->customer_calibration_ready = false;
      this->calibration_from_factory = false;
      this->calibration_from_launch_param = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _color_frame_rate_cur_type =
    double;
  _color_frame_rate_cur_type color_frame_rate_cur;
  using _color_frame_rate_avg_type =
    double;
  _color_frame_rate_avg_type color_frame_rate_avg;
  using _color_frame_rate_min_type =
    double;
  _color_frame_rate_min_type color_frame_rate_min;
  using _color_frame_rate_max_type =
    double;
  _color_frame_rate_max_type color_frame_rate_max;
  using _color_delay_ms_cur_type =
    double;
  _color_delay_ms_cur_type color_delay_ms_cur;
  using _color_delay_ms_avg_type =
    double;
  _color_delay_ms_avg_type color_delay_ms_avg;
  using _color_delay_ms_min_type =
    double;
  _color_delay_ms_min_type color_delay_ms_min;
  using _color_delay_ms_max_type =
    double;
  _color_delay_ms_max_type color_delay_ms_max;
  using _depth_frame_rate_cur_type =
    double;
  _depth_frame_rate_cur_type depth_frame_rate_cur;
  using _depth_frame_rate_avg_type =
    double;
  _depth_frame_rate_avg_type depth_frame_rate_avg;
  using _depth_frame_rate_min_type =
    double;
  _depth_frame_rate_min_type depth_frame_rate_min;
  using _depth_frame_rate_max_type =
    double;
  _depth_frame_rate_max_type depth_frame_rate_max;
  using _depth_delay_ms_cur_type =
    double;
  _depth_delay_ms_cur_type depth_delay_ms_cur;
  using _depth_delay_ms_avg_type =
    double;
  _depth_delay_ms_avg_type depth_delay_ms_avg;
  using _depth_delay_ms_min_type =
    double;
  _depth_delay_ms_min_type depth_delay_ms_min;
  using _depth_delay_ms_max_type =
    double;
  _depth_delay_ms_max_type depth_delay_ms_max;
  using _device_online_type =
    bool;
  _device_online_type device_online;
  using _connection_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _connection_type_type connection_type;
  using _customer_calibration_ready_type =
    bool;
  _customer_calibration_ready_type customer_calibration_ready;
  using _calibration_from_factory_type =
    bool;
  _calibration_from_factory_type calibration_from_factory;
  using _calibration_from_launch_param_type =
    bool;
  _calibration_from_launch_param_type calibration_from_launch_param;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__color_frame_rate_cur(
    const double & _arg)
  {
    this->color_frame_rate_cur = _arg;
    return *this;
  }
  Type & set__color_frame_rate_avg(
    const double & _arg)
  {
    this->color_frame_rate_avg = _arg;
    return *this;
  }
  Type & set__color_frame_rate_min(
    const double & _arg)
  {
    this->color_frame_rate_min = _arg;
    return *this;
  }
  Type & set__color_frame_rate_max(
    const double & _arg)
  {
    this->color_frame_rate_max = _arg;
    return *this;
  }
  Type & set__color_delay_ms_cur(
    const double & _arg)
  {
    this->color_delay_ms_cur = _arg;
    return *this;
  }
  Type & set__color_delay_ms_avg(
    const double & _arg)
  {
    this->color_delay_ms_avg = _arg;
    return *this;
  }
  Type & set__color_delay_ms_min(
    const double & _arg)
  {
    this->color_delay_ms_min = _arg;
    return *this;
  }
  Type & set__color_delay_ms_max(
    const double & _arg)
  {
    this->color_delay_ms_max = _arg;
    return *this;
  }
  Type & set__depth_frame_rate_cur(
    const double & _arg)
  {
    this->depth_frame_rate_cur = _arg;
    return *this;
  }
  Type & set__depth_frame_rate_avg(
    const double & _arg)
  {
    this->depth_frame_rate_avg = _arg;
    return *this;
  }
  Type & set__depth_frame_rate_min(
    const double & _arg)
  {
    this->depth_frame_rate_min = _arg;
    return *this;
  }
  Type & set__depth_frame_rate_max(
    const double & _arg)
  {
    this->depth_frame_rate_max = _arg;
    return *this;
  }
  Type & set__depth_delay_ms_cur(
    const double & _arg)
  {
    this->depth_delay_ms_cur = _arg;
    return *this;
  }
  Type & set__depth_delay_ms_avg(
    const double & _arg)
  {
    this->depth_delay_ms_avg = _arg;
    return *this;
  }
  Type & set__depth_delay_ms_min(
    const double & _arg)
  {
    this->depth_delay_ms_min = _arg;
    return *this;
  }
  Type & set__depth_delay_ms_max(
    const double & _arg)
  {
    this->depth_delay_ms_max = _arg;
    return *this;
  }
  Type & set__device_online(
    const bool & _arg)
  {
    this->device_online = _arg;
    return *this;
  }
  Type & set__connection_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->connection_type = _arg;
    return *this;
  }
  Type & set__customer_calibration_ready(
    const bool & _arg)
  {
    this->customer_calibration_ready = _arg;
    return *this;
  }
  Type & set__calibration_from_factory(
    const bool & _arg)
  {
    this->calibration_from_factory = _arg;
    return *this;
  }
  Type & set__calibration_from_launch_param(
    const bool & _arg)
  {
    this->calibration_from_launch_param = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orbbec_camera_msgs__msg__DeviceStatus
    std::shared_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orbbec_camera_msgs__msg__DeviceStatus
    std::shared_ptr<orbbec_camera_msgs::msg::DeviceStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DeviceStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->color_frame_rate_cur != other.color_frame_rate_cur) {
      return false;
    }
    if (this->color_frame_rate_avg != other.color_frame_rate_avg) {
      return false;
    }
    if (this->color_frame_rate_min != other.color_frame_rate_min) {
      return false;
    }
    if (this->color_frame_rate_max != other.color_frame_rate_max) {
      return false;
    }
    if (this->color_delay_ms_cur != other.color_delay_ms_cur) {
      return false;
    }
    if (this->color_delay_ms_avg != other.color_delay_ms_avg) {
      return false;
    }
    if (this->color_delay_ms_min != other.color_delay_ms_min) {
      return false;
    }
    if (this->color_delay_ms_max != other.color_delay_ms_max) {
      return false;
    }
    if (this->depth_frame_rate_cur != other.depth_frame_rate_cur) {
      return false;
    }
    if (this->depth_frame_rate_avg != other.depth_frame_rate_avg) {
      return false;
    }
    if (this->depth_frame_rate_min != other.depth_frame_rate_min) {
      return false;
    }
    if (this->depth_frame_rate_max != other.depth_frame_rate_max) {
      return false;
    }
    if (this->depth_delay_ms_cur != other.depth_delay_ms_cur) {
      return false;
    }
    if (this->depth_delay_ms_avg != other.depth_delay_ms_avg) {
      return false;
    }
    if (this->depth_delay_ms_min != other.depth_delay_ms_min) {
      return false;
    }
    if (this->depth_delay_ms_max != other.depth_delay_ms_max) {
      return false;
    }
    if (this->device_online != other.device_online) {
      return false;
    }
    if (this->connection_type != other.connection_type) {
      return false;
    }
    if (this->customer_calibration_ready != other.customer_calibration_ready) {
      return false;
    }
    if (this->calibration_from_factory != other.calibration_from_factory) {
      return false;
    }
    if (this->calibration_from_launch_param != other.calibration_from_launch_param) {
      return false;
    }
    return true;
  }
  bool operator!=(const DeviceStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DeviceStatus_

// alias to use template instance with default allocator
using DeviceStatus =
  orbbec_camera_msgs::msg::DeviceStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__DEVICE_STATUS__STRUCT_HPP_
