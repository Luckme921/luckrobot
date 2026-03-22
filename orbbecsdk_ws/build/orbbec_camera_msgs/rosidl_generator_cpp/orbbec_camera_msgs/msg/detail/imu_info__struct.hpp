// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orbbec_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__STRUCT_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__STRUCT_HPP_

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
# define DEPRECATED__orbbec_camera_msgs__msg__IMUInfo __attribute__((deprecated))
#else
# define DEPRECATED__orbbec_camera_msgs__msg__IMUInfo __declspec(deprecated)
#endif

namespace orbbec_camera_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IMUInfo_
{
  using Type = IMUInfo_<ContainerAllocator>;

  explicit IMUInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->noise_density = 0.0;
      this->random_walk = 0.0;
      this->reference_temperature = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->bias.begin(), this->bias.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->gravity.begin(), this->gravity.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->scale_misalignment.begin(), this->scale_misalignment.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->temperature_slope.begin(), this->temperature_slope.end(), 0.0);
    }
  }

  explicit IMUInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    bias(_alloc),
    gravity(_alloc),
    scale_misalignment(_alloc),
    temperature_slope(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->noise_density = 0.0;
      this->random_walk = 0.0;
      this->reference_temperature = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->bias.begin(), this->bias.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->gravity.begin(), this->gravity.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->scale_misalignment.begin(), this->scale_misalignment.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->temperature_slope.begin(), this->temperature_slope.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _noise_density_type =
    double;
  _noise_density_type noise_density;
  using _random_walk_type =
    double;
  _random_walk_type random_walk;
  using _reference_temperature_type =
    double;
  _reference_temperature_type reference_temperature;
  using _bias_type =
    std::array<double, 3>;
  _bias_type bias;
  using _gravity_type =
    std::array<double, 3>;
  _gravity_type gravity;
  using _scale_misalignment_type =
    std::array<double, 9>;
  _scale_misalignment_type scale_misalignment;
  using _temperature_slope_type =
    std::array<double, 9>;
  _temperature_slope_type temperature_slope;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__noise_density(
    const double & _arg)
  {
    this->noise_density = _arg;
    return *this;
  }
  Type & set__random_walk(
    const double & _arg)
  {
    this->random_walk = _arg;
    return *this;
  }
  Type & set__reference_temperature(
    const double & _arg)
  {
    this->reference_temperature = _arg;
    return *this;
  }
  Type & set__bias(
    const std::array<double, 3> & _arg)
  {
    this->bias = _arg;
    return *this;
  }
  Type & set__gravity(
    const std::array<double, 3> & _arg)
  {
    this->gravity = _arg;
    return *this;
  }
  Type & set__scale_misalignment(
    const std::array<double, 9> & _arg)
  {
    this->scale_misalignment = _arg;
    return *this;
  }
  Type & set__temperature_slope(
    const std::array<double, 9> & _arg)
  {
    this->temperature_slope = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orbbec_camera_msgs__msg__IMUInfo
    std::shared_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orbbec_camera_msgs__msg__IMUInfo
    std::shared_ptr<orbbec_camera_msgs::msg::IMUInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IMUInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->noise_density != other.noise_density) {
      return false;
    }
    if (this->random_walk != other.random_walk) {
      return false;
    }
    if (this->reference_temperature != other.reference_temperature) {
      return false;
    }
    if (this->bias != other.bias) {
      return false;
    }
    if (this->gravity != other.gravity) {
      return false;
    }
    if (this->scale_misalignment != other.scale_misalignment) {
      return false;
    }
    if (this->temperature_slope != other.temperature_slope) {
      return false;
    }
    return true;
  }
  bool operator!=(const IMUInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IMUInfo_

// alias to use template instance with default allocator
using IMUInfo =
  orbbec_camera_msgs::msg::IMUInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__STRUCT_HPP_
