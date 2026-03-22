// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orbbec_camera_msgs:msg/Extrinsics.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__MSG__DETAIL__EXTRINSICS__STRUCT_HPP_
#define ORBBEC_CAMERA_MSGS__MSG__DETAIL__EXTRINSICS__STRUCT_HPP_

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
# define DEPRECATED__orbbec_camera_msgs__msg__Extrinsics __attribute__((deprecated))
#else
# define DEPRECATED__orbbec_camera_msgs__msg__Extrinsics __declspec(deprecated)
#endif

namespace orbbec_camera_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Extrinsics_
{
  using Type = Extrinsics_<ContainerAllocator>;

  explicit Extrinsics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->rotation.begin(), this->rotation.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->translation.begin(), this->translation.end(), 0.0);
    }
  }

  explicit Extrinsics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    rotation(_alloc),
    translation(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->rotation.begin(), this->rotation.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->translation.begin(), this->translation.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rotation_type =
    std::array<double, 9>;
  _rotation_type rotation;
  using _translation_type =
    std::array<double, 3>;
  _translation_type translation;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rotation(
    const std::array<double, 9> & _arg)
  {
    this->rotation = _arg;
    return *this;
  }
  Type & set__translation(
    const std::array<double, 3> & _arg)
  {
    this->translation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator> *;
  using ConstRawPtr =
    const orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orbbec_camera_msgs__msg__Extrinsics
    std::shared_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orbbec_camera_msgs__msg__Extrinsics
    std::shared_ptr<orbbec_camera_msgs::msg::Extrinsics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Extrinsics_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rotation != other.rotation) {
      return false;
    }
    if (this->translation != other.translation) {
      return false;
    }
    return true;
  }
  bool operator!=(const Extrinsics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Extrinsics_

// alias to use template instance with default allocator
using Extrinsics =
  orbbec_camera_msgs::msg::Extrinsics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__MSG__DETAIL__EXTRINSICS__STRUCT_HPP_
