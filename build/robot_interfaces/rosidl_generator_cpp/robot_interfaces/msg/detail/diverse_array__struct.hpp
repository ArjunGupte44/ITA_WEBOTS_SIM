// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/DiverseArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__DiverseArray __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__DiverseArray __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DiverseArray_
{
  using Type = DiverseArray_<ContainerAllocator>;

  explicit DiverseArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
      this->arrival_time = 0.0;
    }
  }

  explicit DiverseArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
      this->arrival_time = 0.0;
    }
  }

  // field types and members
  using _robot_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_name_type robot_name;
  using _poi_coords_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _poi_coords_type poi_coords;
  using _arrival_time_type =
    double;
  _arrival_time_type arrival_time;

  // setters for named parameter idiom
  Type & set__robot_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_name = _arg;
    return *this;
  }
  Type & set__poi_coords(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->poi_coords = _arg;
    return *this;
  }
  Type & set__arrival_time(
    const double & _arg)
  {
    this->arrival_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::DiverseArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::DiverseArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::DiverseArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::DiverseArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__DiverseArray
    std::shared_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__DiverseArray
    std::shared_ptr<robot_interfaces::msg::DiverseArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DiverseArray_ & other) const
  {
    if (this->robot_name != other.robot_name) {
      return false;
    }
    if (this->poi_coords != other.poi_coords) {
      return false;
    }
    if (this->arrival_time != other.arrival_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const DiverseArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DiverseArray_

// alias to use template instance with default allocator
using DiverseArray =
  robot_interfaces::msg::DiverseArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__STRUCT_HPP_
