// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/DiverseArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/diverse_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_DiverseArray_arrival_time
{
public:
  explicit Init_DiverseArray_arrival_time(::robot_interfaces::msg::DiverseArray & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::DiverseArray arrival_time(::robot_interfaces::msg::DiverseArray::_arrival_time_type arg)
  {
    msg_.arrival_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::DiverseArray msg_;
};

class Init_DiverseArray_poi_z
{
public:
  explicit Init_DiverseArray_poi_z(::robot_interfaces::msg::DiverseArray & msg)
  : msg_(msg)
  {}
  Init_DiverseArray_arrival_time poi_z(::robot_interfaces::msg::DiverseArray::_poi_z_type arg)
  {
    msg_.poi_z = std::move(arg);
    return Init_DiverseArray_arrival_time(msg_);
  }

private:
  ::robot_interfaces::msg::DiverseArray msg_;
};

class Init_DiverseArray_poi_y
{
public:
  explicit Init_DiverseArray_poi_y(::robot_interfaces::msg::DiverseArray & msg)
  : msg_(msg)
  {}
  Init_DiverseArray_poi_z poi_y(::robot_interfaces::msg::DiverseArray::_poi_y_type arg)
  {
    msg_.poi_y = std::move(arg);
    return Init_DiverseArray_poi_z(msg_);
  }

private:
  ::robot_interfaces::msg::DiverseArray msg_;
};

class Init_DiverseArray_poi_x
{
public:
  explicit Init_DiverseArray_poi_x(::robot_interfaces::msg::DiverseArray & msg)
  : msg_(msg)
  {}
  Init_DiverseArray_poi_y poi_x(::robot_interfaces::msg::DiverseArray::_poi_x_type arg)
  {
    msg_.poi_x = std::move(arg);
    return Init_DiverseArray_poi_y(msg_);
  }

private:
  ::robot_interfaces::msg::DiverseArray msg_;
};

class Init_DiverseArray_robot_name
{
public:
  Init_DiverseArray_robot_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DiverseArray_poi_x robot_name(::robot_interfaces::msg::DiverseArray::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return Init_DiverseArray_poi_x(msg_);
  }

private:
  ::robot_interfaces::msg::DiverseArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::DiverseArray>()
{
  return robot_interfaces::msg::builder::Init_DiverseArray_robot_name();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__BUILDER_HPP_
