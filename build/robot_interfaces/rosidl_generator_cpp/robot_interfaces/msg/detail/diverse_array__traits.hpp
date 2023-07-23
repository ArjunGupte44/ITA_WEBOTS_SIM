// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/DiverseArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/diverse_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DiverseArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_name
  {
    out << "robot_name: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_name, out);
    out << ", ";
  }

  // member: poi_coords
  {
    if (msg.poi_coords.size() == 0) {
      out << "poi_coords: []";
    } else {
      out << "poi_coords: [";
      size_t pending_items = msg.poi_coords.size();
      for (auto item : msg.poi_coords) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: arrival_time
  {
    out << "arrival_time: ";
    rosidl_generator_traits::value_to_yaml(msg.arrival_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DiverseArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_name: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_name, out);
    out << "\n";
  }

  // member: poi_coords
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poi_coords.size() == 0) {
      out << "poi_coords: []\n";
    } else {
      out << "poi_coords:\n";
      for (auto item : msg.poi_coords) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: arrival_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arrival_time: ";
    rosidl_generator_traits::value_to_yaml(msg.arrival_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DiverseArray & msg, bool use_flow_style = false)
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

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::DiverseArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::DiverseArray & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::DiverseArray>()
{
  return "robot_interfaces::msg::DiverseArray";
}

template<>
inline const char * name<robot_interfaces::msg::DiverseArray>()
{
  return "robot_interfaces/msg/DiverseArray";
}

template<>
struct has_fixed_size<robot_interfaces::msg::DiverseArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::msg::DiverseArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::msg::DiverseArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__TRAITS_HPP_
