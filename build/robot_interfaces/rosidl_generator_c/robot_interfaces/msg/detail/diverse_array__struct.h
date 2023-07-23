// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/DiverseArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DiverseArray in the package robot_interfaces.
typedef struct robot_interfaces__msg__DiverseArray
{
  rosidl_runtime_c__String robot_name;
  double poi_x;
  double poi_y;
  double poi_z;
  double arrival_time;
} robot_interfaces__msg__DiverseArray;

// Struct for a sequence of robot_interfaces__msg__DiverseArray.
typedef struct robot_interfaces__msg__DiverseArray__Sequence
{
  robot_interfaces__msg__DiverseArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__DiverseArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__DIVERSE_ARRAY__STRUCT_H_
