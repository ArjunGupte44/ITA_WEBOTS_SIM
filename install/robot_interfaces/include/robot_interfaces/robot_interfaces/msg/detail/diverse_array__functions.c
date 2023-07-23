// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/DiverseArray.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/diverse_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `poi_coords`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
robot_interfaces__msg__DiverseArray__init(robot_interfaces__msg__DiverseArray * msg)
{
  if (!msg) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__init(&msg->robot_name)) {
    robot_interfaces__msg__DiverseArray__fini(msg);
    return false;
  }
  // poi_coords
  if (!rosidl_runtime_c__double__Sequence__init(&msg->poi_coords, 0)) {
    robot_interfaces__msg__DiverseArray__fini(msg);
    return false;
  }
  // arrival_time
  return true;
}

void
robot_interfaces__msg__DiverseArray__fini(robot_interfaces__msg__DiverseArray * msg)
{
  if (!msg) {
    return;
  }
  // robot_name
  rosidl_runtime_c__String__fini(&msg->robot_name);
  // poi_coords
  rosidl_runtime_c__double__Sequence__fini(&msg->poi_coords);
  // arrival_time
}

bool
robot_interfaces__msg__DiverseArray__are_equal(const robot_interfaces__msg__DiverseArray * lhs, const robot_interfaces__msg__DiverseArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_name), &(rhs->robot_name)))
  {
    return false;
  }
  // poi_coords
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->poi_coords), &(rhs->poi_coords)))
  {
    return false;
  }
  // arrival_time
  if (lhs->arrival_time != rhs->arrival_time) {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__DiverseArray__copy(
  const robot_interfaces__msg__DiverseArray * input,
  robot_interfaces__msg__DiverseArray * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_name), &(output->robot_name)))
  {
    return false;
  }
  // poi_coords
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->poi_coords), &(output->poi_coords)))
  {
    return false;
  }
  // arrival_time
  output->arrival_time = input->arrival_time;
  return true;
}

robot_interfaces__msg__DiverseArray *
robot_interfaces__msg__DiverseArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__DiverseArray * msg = (robot_interfaces__msg__DiverseArray *)allocator.allocate(sizeof(robot_interfaces__msg__DiverseArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__DiverseArray));
  bool success = robot_interfaces__msg__DiverseArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__DiverseArray__destroy(robot_interfaces__msg__DiverseArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__DiverseArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__DiverseArray__Sequence__init(robot_interfaces__msg__DiverseArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__DiverseArray * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__DiverseArray *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__DiverseArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__DiverseArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__DiverseArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_interfaces__msg__DiverseArray__Sequence__fini(robot_interfaces__msg__DiverseArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_interfaces__msg__DiverseArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_interfaces__msg__DiverseArray__Sequence *
robot_interfaces__msg__DiverseArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__DiverseArray__Sequence * array = (robot_interfaces__msg__DiverseArray__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__DiverseArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__DiverseArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__DiverseArray__Sequence__destroy(robot_interfaces__msg__DiverseArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__DiverseArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__DiverseArray__Sequence__are_equal(const robot_interfaces__msg__DiverseArray__Sequence * lhs, const robot_interfaces__msg__DiverseArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__DiverseArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__DiverseArray__Sequence__copy(
  const robot_interfaces__msg__DiverseArray__Sequence * input,
  robot_interfaces__msg__DiverseArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__DiverseArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__DiverseArray * data =
      (robot_interfaces__msg__DiverseArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__DiverseArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__DiverseArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__DiverseArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
