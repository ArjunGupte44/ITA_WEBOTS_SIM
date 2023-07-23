// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_interfaces:msg/DiverseArray.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_interfaces/msg/detail/diverse_array__struct.h"
#include "robot_interfaces/msg/detail/diverse_array__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_interfaces__msg__diverse_array__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robot_interfaces.msg._diverse_array.DiverseArray", full_classname_dest, 48) == 0);
  }
  robot_interfaces__msg__DiverseArray * ros_message = _ros_message;
  {  // robot_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->robot_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // poi_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "poi_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->poi_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // poi_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "poi_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->poi_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // poi_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "poi_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->poi_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // arrival_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "arrival_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->arrival_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_interfaces__msg__diverse_array__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DiverseArray */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_interfaces.msg._diverse_array");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DiverseArray");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_interfaces__msg__DiverseArray * ros_message = (robot_interfaces__msg__DiverseArray *)raw_ros_message;
  {  // robot_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->robot_name.data,
      strlen(ros_message->robot_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // poi_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->poi_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "poi_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // poi_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->poi_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "poi_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // poi_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->poi_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "poi_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arrival_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->arrival_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arrival_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
