// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from orbbec_camera_msgs:msg/DeviceStatus.idl
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
#include "orbbec_camera_msgs/msg/detail/device_status__struct.h"
#include "orbbec_camera_msgs/msg/detail/device_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool orbbec_camera_msgs__msg__device_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("orbbec_camera_msgs.msg._device_status.DeviceStatus", full_classname_dest, 50) == 0);
  }
  orbbec_camera_msgs__msg__DeviceStatus * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // color_frame_rate_cur
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_frame_rate_cur");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_frame_rate_cur = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_frame_rate_avg
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_frame_rate_avg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_frame_rate_avg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_frame_rate_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_frame_rate_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_frame_rate_min = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_frame_rate_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_frame_rate_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_frame_rate_max = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_delay_ms_cur
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_delay_ms_cur");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_delay_ms_cur = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_delay_ms_avg
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_delay_ms_avg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_delay_ms_avg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_delay_ms_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_delay_ms_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_delay_ms_min = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // color_delay_ms_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "color_delay_ms_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->color_delay_ms_max = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_frame_rate_cur
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_frame_rate_cur");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_frame_rate_cur = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_frame_rate_avg
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_frame_rate_avg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_frame_rate_avg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_frame_rate_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_frame_rate_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_frame_rate_min = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_frame_rate_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_frame_rate_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_frame_rate_max = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_delay_ms_cur
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_delay_ms_cur");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_delay_ms_cur = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_delay_ms_avg
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_delay_ms_avg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_delay_ms_avg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_delay_ms_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_delay_ms_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_delay_ms_min = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_delay_ms_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_delay_ms_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_delay_ms_max = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // device_online
    PyObject * field = PyObject_GetAttrString(_pymsg, "device_online");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->device_online = (Py_True == field);
    Py_DECREF(field);
  }
  {  // connection_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "connection_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->connection_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // customer_calibration_ready
    PyObject * field = PyObject_GetAttrString(_pymsg, "customer_calibration_ready");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->customer_calibration_ready = (Py_True == field);
    Py_DECREF(field);
  }
  {  // calibration_from_factory
    PyObject * field = PyObject_GetAttrString(_pymsg, "calibration_from_factory");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->calibration_from_factory = (Py_True == field);
    Py_DECREF(field);
  }
  {  // calibration_from_launch_param
    PyObject * field = PyObject_GetAttrString(_pymsg, "calibration_from_launch_param");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->calibration_from_launch_param = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * orbbec_camera_msgs__msg__device_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DeviceStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("orbbec_camera_msgs.msg._device_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DeviceStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  orbbec_camera_msgs__msg__DeviceStatus * ros_message = (orbbec_camera_msgs__msg__DeviceStatus *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_frame_rate_cur
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_frame_rate_cur);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_frame_rate_cur", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_frame_rate_avg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_frame_rate_avg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_frame_rate_avg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_frame_rate_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_frame_rate_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_frame_rate_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_frame_rate_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_frame_rate_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_frame_rate_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_delay_ms_cur
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_delay_ms_cur);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_delay_ms_cur", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_delay_ms_avg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_delay_ms_avg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_delay_ms_avg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_delay_ms_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_delay_ms_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_delay_ms_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color_delay_ms_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->color_delay_ms_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "color_delay_ms_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_frame_rate_cur
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_frame_rate_cur);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_frame_rate_cur", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_frame_rate_avg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_frame_rate_avg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_frame_rate_avg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_frame_rate_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_frame_rate_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_frame_rate_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_frame_rate_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_frame_rate_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_frame_rate_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_delay_ms_cur
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_delay_ms_cur);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_delay_ms_cur", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_delay_ms_avg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_delay_ms_avg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_delay_ms_avg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_delay_ms_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_delay_ms_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_delay_ms_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_delay_ms_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_delay_ms_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_delay_ms_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // device_online
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->device_online ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "device_online", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // connection_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->connection_type.data,
      strlen(ros_message->connection_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "connection_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // customer_calibration_ready
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->customer_calibration_ready ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "customer_calibration_ready", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calibration_from_factory
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->calibration_from_factory ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calibration_from_factory", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // calibration_from_launch_param
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->calibration_from_launch_param ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "calibration_from_launch_param", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
