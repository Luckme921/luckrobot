// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from orbbec_camera_msgs:srv/GetDeviceInfo.idl
// generated code does not contain a copyright notice

#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__FUNCTIONS_H_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "orbbec_camera_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "orbbec_camera_msgs/srv/detail/get_device_info__struct.h"

/// Initialize srv/GetDeviceInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request
 * )) before or use
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Request__init(orbbec_camera_msgs__srv__GetDeviceInfo_Request * msg);

/// Finalize srv/GetDeviceInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Request__fini(orbbec_camera_msgs__srv__GetDeviceInfo_Request * msg);

/// Create srv/GetDeviceInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
orbbec_camera_msgs__srv__GetDeviceInfo_Request *
orbbec_camera_msgs__srv__GetDeviceInfo_Request__create();

/// Destroy srv/GetDeviceInfo message.
/**
 * It calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Request__destroy(orbbec_camera_msgs__srv__GetDeviceInfo_Request * msg);

/// Check for srv/GetDeviceInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Request__are_equal(const orbbec_camera_msgs__srv__GetDeviceInfo_Request * lhs, const orbbec_camera_msgs__srv__GetDeviceInfo_Request * rhs);

/// Copy a srv/GetDeviceInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Request__copy(
  const orbbec_camera_msgs__srv__GetDeviceInfo_Request * input,
  orbbec_camera_msgs__srv__GetDeviceInfo_Request * output);

/// Initialize array of srv/GetDeviceInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__init(orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetDeviceInfo messages.
/**
 * It calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__fini(orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * array);

/// Create array of srv/GetDeviceInfo messages.
/**
 * It allocates the memory for the array and calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence *
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetDeviceInfo messages.
/**
 * It calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__destroy(orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * array);

/// Check for srv/GetDeviceInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__are_equal(const orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * lhs, const orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * rhs);

/// Copy an array of srv/GetDeviceInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence__copy(
  const orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * input,
  orbbec_camera_msgs__srv__GetDeviceInfo_Request__Sequence * output);

/// Initialize srv/GetDeviceInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response
 * )) before or use
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Response__init(orbbec_camera_msgs__srv__GetDeviceInfo_Response * msg);

/// Finalize srv/GetDeviceInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Response__fini(orbbec_camera_msgs__srv__GetDeviceInfo_Response * msg);

/// Create srv/GetDeviceInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
orbbec_camera_msgs__srv__GetDeviceInfo_Response *
orbbec_camera_msgs__srv__GetDeviceInfo_Response__create();

/// Destroy srv/GetDeviceInfo message.
/**
 * It calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Response__destroy(orbbec_camera_msgs__srv__GetDeviceInfo_Response * msg);

/// Check for srv/GetDeviceInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Response__are_equal(const orbbec_camera_msgs__srv__GetDeviceInfo_Response * lhs, const orbbec_camera_msgs__srv__GetDeviceInfo_Response * rhs);

/// Copy a srv/GetDeviceInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Response__copy(
  const orbbec_camera_msgs__srv__GetDeviceInfo_Response * input,
  orbbec_camera_msgs__srv__GetDeviceInfo_Response * output);

/// Initialize array of srv/GetDeviceInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__init(orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetDeviceInfo messages.
/**
 * It calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__fini(orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * array);

/// Create array of srv/GetDeviceInfo messages.
/**
 * It allocates the memory for the array and calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence *
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetDeviceInfo messages.
/**
 * It calls
 * orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
void
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__destroy(orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * array);

/// Check for srv/GetDeviceInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__are_equal(const orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * lhs, const orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * rhs);

/// Copy an array of srv/GetDeviceInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_orbbec_camera_msgs
bool
orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence__copy(
  const orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * input,
  orbbec_camera_msgs__srv__GetDeviceInfo_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__FUNCTIONS_H_
