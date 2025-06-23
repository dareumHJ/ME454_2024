// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from midterm_2024_msg:msg/Midterm.idl
// generated code does not contain a copyright notice

#ifndef MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__FUNCTIONS_H_
#define MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "midterm_2024_msg/msg/rosidl_generator_c__visibility_control.h"

#include "midterm_2024_msg/msg/detail/midterm__struct.h"

/// Initialize msg/Midterm message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * midterm_2024_msg__msg__Midterm
 * )) before or use
 * midterm_2024_msg__msg__Midterm__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
bool
midterm_2024_msg__msg__Midterm__init(midterm_2024_msg__msg__Midterm * msg);

/// Finalize msg/Midterm message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
void
midterm_2024_msg__msg__Midterm__fini(midterm_2024_msg__msg__Midterm * msg);

/// Create msg/Midterm message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * midterm_2024_msg__msg__Midterm__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
midterm_2024_msg__msg__Midterm *
midterm_2024_msg__msg__Midterm__create();

/// Destroy msg/Midterm message.
/**
 * It calls
 * midterm_2024_msg__msg__Midterm__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
void
midterm_2024_msg__msg__Midterm__destroy(midterm_2024_msg__msg__Midterm * msg);

/// Check for msg/Midterm message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
bool
midterm_2024_msg__msg__Midterm__are_equal(const midterm_2024_msg__msg__Midterm * lhs, const midterm_2024_msg__msg__Midterm * rhs);

/// Copy a msg/Midterm message.
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
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
bool
midterm_2024_msg__msg__Midterm__copy(
  const midterm_2024_msg__msg__Midterm * input,
  midterm_2024_msg__msg__Midterm * output);

/// Initialize array of msg/Midterm messages.
/**
 * It allocates the memory for the number of elements and calls
 * midterm_2024_msg__msg__Midterm__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
bool
midterm_2024_msg__msg__Midterm__Sequence__init(midterm_2024_msg__msg__Midterm__Sequence * array, size_t size);

/// Finalize array of msg/Midterm messages.
/**
 * It calls
 * midterm_2024_msg__msg__Midterm__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
void
midterm_2024_msg__msg__Midterm__Sequence__fini(midterm_2024_msg__msg__Midterm__Sequence * array);

/// Create array of msg/Midterm messages.
/**
 * It allocates the memory for the array and calls
 * midterm_2024_msg__msg__Midterm__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
midterm_2024_msg__msg__Midterm__Sequence *
midterm_2024_msg__msg__Midterm__Sequence__create(size_t size);

/// Destroy array of msg/Midterm messages.
/**
 * It calls
 * midterm_2024_msg__msg__Midterm__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
void
midterm_2024_msg__msg__Midterm__Sequence__destroy(midterm_2024_msg__msg__Midterm__Sequence * array);

/// Check for msg/Midterm message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
bool
midterm_2024_msg__msg__Midterm__Sequence__are_equal(const midterm_2024_msg__msg__Midterm__Sequence * lhs, const midterm_2024_msg__msg__Midterm__Sequence * rhs);

/// Copy an array of msg/Midterm messages.
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
ROSIDL_GENERATOR_C_PUBLIC_midterm_2024_msg
bool
midterm_2024_msg__msg__Midterm__Sequence__copy(
  const midterm_2024_msg__msg__Midterm__Sequence * input,
  midterm_2024_msg__msg__Midterm__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MIDTERM_2024_MSG__MSG__DETAIL__MIDTERM__FUNCTIONS_H_
