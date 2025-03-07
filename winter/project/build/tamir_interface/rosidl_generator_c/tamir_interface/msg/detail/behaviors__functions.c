// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice
#include "tamir_interface/msg/detail/behaviors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
tamir_interface__msg__Behaviors__init(tamir_interface__msg__Behaviors * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    tamir_interface__msg__Behaviors__fini(msg);
    return false;
  }
  // state
  return true;
}

void
tamir_interface__msg__Behaviors__fini(tamir_interface__msg__Behaviors * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // state
}

bool
tamir_interface__msg__Behaviors__are_equal(const tamir_interface__msg__Behaviors * lhs, const tamir_interface__msg__Behaviors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
tamir_interface__msg__Behaviors__copy(
  const tamir_interface__msg__Behaviors * input,
  tamir_interface__msg__Behaviors * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // state
  output->state = input->state;
  return true;
}

tamir_interface__msg__Behaviors *
tamir_interface__msg__Behaviors__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__msg__Behaviors * msg = (tamir_interface__msg__Behaviors *)allocator.allocate(sizeof(tamir_interface__msg__Behaviors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tamir_interface__msg__Behaviors));
  bool success = tamir_interface__msg__Behaviors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tamir_interface__msg__Behaviors__destroy(tamir_interface__msg__Behaviors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tamir_interface__msg__Behaviors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tamir_interface__msg__Behaviors__Sequence__init(tamir_interface__msg__Behaviors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__msg__Behaviors * data = NULL;

  if (size) {
    data = (tamir_interface__msg__Behaviors *)allocator.zero_allocate(size, sizeof(tamir_interface__msg__Behaviors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tamir_interface__msg__Behaviors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tamir_interface__msg__Behaviors__fini(&data[i - 1]);
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
tamir_interface__msg__Behaviors__Sequence__fini(tamir_interface__msg__Behaviors__Sequence * array)
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
      tamir_interface__msg__Behaviors__fini(&array->data[i]);
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

tamir_interface__msg__Behaviors__Sequence *
tamir_interface__msg__Behaviors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tamir_interface__msg__Behaviors__Sequence * array = (tamir_interface__msg__Behaviors__Sequence *)allocator.allocate(sizeof(tamir_interface__msg__Behaviors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tamir_interface__msg__Behaviors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tamir_interface__msg__Behaviors__Sequence__destroy(tamir_interface__msg__Behaviors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tamir_interface__msg__Behaviors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tamir_interface__msg__Behaviors__Sequence__are_equal(const tamir_interface__msg__Behaviors__Sequence * lhs, const tamir_interface__msg__Behaviors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tamir_interface__msg__Behaviors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tamir_interface__msg__Behaviors__Sequence__copy(
  const tamir_interface__msg__Behaviors__Sequence * input,
  tamir_interface__msg__Behaviors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tamir_interface__msg__Behaviors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tamir_interface__msg__Behaviors * data =
      (tamir_interface__msg__Behaviors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tamir_interface__msg__Behaviors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tamir_interface__msg__Behaviors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tamir_interface__msg__Behaviors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
