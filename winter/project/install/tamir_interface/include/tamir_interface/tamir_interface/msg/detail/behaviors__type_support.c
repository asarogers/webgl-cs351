// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tamir_interface/msg/detail/behaviors__rosidl_typesupport_introspection_c.h"
#include "tamir_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tamir_interface/msg/detail/behaviors__functions.h"
#include "tamir_interface/msg/detail/behaviors__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tamir_interface__msg__Behaviors__init(message_memory);
}

void tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_fini_function(void * message_memory)
{
  tamir_interface__msg__Behaviors__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_member_array[2] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tamir_interface__msg__Behaviors, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tamir_interface__msg__Behaviors, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_members = {
  "tamir_interface__msg",  // message namespace
  "Behaviors",  // message name
  2,  // number of fields
  sizeof(tamir_interface__msg__Behaviors),
  false,  // has_any_key_member_
  tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_member_array,  // message members
  tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_init_function,  // function to initialize message memory (memory has to be allocated)
  tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_type_support_handle = {
  0,
  &tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_members,
  get_message_typesupport_handle_function,
  &tamir_interface__msg__Behaviors__get_type_hash,
  &tamir_interface__msg__Behaviors__get_type_description,
  &tamir_interface__msg__Behaviors__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tamir_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tamir_interface, msg, Behaviors)() {
  if (!tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_type_support_handle.typesupport_identifier) {
    tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tamir_interface__msg__Behaviors__rosidl_typesupport_introspection_c__Behaviors_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
