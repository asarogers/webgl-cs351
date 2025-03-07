// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tamir_interface:msg/Behaviors.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tamir_interface/msg/detail/behaviors__functions.h"
#include "tamir_interface/msg/detail/behaviors__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tamir_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Behaviors_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tamir_interface::msg::Behaviors(_init);
}

void Behaviors_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tamir_interface::msg::Behaviors *>(message_memory);
  typed_message->~Behaviors();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Behaviors_message_member_array[2] = {
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tamir_interface::msg::Behaviors, name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tamir_interface::msg::Behaviors, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Behaviors_message_members = {
  "tamir_interface::msg",  // message namespace
  "Behaviors",  // message name
  2,  // number of fields
  sizeof(tamir_interface::msg::Behaviors),
  false,  // has_any_key_member_
  Behaviors_message_member_array,  // message members
  Behaviors_init_function,  // function to initialize message memory (memory has to be allocated)
  Behaviors_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Behaviors_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Behaviors_message_members,
  get_message_typesupport_handle_function,
  &tamir_interface__msg__Behaviors__get_type_hash,
  &tamir_interface__msg__Behaviors__get_type_description,
  &tamir_interface__msg__Behaviors__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tamir_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tamir_interface::msg::Behaviors>()
{
  return &::tamir_interface::msg::rosidl_typesupport_introspection_cpp::Behaviors_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tamir_interface, msg, Behaviors)() {
  return &::tamir_interface::msg::rosidl_typesupport_introspection_cpp::Behaviors_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
