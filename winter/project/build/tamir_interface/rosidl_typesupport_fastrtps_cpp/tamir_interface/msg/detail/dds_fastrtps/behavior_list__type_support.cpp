// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from tamir_interface:msg/BehaviorList.idl
// generated code does not contain a copyright notice
#include "tamir_interface/msg/detail/behavior_list__rosidl_typesupport_fastrtps_cpp.hpp"
#include "tamir_interface/msg/detail/behavior_list__functions.h"
#include "tamir_interface/msg/detail/behavior_list__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace tamir_interface
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const tamir_interface::msg::Behaviors &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  tamir_interface::msg::Behaviors &);
size_t get_serialized_size(
  const tamir_interface::msg::Behaviors &,
  size_t current_alignment);
size_t
max_serialized_size_Behaviors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const tamir_interface::msg::Behaviors &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const tamir_interface::msg::Behaviors &,
  size_t current_alignment);
size_t
max_serialized_size_key_Behaviors(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace tamir_interface


namespace tamir_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
cdr_serialize(
  const tamir_interface::msg::BehaviorList & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: states
  {
    size_t size = ros_message.states.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      tamir_interface::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.states[i],
        cdr);
    }
  }

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  tamir_interface::msg::BehaviorList & ros_message)
{
  // Member: states
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.states.resize(size);
    for (size_t i = 0; i < size; i++) {
      tamir_interface::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.states[i]);
    }
  }

  return true;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
get_serialized_size(
  const tamir_interface::msg::BehaviorList & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: states
  {
    size_t array_size = ros_message.states.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        tamir_interface::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.states[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
max_serialized_size_BehaviorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        tamir_interface::msg::typesupport_fastrtps_cpp::max_serialized_size_Behaviors(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = tamir_interface::msg::BehaviorList;
    is_plain =
      (
      offsetof(DataType, states) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
cdr_serialize_key(
  const tamir_interface::msg::BehaviorList & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: states
  {
    size_t size = ros_message.states.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      tamir_interface::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
        ros_message.states[i],
        cdr);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
get_serialized_size_key(
  const tamir_interface::msg::BehaviorList & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: states
  {
    size_t array_size = ros_message.states.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        tamir_interface::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
        ros_message.states[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tamir_interface
max_serialized_size_key_BehaviorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: states
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        tamir_interface::msg::typesupport_fastrtps_cpp::max_serialized_size_key_Behaviors(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = tamir_interface::msg::BehaviorList;
    is_plain =
      (
      offsetof(DataType, states) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _BehaviorList__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const tamir_interface::msg::BehaviorList *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BehaviorList__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<tamir_interface::msg::BehaviorList *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BehaviorList__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const tamir_interface::msg::BehaviorList *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BehaviorList__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_BehaviorList(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _BehaviorList__callbacks = {
  "tamir_interface::msg",
  "BehaviorList",
  _BehaviorList__cdr_serialize,
  _BehaviorList__cdr_deserialize,
  _BehaviorList__get_serialized_size,
  _BehaviorList__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _BehaviorList__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BehaviorList__callbacks,
  get_message_typesupport_handle_function,
  &tamir_interface__msg__BehaviorList__get_type_hash,
  &tamir_interface__msg__BehaviorList__get_type_description,
  &tamir_interface__msg__BehaviorList__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace tamir_interface

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_tamir_interface
const rosidl_message_type_support_t *
get_message_type_support_handle<tamir_interface::msg::BehaviorList>()
{
  return &tamir_interface::msg::typesupport_fastrtps_cpp::_BehaviorList__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tamir_interface, msg, BehaviorList)() {
  return &tamir_interface::msg::typesupport_fastrtps_cpp::_BehaviorList__handle;
}

#ifdef __cplusplus
}
#endif
