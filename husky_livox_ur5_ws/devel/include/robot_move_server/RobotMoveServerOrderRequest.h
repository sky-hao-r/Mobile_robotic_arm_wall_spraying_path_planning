// Generated by gencpp from file robot_move_server/RobotMoveServerOrderRequest.msg
// DO NOT EDIT!


#ifndef ROBOT_MOVE_SERVER_MESSAGE_ROBOTMOVESERVERORDERREQUEST_H
#define ROBOT_MOVE_SERVER_MESSAGE_ROBOTMOVESERVERORDERREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_move_server
{
template <class ContainerAllocator>
struct RobotMoveServerOrderRequest_
{
  typedef RobotMoveServerOrderRequest_<ContainerAllocator> Type;

  RobotMoveServerOrderRequest_()
    : mode()
    , value(0.0)  {
    }
  RobotMoveServerOrderRequest_(const ContainerAllocator& _alloc)
    : mode(_alloc)
    , value(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _mode_type;
  _mode_type mode;

   typedef float _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RobotMoveServerOrderRequest_

typedef ::robot_move_server::RobotMoveServerOrderRequest_<std::allocator<void> > RobotMoveServerOrderRequest;

typedef boost::shared_ptr< ::robot_move_server::RobotMoveServerOrderRequest > RobotMoveServerOrderRequestPtr;
typedef boost::shared_ptr< ::robot_move_server::RobotMoveServerOrderRequest const> RobotMoveServerOrderRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator1> & lhs, const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode &&
    lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator1> & lhs, const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_move_server

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7c91705426861d66b29c8f7060bf942b";
  }

  static const char* value(const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7c91705426861d66ULL;
  static const uint64_t static_value2 = 0xb29c8f7060bf942bULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_move_server/RobotMoveServerOrderRequest";
  }

  static const char* value(const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string mode\n"
"float32 value\n"
;
  }

  static const char* value(const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotMoveServerOrderRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_move_server::RobotMoveServerOrderRequest_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.mode);
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MOVE_SERVER_MESSAGE_ROBOTMOVESERVERORDERREQUEST_H
