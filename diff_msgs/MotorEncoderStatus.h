// Generated by gencpp from file diff_msgs/MotorEncoderStatus.msg
// DO NOT EDIT!


#ifndef DIFF_MSGS_MESSAGE_MOTORENCODERSTATUS_H
#define DIFF_MSGS_MESSAGE_MOTORENCODERSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace diff_msgs
{
template <class ContainerAllocator>
struct MotorEncoderStatus_
{
  typedef MotorEncoderStatus_<ContainerAllocator> Type;

  MotorEncoderStatus_()
    : header()
    , leftMotorSteering(0.0)
    , rightMotorSteering(0.0)  {
    }
  MotorEncoderStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , leftMotorSteering(0.0)
    , rightMotorSteering(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _leftMotorSteering_type;
  _leftMotorSteering_type leftMotorSteering;

   typedef float _rightMotorSteering_type;
  _rightMotorSteering_type rightMotorSteering;





  typedef boost::shared_ptr< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> const> ConstPtr;

}; // struct MotorEncoderStatus_

typedef ::diff_msgs::MotorEncoderStatus_<std::allocator<void> > MotorEncoderStatus;

typedef boost::shared_ptr< ::diff_msgs::MotorEncoderStatus > MotorEncoderStatusPtr;
typedef boost::shared_ptr< ::diff_msgs::MotorEncoderStatus const> MotorEncoderStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace diff_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'diff_msgs': ['/home/lie/catkin_ws/src/diff_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4c4ecbbac42bb7c22bdc6d97bd32861c";
  }

  static const char* value(const ::diff_msgs::MotorEncoderStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4c4ecbbac42bb7c2ULL;
  static const uint64_t static_value2 = 0x2bdc6d97bd32861cULL;
};

template<class ContainerAllocator>
struct DataType< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "diff_msgs/MotorEncoderStatus";
  }

  static const char* value(const ::diff_msgs::MotorEncoderStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# 电机编码器状态\n\
# 报文标识符 0x402\n\
\n\
Header header\n\
float32 leftMotorSteering    #左轮电机转角 单位°\n\
float32 rightMotorSteering   #右轮电机转角 单位°\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::diff_msgs::MotorEncoderStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.leftMotorSteering);
      stream.next(m.rightMotorSteering);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorEncoderStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::diff_msgs::MotorEncoderStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::diff_msgs::MotorEncoderStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "leftMotorSteering: ";
    Printer<float>::stream(s, indent + "  ", v.leftMotorSteering);
    s << indent << "rightMotorSteering: ";
    Printer<float>::stream(s, indent + "  ", v.rightMotorSteering);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DIFF_MSGS_MESSAGE_MOTORENCODERSTATUS_H
