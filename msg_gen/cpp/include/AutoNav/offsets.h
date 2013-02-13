/* Auto-generated by genmsg_cpp for file /home/dev/ros/stacks/AutoNav/msg/offsets.msg */
#ifndef AUTONAV_MESSAGE_OFFSETS_H
#define AUTONAV_MESSAGE_OFFSETS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace AutoNav
{
template <class ContainerAllocator>
struct offsets_ {
  typedef offsets_<ContainerAllocator> Type;

  offsets_()
  : timestamp(0)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  {
  }

  offsets_(const ContainerAllocator& _alloc)
  : timestamp(0)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  {
  }

  typedef int32_t _timestamp_type;
  int32_t timestamp;

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _roll_type;
  double roll;

  typedef double _pitch_type;
  double pitch;

  typedef double _yaw_type;
  double yaw;


  typedef boost::shared_ptr< ::AutoNav::offsets_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::AutoNav::offsets_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct offsets
typedef  ::AutoNav::offsets_<std::allocator<void> > offsets;

typedef boost::shared_ptr< ::AutoNav::offsets> offsetsPtr;
typedef boost::shared_ptr< ::AutoNav::offsets const> offsetsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::AutoNav::offsets_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::AutoNav::offsets_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace AutoNav

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::AutoNav::offsets_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::AutoNav::offsets_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::AutoNav::offsets_<ContainerAllocator> > {
  static const char* value() 
  {
    return "54d823b33988acec25ffc8e2fbdd24a4";
  }

  static const char* value(const  ::AutoNav::offsets_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x54d823b33988acecULL;
  static const uint64_t static_value2 = 0x25ffc8e2fbdd24a4ULL;
};

template<class ContainerAllocator>
struct DataType< ::AutoNav::offsets_<ContainerAllocator> > {
  static const char* value() 
  {
    return "AutoNav/offsets";
  }

  static const char* value(const  ::AutoNav::offsets_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::AutoNav::offsets_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 timestamp\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
";
  }

  static const char* value(const  ::AutoNav::offsets_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::AutoNav::offsets_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::AutoNav::offsets_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.timestamp);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct offsets_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::AutoNav::offsets_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::AutoNav::offsets_<ContainerAllocator> & v) 
  {
    s << indent << "timestamp: ";
    Printer<int32_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};


} // namespace message_operations
} // namespace ros

#endif // AUTONAV_MESSAGE_OFFSETS_H
