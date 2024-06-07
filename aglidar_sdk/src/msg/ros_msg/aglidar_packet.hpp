// Generated by gencpp from file aglidar_msg/AglidarPacket.msg
// DO NOT EDIT!


#ifndef AGLIDAR_MSG_MESSAGE_AGLIDARPACKET_H
#define AGLIDAR_MSG_MESSAGE_AGLIDARPACKET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace aglidar_msg
{
template <class ContainerAllocator>
struct AglidarPacket_
{
  typedef AglidarPacket_<ContainerAllocator> Type;

  AglidarPacket_()
    : header()
    , is_difop(0)
    , is_frame_begin(0)
    , data()  {
    }
  AglidarPacket_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , is_difop(0)
    , is_frame_begin(0)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _is_difop_type;
  _is_difop_type is_difop;

   typedef uint8_t _is_frame_begin_type;
  _is_frame_begin_type is_frame_begin;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::aglidar_msg::AglidarPacket_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aglidar_msg::AglidarPacket_<ContainerAllocator> const> ConstPtr;

}; // struct AglidarPacket_

typedef ::aglidar_msg::AglidarPacket_<std::allocator<void> > AglidarPacket;

typedef boost::shared_ptr< ::aglidar_msg::AglidarPacket > AglidarPacketPtr;
typedef boost::shared_ptr< ::aglidar_msg::AglidarPacket const> AglidarPacketConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aglidar_msg::AglidarPacket_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::aglidar_msg::AglidarPacket_<ContainerAllocator1> & lhs, const ::aglidar_msg::AglidarPacket_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.is_difop == rhs.is_difop &&
    lhs.is_frame_begin == rhs.is_frame_begin &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::aglidar_msg::AglidarPacket_<ContainerAllocator1> & lhs, const ::aglidar_msg::AglidarPacket_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace aglidar_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aglidar_msg::AglidarPacket_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aglidar_msg::AglidarPacket_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aglidar_msg::AglidarPacket_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b1cc155a9097c0cb935a7abf46d6eef";
  }

  static const char* value(const ::aglidar_msg::AglidarPacket_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b1cc155a9097c0cULL;
  static const uint64_t static_value2 = 0xb935a7abf46d6eefULL;
};

template<class ContainerAllocator>
struct DataType< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aglidar_msg/AglidarPacket";
  }

  static const char* value(const ::aglidar_msg::AglidarPacket_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"uint8 is_difop\n"
"uint8 is_frame_begin\n"
"uint8[] data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::aglidar_msg::AglidarPacket_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.is_difop);
      stream.next(m.is_frame_begin);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AglidarPacket_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aglidar_msg::AglidarPacket_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aglidar_msg::AglidarPacket_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "is_difop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_difop);
    s << indent << "is_frame_begin: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_frame_begin);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AgLIDAR_MSG_MESSAGE_AgLIDARPACKET_H