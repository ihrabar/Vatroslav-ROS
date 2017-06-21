// Generated by gencpp from file vatroslav/CanMsg.msg
// DO NOT EDIT!


#ifndef VATROSLAV_MESSAGE_CANMSG_H
#define VATROSLAV_MESSAGE_CANMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vatroslav
{
template <class ContainerAllocator>
struct CanMsg_
{
  typedef CanMsg_<ContainerAllocator> Type;

  CanMsg_()
    : id(0)
    , data()
    , size(0)
    , time()  {
    }
  CanMsg_(const ContainerAllocator& _alloc)
    : id(0)
    , data(_alloc)
    , size(0)
    , time()  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _data_type;
  _data_type data;

   typedef uint8_t _size_type;
  _size_type size;

   typedef ros::Time _time_type;
  _time_type time;




  typedef boost::shared_ptr< ::vatroslav::CanMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vatroslav::CanMsg_<ContainerAllocator> const> ConstPtr;

}; // struct CanMsg_

typedef ::vatroslav::CanMsg_<std::allocator<void> > CanMsg;

typedef boost::shared_ptr< ::vatroslav::CanMsg > CanMsgPtr;
typedef boost::shared_ptr< ::vatroslav::CanMsg const> CanMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vatroslav::CanMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vatroslav::CanMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vatroslav

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'vatroslav': ['/home/larics/catkin_ws/src/VIV/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vatroslav::CanMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vatroslav::CanMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vatroslav::CanMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vatroslav::CanMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vatroslav::CanMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vatroslav::CanMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vatroslav::CanMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fb033d7b450080f676a98353203d84c3";
  }

  static const char* value(const ::vatroslav::CanMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfb033d7b450080f6ULL;
  static const uint64_t static_value2 = 0x76a98353203d84c3ULL;
};

template<class ContainerAllocator>
struct DataType< ::vatroslav::CanMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vatroslav/CanMsg";
  }

  static const char* value(const ::vatroslav::CanMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vatroslav::CanMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id \n\
string data  \n\
uint8 size\n\
time time\n\
";
  }

  static const char* value(const ::vatroslav::CanMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vatroslav::CanMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.data);
      stream.next(m.size);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CanMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vatroslav::CanMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vatroslav::CanMsg_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "data: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.data);
    s << indent << "size: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.size);
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VATROSLAV_MESSAGE_CANMSG_H
