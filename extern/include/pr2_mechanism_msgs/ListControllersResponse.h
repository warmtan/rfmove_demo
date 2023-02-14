// Generated by gencpp from file pr2_mechanism_msgs/ListControllersResponse.msg
// DO NOT EDIT!


#ifndef PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERSRESPONSE_H
#define PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pr2_mechanism_msgs
{
template <class ContainerAllocator>
struct ListControllersResponse_
{
  typedef ListControllersResponse_<ContainerAllocator> Type;

  ListControllersResponse_()
    : controllers()
    , state()  {
    }
  ListControllersResponse_(const ContainerAllocator& _alloc)
    : controllers(_alloc)
    , state(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _controllers_type;
  _controllers_type controllers;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ListControllersResponse_

typedef ::pr2_mechanism_msgs::ListControllersResponse_<std::allocator<void> > ListControllersResponse;

typedef boost::shared_ptr< ::pr2_mechanism_msgs::ListControllersResponse > ListControllersResponsePtr;
typedef boost::shared_ptr< ::pr2_mechanism_msgs::ListControllersResponse const> ListControllersResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_mechanism_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'pr2_mechanism_msgs': ['/tmp/binarydeb/ros-kinetic-pr2-mechanism-msgs-1.8.2/obj-x86_64-linux-gnu/devel/share/pr2_mechanism_msgs/msg', '/tmp/binarydeb/ros-kinetic-pr2-mechanism-msgs-1.8.2/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39c8d39516aed5c7d76284ac06c220e5";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39c8d39516aed5c7ULL;
  static const uint64_t static_value2 = 0xd76284ac06c220e5ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_mechanism_msgs/ListControllersResponse";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] controllers\n\
string[] state\n\
";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.controllers);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ListControllersResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_mechanism_msgs::ListControllersResponse_<ContainerAllocator>& v)
  {
    s << indent << "controllers[]" << std::endl;
    for (size_t i = 0; i < v.controllers.size(); ++i)
    {
      s << indent << "  controllers[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.controllers[i]);
    }
    s << indent << "state[]" << std::endl;
    for (size_t i = 0; i < v.state.size(); ++i)
    {
      s << indent << "  state[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.state[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERSRESPONSE_H
