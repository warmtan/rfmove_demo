// Generated by gencpp from file controller_manager_msgs/HardwareInterfaceResources.msg
// DO NOT EDIT!


#ifndef CONTROLLER_MANAGER_MSGS_MESSAGE_HARDWAREINTERFACERESOURCES_H
#define CONTROLLER_MANAGER_MSGS_MESSAGE_HARDWAREINTERFACERESOURCES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace controller_manager_msgs
{
template <class ContainerAllocator>
struct HardwareInterfaceResources_
{
  typedef HardwareInterfaceResources_<ContainerAllocator> Type;

  HardwareInterfaceResources_()
    : hardware_interface()
    , resources()  {
    }
  HardwareInterfaceResources_(const ContainerAllocator& _alloc)
    : hardware_interface(_alloc)
    , resources(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _hardware_interface_type;
  _hardware_interface_type hardware_interface;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _resources_type;
  _resources_type resources;





  typedef boost::shared_ptr< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> const> ConstPtr;

}; // struct HardwareInterfaceResources_

typedef ::controller_manager_msgs::HardwareInterfaceResources_<std::allocator<void> > HardwareInterfaceResources;

typedef boost::shared_ptr< ::controller_manager_msgs::HardwareInterfaceResources > HardwareInterfaceResourcesPtr;
typedef boost::shared_ptr< ::controller_manager_msgs::HardwareInterfaceResources const> HardwareInterfaceResourcesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace controller_manager_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'controller_manager_msgs': ['/tmp/binarydeb/ros-kinetic-controller-manager-msgs-0.13.5/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f25b55cbf1d1f76e82e5ec9e83f76258";
  }

  static const char* value(const ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf25b55cbf1d1f76eULL;
  static const uint64_t static_value2 = 0x82e5ec9e83f76258ULL;
};

template<class ContainerAllocator>
struct DataType< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
{
  static const char* value()
  {
    return "controller_manager_msgs/HardwareInterfaceResources";
  }

  static const char* value(const ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Type of hardware interface\n\
string hardware_interface\n\
# List of resources belonging to the hardware interface\n\
string[] resources\n\
";
  }

  static const char* value(const ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.hardware_interface);
      stream.next(m.resources);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HardwareInterfaceResources_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::controller_manager_msgs::HardwareInterfaceResources_<ContainerAllocator>& v)
  {
    s << indent << "hardware_interface: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.hardware_interface);
    s << indent << "resources[]" << std::endl;
    for (size_t i = 0; i < v.resources.size(); ++i)
    {
      s << indent << "  resources[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.resources[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROLLER_MANAGER_MSGS_MESSAGE_HARDWAREINTERFACERESOURCES_H
