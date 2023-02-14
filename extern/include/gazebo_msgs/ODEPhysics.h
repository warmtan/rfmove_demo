// Generated by gencpp from file gazebo_msgs/ODEPhysics.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_ODEPHYSICS_H
#define GAZEBO_MSGS_MESSAGE_ODEPHYSICS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gazebo_msgs
{
template <class ContainerAllocator>
struct ODEPhysics_
{
  typedef ODEPhysics_<ContainerAllocator> Type;

  ODEPhysics_()
    : auto_disable_bodies(false)
    , sor_pgs_precon_iters(0)
    , sor_pgs_iters(0)
    , sor_pgs_w(0.0)
    , sor_pgs_rms_error_tol(0.0)
    , contact_surface_layer(0.0)
    , contact_max_correcting_vel(0.0)
    , cfm(0.0)
    , erp(0.0)
    , max_contacts(0)  {
    }
  ODEPhysics_(const ContainerAllocator& _alloc)
    : auto_disable_bodies(false)
    , sor_pgs_precon_iters(0)
    , sor_pgs_iters(0)
    , sor_pgs_w(0.0)
    , sor_pgs_rms_error_tol(0.0)
    , contact_surface_layer(0.0)
    , contact_max_correcting_vel(0.0)
    , cfm(0.0)
    , erp(0.0)
    , max_contacts(0)  {
  (void)_alloc;
    }



   typedef uint8_t _auto_disable_bodies_type;
  _auto_disable_bodies_type auto_disable_bodies;

   typedef uint32_t _sor_pgs_precon_iters_type;
  _sor_pgs_precon_iters_type sor_pgs_precon_iters;

   typedef uint32_t _sor_pgs_iters_type;
  _sor_pgs_iters_type sor_pgs_iters;

   typedef double _sor_pgs_w_type;
  _sor_pgs_w_type sor_pgs_w;

   typedef double _sor_pgs_rms_error_tol_type;
  _sor_pgs_rms_error_tol_type sor_pgs_rms_error_tol;

   typedef double _contact_surface_layer_type;
  _contact_surface_layer_type contact_surface_layer;

   typedef double _contact_max_correcting_vel_type;
  _contact_max_correcting_vel_type contact_max_correcting_vel;

   typedef double _cfm_type;
  _cfm_type cfm;

   typedef double _erp_type;
  _erp_type erp;

   typedef uint32_t _max_contacts_type;
  _max_contacts_type max_contacts;





  typedef boost::shared_ptr< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> const> ConstPtr;

}; // struct ODEPhysics_

typedef ::gazebo_msgs::ODEPhysics_<std::allocator<void> > ODEPhysics;

typedef boost::shared_ptr< ::gazebo_msgs::ODEPhysics > ODEPhysicsPtr;
typedef boost::shared_ptr< ::gazebo_msgs::ODEPhysics const> ODEPhysicsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_msgs::ODEPhysics_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gazebo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'gazebo_msgs': ['/tmp/binarydeb/ros-kinetic-gazebo-msgs-2.5.21/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "667d56ddbd547918c32d1934503dc335";
  }

  static const char* value(const ::gazebo_msgs::ODEPhysics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x667d56ddbd547918ULL;
  static const uint64_t static_value2 = 0xc32d1934503dc335ULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_msgs/ODEPhysics";
  }

  static const char* value(const ::gazebo_msgs::ODEPhysics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool auto_disable_bodies           # enable auto disabling of bodies, default false\n\
uint32 sor_pgs_precon_iters        # preconditioning inner iterations when uisng projected Gauss Seidel\n\
uint32 sor_pgs_iters               # inner iterations when uisng projected Gauss Seidel\n\
float64 sor_pgs_w                  # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation\n\
float64 sor_pgs_rms_error_tol      # rms error tolerance before stopping inner iterations\n\
float64 contact_surface_layer      # contact \"dead-band\" width\n\
float64 contact_max_correcting_vel # contact maximum correction velocity\n\
float64 cfm                        # global constraint force mixing\n\
float64 erp                        # global error reduction parameter\n\
uint32 max_contacts                # maximum contact joints between two geoms\n\
";
  }

  static const char* value(const ::gazebo_msgs::ODEPhysics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.auto_disable_bodies);
      stream.next(m.sor_pgs_precon_iters);
      stream.next(m.sor_pgs_iters);
      stream.next(m.sor_pgs_w);
      stream.next(m.sor_pgs_rms_error_tol);
      stream.next(m.contact_surface_layer);
      stream.next(m.contact_max_correcting_vel);
      stream.next(m.cfm);
      stream.next(m.erp);
      stream.next(m.max_contacts);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ODEPhysics_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_msgs::ODEPhysics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_msgs::ODEPhysics_<ContainerAllocator>& v)
  {
    s << indent << "auto_disable_bodies: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.auto_disable_bodies);
    s << indent << "sor_pgs_precon_iters: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sor_pgs_precon_iters);
    s << indent << "sor_pgs_iters: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sor_pgs_iters);
    s << indent << "sor_pgs_w: ";
    Printer<double>::stream(s, indent + "  ", v.sor_pgs_w);
    s << indent << "sor_pgs_rms_error_tol: ";
    Printer<double>::stream(s, indent + "  ", v.sor_pgs_rms_error_tol);
    s << indent << "contact_surface_layer: ";
    Printer<double>::stream(s, indent + "  ", v.contact_surface_layer);
    s << indent << "contact_max_correcting_vel: ";
    Printer<double>::stream(s, indent + "  ", v.contact_max_correcting_vel);
    s << indent << "cfm: ";
    Printer<double>::stream(s, indent + "  ", v.cfm);
    s << indent << "erp: ";
    Printer<double>::stream(s, indent + "  ", v.erp);
    s << indent << "max_contacts: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.max_contacts);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_ODEPHYSICS_H
