// Generated by gencpp from file pr2_mechanism_msgs/ListControllers.msg
// DO NOT EDIT!


#ifndef PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERS_H
#define PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERS_H

#include <ros/service_traits.h>


#include <pr2_mechanism_msgs/ListControllersRequest.h>
#include <pr2_mechanism_msgs/ListControllersResponse.h>


namespace pr2_mechanism_msgs
{

struct ListControllers
{

typedef ListControllersRequest Request;
typedef ListControllersResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ListControllers
} // namespace pr2_mechanism_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllers > {
  static const char* value()
  {
    return "39c8d39516aed5c7d76284ac06c220e5";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllers&) { return value(); }
};

template<>
struct DataType< ::pr2_mechanism_msgs::ListControllers > {
  static const char* value()
  {
    return "pr2_mechanism_msgs/ListControllers";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllers&) { return value(); }
};


// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllersRequest> should match 
// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllers > 
template<>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllersRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pr2_mechanism_msgs::ListControllers >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllersRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pr2_mechanism_msgs::ListControllersRequest> should match 
// service_traits::DataType< ::pr2_mechanism_msgs::ListControllers > 
template<>
struct DataType< ::pr2_mechanism_msgs::ListControllersRequest>
{
  static const char* value()
  {
    return DataType< ::pr2_mechanism_msgs::ListControllers >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllersRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllersResponse> should match 
// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllers > 
template<>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllersResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pr2_mechanism_msgs::ListControllers >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllersResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pr2_mechanism_msgs::ListControllersResponse> should match 
// service_traits::DataType< ::pr2_mechanism_msgs::ListControllers > 
template<>
struct DataType< ::pr2_mechanism_msgs::ListControllersResponse>
{
  static const char* value()
  {
    return DataType< ::pr2_mechanism_msgs::ListControllers >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllersResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERS_H
