// Generated by gencpp from file pr2_mechanism_msgs/ListControllerTypes.msg
// DO NOT EDIT!


#ifndef PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERTYPES_H
#define PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERTYPES_H

#include <ros/service_traits.h>


#include <pr2_mechanism_msgs/ListControllerTypesRequest.h>
#include <pr2_mechanism_msgs/ListControllerTypesResponse.h>


namespace pr2_mechanism_msgs
{

struct ListControllerTypes
{

typedef ListControllerTypesRequest Request;
typedef ListControllerTypesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ListControllerTypes
} // namespace pr2_mechanism_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllerTypes > {
  static const char* value()
  {
    return "80aee506387f88339842e9a93044c959";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllerTypes&) { return value(); }
};

template<>
struct DataType< ::pr2_mechanism_msgs::ListControllerTypes > {
  static const char* value()
  {
    return "pr2_mechanism_msgs/ListControllerTypes";
  }

  static const char* value(const ::pr2_mechanism_msgs::ListControllerTypes&) { return value(); }
};


// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllerTypesRequest> should match 
// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllerTypes > 
template<>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllerTypesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pr2_mechanism_msgs::ListControllerTypes >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllerTypesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pr2_mechanism_msgs::ListControllerTypesRequest> should match 
// service_traits::DataType< ::pr2_mechanism_msgs::ListControllerTypes > 
template<>
struct DataType< ::pr2_mechanism_msgs::ListControllerTypesRequest>
{
  static const char* value()
  {
    return DataType< ::pr2_mechanism_msgs::ListControllerTypes >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllerTypesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllerTypesResponse> should match 
// service_traits::MD5Sum< ::pr2_mechanism_msgs::ListControllerTypes > 
template<>
struct MD5Sum< ::pr2_mechanism_msgs::ListControllerTypesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pr2_mechanism_msgs::ListControllerTypes >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllerTypesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pr2_mechanism_msgs::ListControllerTypesResponse> should match 
// service_traits::DataType< ::pr2_mechanism_msgs::ListControllerTypes > 
template<>
struct DataType< ::pr2_mechanism_msgs::ListControllerTypesResponse>
{
  static const char* value()
  {
    return DataType< ::pr2_mechanism_msgs::ListControllerTypes >::value();
  }
  static const char* value(const ::pr2_mechanism_msgs::ListControllerTypesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PR2_MECHANISM_MSGS_MESSAGE_LISTCONTROLLERTYPES_H
