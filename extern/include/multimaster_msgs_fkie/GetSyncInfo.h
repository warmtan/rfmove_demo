// Generated by gencpp from file multimaster_msgs_fkie/GetSyncInfo.msg
// DO NOT EDIT!


#ifndef MULTIMASTER_MSGS_FKIE_MESSAGE_GETSYNCINFO_H
#define MULTIMASTER_MSGS_FKIE_MESSAGE_GETSYNCINFO_H

#include <ros/service_traits.h>


#include <multimaster_msgs_fkie/GetSyncInfoRequest.h>
#include <multimaster_msgs_fkie/GetSyncInfoResponse.h>


namespace multimaster_msgs_fkie
{

struct GetSyncInfo
{

typedef GetSyncInfoRequest Request;
typedef GetSyncInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetSyncInfo
} // namespace multimaster_msgs_fkie


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::multimaster_msgs_fkie::GetSyncInfo > {
  static const char* value()
  {
    return "d5261ec56e202860a07fb47b41e1b2a8";
  }

  static const char* value(const ::multimaster_msgs_fkie::GetSyncInfo&) { return value(); }
};

template<>
struct DataType< ::multimaster_msgs_fkie::GetSyncInfo > {
  static const char* value()
  {
    return "multimaster_msgs_fkie/GetSyncInfo";
  }

  static const char* value(const ::multimaster_msgs_fkie::GetSyncInfo&) { return value(); }
};


// service_traits::MD5Sum< ::multimaster_msgs_fkie::GetSyncInfoRequest> should match 
// service_traits::MD5Sum< ::multimaster_msgs_fkie::GetSyncInfo > 
template<>
struct MD5Sum< ::multimaster_msgs_fkie::GetSyncInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::multimaster_msgs_fkie::GetSyncInfo >::value();
  }
  static const char* value(const ::multimaster_msgs_fkie::GetSyncInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::multimaster_msgs_fkie::GetSyncInfoRequest> should match 
// service_traits::DataType< ::multimaster_msgs_fkie::GetSyncInfo > 
template<>
struct DataType< ::multimaster_msgs_fkie::GetSyncInfoRequest>
{
  static const char* value()
  {
    return DataType< ::multimaster_msgs_fkie::GetSyncInfo >::value();
  }
  static const char* value(const ::multimaster_msgs_fkie::GetSyncInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::multimaster_msgs_fkie::GetSyncInfoResponse> should match 
// service_traits::MD5Sum< ::multimaster_msgs_fkie::GetSyncInfo > 
template<>
struct MD5Sum< ::multimaster_msgs_fkie::GetSyncInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::multimaster_msgs_fkie::GetSyncInfo >::value();
  }
  static const char* value(const ::multimaster_msgs_fkie::GetSyncInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::multimaster_msgs_fkie::GetSyncInfoResponse> should match 
// service_traits::DataType< ::multimaster_msgs_fkie::GetSyncInfo > 
template<>
struct DataType< ::multimaster_msgs_fkie::GetSyncInfoResponse>
{
  static const char* value()
  {
    return DataType< ::multimaster_msgs_fkie::GetSyncInfo >::value();
  }
  static const char* value(const ::multimaster_msgs_fkie::GetSyncInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MULTIMASTER_MSGS_FKIE_MESSAGE_GETSYNCINFO_H