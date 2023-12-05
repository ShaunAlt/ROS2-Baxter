// Generated by gencpp from file baxter_core_msgs/BridgePublishersAuthResponse.msg
// DO NOT EDIT!


#ifndef BAXTER_CORE_MSGS_MESSAGE_BRIDGEPUBLISHERSAUTHRESPONSE_H
#define BAXTER_CORE_MSGS_MESSAGE_BRIDGEPUBLISHERSAUTHRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <baxter_core_msgs/BridgePublisher.h>

namespace baxter_core_msgs
{
template <class ContainerAllocator>
struct BridgePublishersAuthResponse_
{
  typedef BridgePublishersAuthResponse_<ContainerAllocator> Type;

  BridgePublishersAuthResponse_()
    : publishers()
    , forced_left()
    , forced_right()  {
    }
  BridgePublishersAuthResponse_(const ContainerAllocator& _alloc)
    : publishers(_alloc)
    , forced_left(_alloc)
    , forced_right(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::baxter_core_msgs::BridgePublisher_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::baxter_core_msgs::BridgePublisher_<ContainerAllocator> >> _publishers_type;
  _publishers_type publishers;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _forced_left_type;
  _forced_left_type forced_left;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _forced_right_type;
  _forced_right_type forced_right;





  typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> const> ConstPtr;

}; // struct BridgePublishersAuthResponse_

typedef ::baxter_core_msgs::BridgePublishersAuthResponse_<std::allocator<void> > BridgePublishersAuthResponse;

typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthResponse > BridgePublishersAuthResponsePtr;
typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthResponse const> BridgePublishersAuthResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator1> & lhs, const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator2> & rhs)
{
  return lhs.publishers == rhs.publishers &&
    lhs.forced_left == rhs.forced_left &&
    lhs.forced_right == rhs.forced_right;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator1> & lhs, const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace baxter_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "24a5e9d55c3f27d480c67c4c78f3ab3d";
  }

  static const char* value(const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x24a5e9d55c3f27d4ULL;
  static const uint64_t static_value2 = 0x80c67c4c78f3ab3dULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_core_msgs/BridgePublishersAuthResponse";
  }

  static const char* value(const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "BridgePublisher[] publishers\n"
"string forced_left\n"
"string forced_right\n"
"\n"
"\n"
"================================================================================\n"
"MSG: baxter_core_msgs/BridgePublisher\n"
"string topic\n"
"string user\n"
"float64 time\n"
;
  }

  static const char* value(const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.publishers);
      stream.next(m.forced_left);
      stream.next(m.forced_right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BridgePublishersAuthResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_core_msgs::BridgePublishersAuthResponse_<ContainerAllocator>& v)
  {
    s << indent << "publishers[]" << std::endl;
    for (size_t i = 0; i < v.publishers.size(); ++i)
    {
      s << indent << "  publishers[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::baxter_core_msgs::BridgePublisher_<ContainerAllocator> >::stream(s, indent + "    ", v.publishers[i]);
    }
    s << indent << "forced_left: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.forced_left);
    s << indent << "forced_right: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.forced_right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_CORE_MSGS_MESSAGE_BRIDGEPUBLISHERSAUTHRESPONSE_H
