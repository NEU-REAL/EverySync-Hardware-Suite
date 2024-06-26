// Generated by gencpp from file versavis/TimeNumbered.msg
// DO NOT EDIT!


#ifndef VERSAVIS_MESSAGE_TIMENUMBERED_H
#define VERSAVIS_MESSAGE_TIMENUMBERED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace versavis
{
template <class ContainerAllocator>
struct TimeNumbered_
{
  typedef TimeNumbered_<ContainerAllocator> Type;

  TimeNumbered_()
    : time()
    , number(0)  {
    }
  TimeNumbered_(const ContainerAllocator& _alloc)
    : time()
    , number(0)  {
  (void)_alloc;
    }



   typedef ros::Time _time_type;
  _time_type time;

   typedef uint64_t _number_type;
  _number_type number;





  typedef boost::shared_ptr< ::versavis::TimeNumbered_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::versavis::TimeNumbered_<ContainerAllocator> const> ConstPtr;

}; // struct TimeNumbered_

typedef ::versavis::TimeNumbered_<std::allocator<void> > TimeNumbered;

typedef boost::shared_ptr< ::versavis::TimeNumbered > TimeNumberedPtr;
typedef boost::shared_ptr< ::versavis::TimeNumbered const> TimeNumberedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::versavis::TimeNumbered_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::versavis::TimeNumbered_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::versavis::TimeNumbered_<ContainerAllocator1> & lhs, const ::versavis::TimeNumbered_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time &&
    lhs.number == rhs.number;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::versavis::TimeNumbered_<ContainerAllocator1> & lhs, const ::versavis::TimeNumbered_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace versavis

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::versavis::TimeNumbered_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::versavis::TimeNumbered_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::versavis::TimeNumbered_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::versavis::TimeNumbered_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::versavis::TimeNumbered_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::versavis::TimeNumbered_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::versavis::TimeNumbered_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a1a1eed10c75c0326a67a86fc18b029d";
  }

  static const char* value(const ::versavis::TimeNumbered_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa1a1eed10c75c032ULL;
  static const uint64_t static_value2 = 0x6a67a86fc18b029dULL;
};

template<class ContainerAllocator>
struct DataType< ::versavis::TimeNumbered_<ContainerAllocator> >
{
  static const char* value()
  {
    return "versavis/TimeNumbered";
  }

  static const char* value(const ::versavis::TimeNumbered_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::versavis::TimeNumbered_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This is a time message including a sequence number for association with a payload message.\n"
"#\n"
"\n"
"time time\n"
"uint64 number\n"
;
  }

  static const char* value(const ::versavis::TimeNumbered_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::versavis::TimeNumbered_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.number);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimeNumbered_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::versavis::TimeNumbered_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::versavis::TimeNumbered_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
    s << indent << "number: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.number);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VERSAVIS_MESSAGE_TIMENUMBERED_H
