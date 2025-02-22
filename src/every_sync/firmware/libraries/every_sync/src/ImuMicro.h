// Generated by gencpp from file versavis/ImuMicro.msg
// DO NOT EDIT!


#ifndef VERSAVIS_MESSAGE_IMUMICRO_H
#define VERSAVIS_MESSAGE_IMUMICRO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Time.h>

namespace versavis
{
template <class ContainerAllocator>
struct ImuMicro_
{
  typedef ImuMicro_<ContainerAllocator> Type;

  ImuMicro_()
    : time()
    , ax(0)
    , ay(0)
    , az(0)
    , gx(0)
    , gy(0)
    , gz(0)  {
    }
  ImuMicro_(const ContainerAllocator& _alloc)
    : time(_alloc)
    , ax(0)
    , ay(0)
    , az(0)
    , gx(0)
    , gy(0)
    , gz(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Time_<ContainerAllocator>  _time_type;
  _time_type time;

   typedef int16_t _ax_type;
  _ax_type ax;

   typedef int16_t _ay_type;
  _ay_type ay;

   typedef int16_t _az_type;
  _az_type az;

   typedef int16_t _gx_type;
  _gx_type gx;

   typedef int16_t _gy_type;
  _gy_type gy;

   typedef int16_t _gz_type;
  _gz_type gz;





  typedef boost::shared_ptr< ::versavis::ImuMicro_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::versavis::ImuMicro_<ContainerAllocator> const> ConstPtr;

}; // struct ImuMicro_

typedef ::versavis::ImuMicro_<std::allocator<void> > ImuMicro;

typedef boost::shared_ptr< ::versavis::ImuMicro > ImuMicroPtr;
typedef boost::shared_ptr< ::versavis::ImuMicro const> ImuMicroConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::versavis::ImuMicro_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::versavis::ImuMicro_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::versavis::ImuMicro_<ContainerAllocator1> & lhs, const ::versavis::ImuMicro_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time &&
    lhs.ax == rhs.ax &&
    lhs.ay == rhs.ay &&
    lhs.az == rhs.az &&
    lhs.gx == rhs.gx &&
    lhs.gy == rhs.gy &&
    lhs.gz == rhs.gz;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::versavis::ImuMicro_<ContainerAllocator1> & lhs, const ::versavis::ImuMicro_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace versavis

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::versavis::ImuMicro_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::versavis::ImuMicro_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::versavis::ImuMicro_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::versavis::ImuMicro_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::versavis::ImuMicro_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::versavis::ImuMicro_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::versavis::ImuMicro_<ContainerAllocator> >
{
  static const char* value()
  {
    return "129a0337b7155079d9bf5f4056e62132";
  }

  static const char* value(const ::versavis::ImuMicro_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x129a0337b7155079ULL;
  static const uint64_t static_value2 = 0xd9bf5f4056e62132ULL;
};

template<class ContainerAllocator>
struct DataType< ::versavis::ImuMicro_<ContainerAllocator> >
{
  static const char* value()
  {
    return "versavis/ImuMicro";
  }

  static const char* value(const ::versavis::ImuMicro_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::versavis::ImuMicro_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This is a super-minimized message to hold data from an IMU (Inertial Measurement Unit).\n"
"#\n"
"# Accelerations and rotational velocities are raw data and have to be scaled!\n"
"#\n"
"# To minimize the amount of data transfered by this message (e.g. form an Arduino through rosserial)\n"
"# the amount of data is truncated to its absolute minimum.\n"
"\n"
"std_msgs/Time time\n"
"\n"
"int16 ax\n"
"int16 ay\n"
"int16 az\n"
"int16 gx\n"
"int16 gy\n"
"int16 gz\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Time\n"
"time data\n"
;
  }

  static const char* value(const ::versavis::ImuMicro_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::versavis::ImuMicro_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.ax);
      stream.next(m.ay);
      stream.next(m.az);
      stream.next(m.gx);
      stream.next(m.gy);
      stream.next(m.gz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ImuMicro_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::versavis::ImuMicro_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::versavis::ImuMicro_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    s << std::endl;
    Printer< ::std_msgs::Time_<ContainerAllocator> >::stream(s, indent + "  ", v.time);
    s << indent << "ax: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ax);
    s << indent << "ay: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ay);
    s << indent << "az: ";
    Printer<int16_t>::stream(s, indent + "  ", v.az);
    s << indent << "gx: ";
    Printer<int16_t>::stream(s, indent + "  ", v.gx);
    s << indent << "gy: ";
    Printer<int16_t>::stream(s, indent + "  ", v.gy);
    s << indent << "gz: ";
    Printer<int16_t>::stream(s, indent + "  ", v.gz);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VERSAVIS_MESSAGE_IMUMICRO_H
