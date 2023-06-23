#ifndef _ROS_imu_numbered_msgs_ImuNumbered_h
#define _ROS_imu_numbered_msgs_ImuNumbered_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"

namespace imu_numbered_msgs
{

  class ImuNumbered : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef sensor_msgs::Imu _imu_type;
      _imu_type imu;
      typedef uint64_t _number_type;
      _number_type number;

    ImuNumbered():
      header(),
      imu(),
      number(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->imu.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->number >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->number >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->number >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->number >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->number >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->number >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->number >> (8 * 7)) & 0xFF;
      offset += sizeof(this->number);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->imu.deserialize(inbuffer + offset);
      this->number =  ((uint64_t) (*(inbuffer + offset)));
      this->number |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->number |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->number |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->number |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->number |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->number |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->number);
     return offset;
    }

    virtual const char * getType() override { return "imu_numbered_msgs/ImuNumbered"; };
    virtual const char * getMD5() override { return "6e0cc958e640de2408a57558e15090f0"; };

  };

}
#endif
