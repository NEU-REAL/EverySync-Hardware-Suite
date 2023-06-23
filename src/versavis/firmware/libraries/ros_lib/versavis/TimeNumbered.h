#ifndef _ROS_versavis_TimeNumbered_h
#define _ROS_versavis_TimeNumbered_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace versavis
{

  class TimeNumbered : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef uint64_t _number_type;
      _number_type number;

    TimeNumbered():
      time(),
      number(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
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
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
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

    virtual const char * getType() override { return "versavis/TimeNumbered"; };
    virtual const char * getMD5() override { return "a1a1eed10c75c0326a67a86fc18b029d"; };

  };

}
#endif
