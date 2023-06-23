#ifndef _ROS_ext_clk_msgs_GnssNumbered_h
#define _ROS_ext_clk_msgs_GnssNumbered_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ext_clk_msgs
{

  class GnssNumbered : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint64_t _number_type;
      _number_type number;

    GnssNumbered():
      header(),
      number(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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

    virtual const char * getType() override { return "ext_clk_msgs/GnssNumbered"; };
    virtual const char * getMD5() override { return "c138c1e390e7899af3df27ec04a0d8a7"; };

  };

}
#endif
