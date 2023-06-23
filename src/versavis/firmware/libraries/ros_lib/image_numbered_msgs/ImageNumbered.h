#ifndef _ROS_image_numbered_msgs_ImageNumbered_h
#define _ROS_image_numbered_msgs_ImageNumbered_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

namespace image_numbered_msgs
{

  class ImageNumbered : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef sensor_msgs::Image _image_type;
      _image_type image;
      typedef uint64_t _number_type;
      _number_type number;
      typedef float _exposure_type;
      _exposure_type exposure;

    ImageNumbered():
      header(),
      image(),
      number(0),
      exposure(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->image.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->number >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->number >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->number >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->number >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->number >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->number >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->number >> (8 * 7)) & 0xFF;
      offset += sizeof(this->number);
      union {
        float real;
        uint32_t base;
      } u_exposure;
      u_exposure.real = this->exposure;
      *(outbuffer + offset + 0) = (u_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->exposure);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->image.deserialize(inbuffer + offset);
      this->number =  ((uint64_t) (*(inbuffer + offset)));
      this->number |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->number |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->number |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->number |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->number |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->number |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->number);
      union {
        float real;
        uint32_t base;
      } u_exposure;
      u_exposure.base = 0;
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->exposure = u_exposure.real;
      offset += sizeof(this->exposure);
     return offset;
    }

    virtual const char * getType() override { return "image_numbered_msgs/ImageNumbered"; };
    virtual const char * getMD5() override { return "7bf045a4bd90f03357c1865754873988"; };

  };

}
#endif
