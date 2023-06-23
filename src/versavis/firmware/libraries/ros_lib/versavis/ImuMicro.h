#ifndef _ROS_versavis_ImuMicro_h
#define _ROS_versavis_ImuMicro_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Time.h"

namespace versavis
{

  class ImuMicro : public ros::Msg
  {
    public:
      typedef std_msgs::Time _time_type;
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

    ImuMicro():
      time(),
      ax(0),
      ay(0),
      az(0),
      gx(0),
      gy(0),
      gz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->time.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_ax;
      u_ax.real = this->ax;
      *(outbuffer + offset + 0) = (u_ax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ax.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ax);
      union {
        int16_t real;
        uint16_t base;
      } u_ay;
      u_ay.real = this->ay;
      *(outbuffer + offset + 0) = (u_ay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ay.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ay);
      union {
        int16_t real;
        uint16_t base;
      } u_az;
      u_az.real = this->az;
      *(outbuffer + offset + 0) = (u_az.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_az.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->az);
      union {
        int16_t real;
        uint16_t base;
      } u_gx;
      u_gx.real = this->gx;
      *(outbuffer + offset + 0) = (u_gx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gx.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gx);
      union {
        int16_t real;
        uint16_t base;
      } u_gy;
      u_gy.real = this->gy;
      *(outbuffer + offset + 0) = (u_gy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gy.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gy);
      union {
        int16_t real;
        uint16_t base;
      } u_gz;
      u_gz.real = this->gz;
      *(outbuffer + offset + 0) = (u_gz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gz.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->time.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_ax;
      u_ax.base = 0;
      u_ax.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ax.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ax = u_ax.real;
      offset += sizeof(this->ax);
      union {
        int16_t real;
        uint16_t base;
      } u_ay;
      u_ay.base = 0;
      u_ay.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ay.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ay = u_ay.real;
      offset += sizeof(this->ay);
      union {
        int16_t real;
        uint16_t base;
      } u_az;
      u_az.base = 0;
      u_az.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_az.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->az = u_az.real;
      offset += sizeof(this->az);
      union {
        int16_t real;
        uint16_t base;
      } u_gx;
      u_gx.base = 0;
      u_gx.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gx.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gx = u_gx.real;
      offset += sizeof(this->gx);
      union {
        int16_t real;
        uint16_t base;
      } u_gy;
      u_gy.base = 0;
      u_gy.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gy.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gy = u_gy.real;
      offset += sizeof(this->gy);
      union {
        int16_t real;
        uint16_t base;
      } u_gz;
      u_gz.base = 0;
      u_gz.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gz.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gz = u_gz.real;
      offset += sizeof(this->gz);
     return offset;
    }

    virtual const char * getType() override { return "versavis/ImuMicro"; };
    virtual const char * getMD5() override { return "129a0337b7155079d9bf5f4056e62132"; };

  };

}
#endif
