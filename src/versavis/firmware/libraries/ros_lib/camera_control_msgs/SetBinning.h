#ifndef _ROS_SERVICE_SetBinning_h
#define _ROS_SERVICE_SetBinning_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETBINNING[] = "camera_control_msgs/SetBinning";

  class SetBinningRequest : public ros::Msg
  {
    public:
      typedef uint32_t _target_binning_x_type;
      _target_binning_x_type target_binning_x;
      typedef uint32_t _target_binning_y_type;
      _target_binning_y_type target_binning_y;

    SetBinningRequest():
      target_binning_x(0),
      target_binning_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->target_binning_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->target_binning_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->target_binning_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->target_binning_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_binning_x);
      *(outbuffer + offset + 0) = (this->target_binning_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->target_binning_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->target_binning_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->target_binning_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_binning_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->target_binning_x =  ((uint32_t) (*(inbuffer + offset)));
      this->target_binning_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->target_binning_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->target_binning_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->target_binning_x);
      this->target_binning_y =  ((uint32_t) (*(inbuffer + offset)));
      this->target_binning_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->target_binning_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->target_binning_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->target_binning_y);
     return offset;
    }

    virtual const char * getType() override { return SETBINNING; };
    virtual const char * getMD5() override { return "070d3ce2d671acce3b96edb95ae60995"; };

  };

  class SetBinningResponse : public ros::Msg
  {
    public:
      typedef uint32_t _reached_binning_x_type;
      _reached_binning_x_type reached_binning_x;
      typedef uint32_t _reached_binning_y_type;
      _reached_binning_y_type reached_binning_y;
      typedef bool _success_type;
      _success_type success;

    SetBinningResponse():
      reached_binning_x(0),
      reached_binning_y(0),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->reached_binning_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reached_binning_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reached_binning_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reached_binning_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_binning_x);
      *(outbuffer + offset + 0) = (this->reached_binning_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reached_binning_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reached_binning_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reached_binning_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_binning_y);
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->reached_binning_x =  ((uint32_t) (*(inbuffer + offset)));
      this->reached_binning_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->reached_binning_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->reached_binning_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->reached_binning_x);
      this->reached_binning_y =  ((uint32_t) (*(inbuffer + offset)));
      this->reached_binning_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->reached_binning_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->reached_binning_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->reached_binning_y);
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return SETBINNING; };
    virtual const char * getMD5() override { return "519ec992dd33973d0740b5940161bf8e"; };

  };

  class SetBinning {
    public:
    typedef SetBinningRequest Request;
    typedef SetBinningResponse Response;
  };

}
#endif
