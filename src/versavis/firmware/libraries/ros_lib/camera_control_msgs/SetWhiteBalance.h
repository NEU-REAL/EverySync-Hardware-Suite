#ifndef _ROS_SERVICE_SetWhiteBalance_h
#define _ROS_SERVICE_SetWhiteBalance_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETWHITEBALANCE[] = "camera_control_msgs/SetWhiteBalance";

  class SetWhiteBalanceRequest : public ros::Msg
  {
    public:
      typedef float _balance_ratio_red_type;
      _balance_ratio_red_type balance_ratio_red;
      typedef float _balance_ratio_green_type;
      _balance_ratio_green_type balance_ratio_green;
      typedef float _balance_ratio_blue_type;
      _balance_ratio_blue_type balance_ratio_blue;

    SetWhiteBalanceRequest():
      balance_ratio_red(0),
      balance_ratio_green(0),
      balance_ratio_blue(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_balance_ratio_red;
      u_balance_ratio_red.real = this->balance_ratio_red;
      *(outbuffer + offset + 0) = (u_balance_ratio_red.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_balance_ratio_red.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_balance_ratio_red.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_balance_ratio_red.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->balance_ratio_red);
      union {
        float real;
        uint32_t base;
      } u_balance_ratio_green;
      u_balance_ratio_green.real = this->balance_ratio_green;
      *(outbuffer + offset + 0) = (u_balance_ratio_green.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_balance_ratio_green.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_balance_ratio_green.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_balance_ratio_green.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->balance_ratio_green);
      union {
        float real;
        uint32_t base;
      } u_balance_ratio_blue;
      u_balance_ratio_blue.real = this->balance_ratio_blue;
      *(outbuffer + offset + 0) = (u_balance_ratio_blue.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_balance_ratio_blue.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_balance_ratio_blue.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_balance_ratio_blue.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->balance_ratio_blue);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_balance_ratio_red;
      u_balance_ratio_red.base = 0;
      u_balance_ratio_red.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_balance_ratio_red.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_balance_ratio_red.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_balance_ratio_red.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->balance_ratio_red = u_balance_ratio_red.real;
      offset += sizeof(this->balance_ratio_red);
      union {
        float real;
        uint32_t base;
      } u_balance_ratio_green;
      u_balance_ratio_green.base = 0;
      u_balance_ratio_green.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_balance_ratio_green.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_balance_ratio_green.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_balance_ratio_green.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->balance_ratio_green = u_balance_ratio_green.real;
      offset += sizeof(this->balance_ratio_green);
      union {
        float real;
        uint32_t base;
      } u_balance_ratio_blue;
      u_balance_ratio_blue.base = 0;
      u_balance_ratio_blue.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_balance_ratio_blue.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_balance_ratio_blue.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_balance_ratio_blue.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->balance_ratio_blue = u_balance_ratio_blue.real;
      offset += sizeof(this->balance_ratio_blue);
     return offset;
    }

    virtual const char * getType() override { return SETWHITEBALANCE; };
    virtual const char * getMD5() override { return "d091419509df5ea2efedf994e89474af"; };

  };

  class SetWhiteBalanceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    SetWhiteBalanceResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return SETWHITEBALANCE; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class SetWhiteBalance {
    public:
    typedef SetWhiteBalanceRequest Request;
    typedef SetWhiteBalanceResponse Response;
  };

}
#endif
