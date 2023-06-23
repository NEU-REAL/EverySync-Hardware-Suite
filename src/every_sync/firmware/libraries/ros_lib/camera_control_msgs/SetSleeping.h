#ifndef _ROS_SERVICE_SetSleeping_h
#define _ROS_SERVICE_SetSleeping_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETSLEEPING[] = "camera_control_msgs/SetSleeping";

  class SetSleepingRequest : public ros::Msg
  {
    public:
      typedef bool _set_sleeping_type;
      _set_sleeping_type set_sleeping;

    SetSleepingRequest():
      set_sleeping(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_set_sleeping;
      u_set_sleeping.real = this->set_sleeping;
      *(outbuffer + offset + 0) = (u_set_sleeping.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->set_sleeping);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_set_sleeping;
      u_set_sleeping.base = 0;
      u_set_sleeping.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->set_sleeping = u_set_sleeping.real;
      offset += sizeof(this->set_sleeping);
     return offset;
    }

    virtual const char * getType() override { return SETSLEEPING; };
    virtual const char * getMD5() override { return "58aad3b532b8c4776fd592aec6441836"; };

  };

  class SetSleepingResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetSleepingResponse():
      success(0)
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
     return offset;
    }

    virtual const char * getType() override { return SETSLEEPING; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetSleeping {
    public:
    typedef SetSleepingRequest Request;
    typedef SetSleepingResponse Response;
  };

}
#endif
