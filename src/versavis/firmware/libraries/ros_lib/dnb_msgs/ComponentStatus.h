#ifndef _ROS_dnb_msgs_ComponentStatus_h
#define _ROS_dnb_msgs_ComponentStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dnb_msgs
{

  class ComponentStatus : public ros::Msg
  {
    public:
      typedef int8_t _status_id_type;
      _status_id_type status_id;
      typedef const char* _status_msg_type;
      _status_msg_type status_msg;
      enum { INITIALIZED =  0 };
      enum { STOPPED =  1 };
      enum { RUNNING =  2 };
      enum { CONFIG_NEEDED =  3 };
      enum { ERROR =  4 };
      enum { INTERACTION_REQUEST =  5 };
      enum { DEACTIVATED =  6 };

    ComponentStatus():
      status_id(0),
      status_msg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status_id;
      u_status_id.real = this->status_id;
      *(outbuffer + offset + 0) = (u_status_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status_id);
      uint32_t length_status_msg = strlen(this->status_msg);
      varToArr(outbuffer + offset, length_status_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->status_msg, length_status_msg);
      offset += length_status_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status_id;
      u_status_id.base = 0;
      u_status_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status_id = u_status_id.real;
      offset += sizeof(this->status_id);
      uint32_t length_status_msg;
      arrToVar(length_status_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_msg-1]=0;
      this->status_msg = (char *)(inbuffer + offset-1);
      offset += length_status_msg;
     return offset;
    }

    virtual const char * getType() override { return "dnb_msgs/ComponentStatus"; };
    virtual const char * getMD5() override { return "3edf5da01aa562198fec3c468cecb80b"; };

  };

}
#endif
