#ifndef _ROS_ouster_ros_PacketMsg_h
#define _ROS_ouster_ros_PacketMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ouster_ros
{

  class PacketMsg : public ros::Msg
  {
    public:
      uint32_t buf_length;
      typedef uint8_t _buf_type;
      _buf_type st_buf;
      _buf_type * buf;

    PacketMsg():
      buf_length(0), st_buf(), buf(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->buf_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->buf_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->buf_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->buf_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buf_length);
      for( uint32_t i = 0; i < buf_length; i++){
      *(outbuffer + offset + 0) = (this->buf[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->buf[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t buf_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      buf_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      buf_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      buf_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->buf_length);
      if(buf_lengthT > buf_length)
        this->buf = (uint8_t*)realloc(this->buf, buf_lengthT * sizeof(uint8_t));
      buf_length = buf_lengthT;
      for( uint32_t i = 0; i < buf_length; i++){
      this->st_buf =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_buf);
        memcpy( &(this->buf[i]), &(this->st_buf), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "ouster_ros/PacketMsg"; };
    virtual const char * getMD5() override { return "4f7b5949e76f86d01e96b0e33ba9b5e3"; };

  };

}
#endif
