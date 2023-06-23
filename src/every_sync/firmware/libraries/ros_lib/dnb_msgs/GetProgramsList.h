#ifndef _ROS_SERVICE_GetProgramsList_h
#define _ROS_SERVICE_GetProgramsList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dnb_msgs/Program.h"

namespace dnb_msgs
{

static const char GETPROGRAMSLIST[] = "dnb_msgs/GetProgramsList";

  class GetProgramsListRequest : public ros::Msg
  {
    public:

    GetProgramsListRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETPROGRAMSLIST; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetProgramsListResponse : public ros::Msg
  {
    public:
      uint32_t programs_length;
      typedef dnb_msgs::Program _programs_type;
      _programs_type st_programs;
      _programs_type * programs;

    GetProgramsListResponse():
      programs_length(0), st_programs(), programs(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->programs_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->programs_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->programs_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->programs_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->programs_length);
      for( uint32_t i = 0; i < programs_length; i++){
      offset += this->programs[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t programs_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      programs_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      programs_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      programs_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->programs_length);
      if(programs_lengthT > programs_length)
        this->programs = (dnb_msgs::Program*)realloc(this->programs, programs_lengthT * sizeof(dnb_msgs::Program));
      programs_length = programs_lengthT;
      for( uint32_t i = 0; i < programs_length; i++){
      offset += this->st_programs.deserialize(inbuffer + offset);
        memcpy( &(this->programs[i]), &(this->st_programs), sizeof(dnb_msgs::Program));
      }
     return offset;
    }

    virtual const char * getType() override { return GETPROGRAMSLIST; };
    virtual const char * getMD5() override { return "72cea8df2b0c1e870498322f5ffbab0b"; };

  };

  class GetProgramsList {
    public:
    typedef GetProgramsListRequest Request;
    typedef GetProgramsListResponse Response;
  };

}
#endif
