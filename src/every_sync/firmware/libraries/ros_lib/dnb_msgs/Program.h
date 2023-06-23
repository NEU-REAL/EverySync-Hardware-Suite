#ifndef _ROS_dnb_msgs_Program_h
#define _ROS_dnb_msgs_Program_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dnb_msgs/Argument.h"

namespace dnb_msgs
{

  class Program : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      uint32_t arguments_length;
      typedef dnb_msgs::Argument _arguments_type;
      _arguments_type st_arguments;
      _arguments_type * arguments;

    Program():
      name(""),
      arguments_length(0), st_arguments(), arguments(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->arguments_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arguments_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arguments_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arguments_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arguments_length);
      for( uint32_t i = 0; i < arguments_length; i++){
      offset += this->arguments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t arguments_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      arguments_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      arguments_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      arguments_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->arguments_length);
      if(arguments_lengthT > arguments_length)
        this->arguments = (dnb_msgs::Argument*)realloc(this->arguments, arguments_lengthT * sizeof(dnb_msgs::Argument));
      arguments_length = arguments_lengthT;
      for( uint32_t i = 0; i < arguments_length; i++){
      offset += this->st_arguments.deserialize(inbuffer + offset);
        memcpy( &(this->arguments[i]), &(this->st_arguments), sizeof(dnb_msgs::Argument));
      }
     return offset;
    }

    virtual const char * getType() override { return "dnb_msgs/Program"; };
    virtual const char * getMD5() override { return "1d51239b0eb86a694c9fb7b883a96b58"; };

  };

}
#endif
