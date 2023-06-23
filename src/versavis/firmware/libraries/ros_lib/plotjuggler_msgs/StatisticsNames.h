#ifndef _ROS_plotjuggler_msgs_StatisticsNames_h
#define _ROS_plotjuggler_msgs_StatisticsNames_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace plotjuggler_msgs
{

  class StatisticsNames : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t names_length;
      typedef char* _names_type;
      _names_type st_names;
      _names_type * names;
      typedef uint32_t _names_version_type;
      _names_version_type names_version;

    StatisticsNames():
      header(),
      names_length(0), st_names(), names(nullptr),
      names_version(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->names_length);
      for( uint32_t i = 0; i < names_length; i++){
      uint32_t length_namesi = strlen(this->names[i]);
      varToArr(outbuffer + offset, length_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->names[i], length_namesi);
      offset += length_namesi;
      }
      *(outbuffer + offset + 0) = (this->names_version >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->names_version >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->names_version >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->names_version >> (8 * 3)) & 0xFF;
      offset += sizeof(this->names_version);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->names_length);
      if(names_lengthT > names_length)
        this->names = (char**)realloc(this->names, names_lengthT * sizeof(char*));
      names_length = names_lengthT;
      for( uint32_t i = 0; i < names_length; i++){
      uint32_t length_st_names;
      arrToVar(length_st_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_names-1]=0;
      this->st_names = (char *)(inbuffer + offset-1);
      offset += length_st_names;
        memcpy( &(this->names[i]), &(this->st_names), sizeof(char*));
      }
      this->names_version =  ((uint32_t) (*(inbuffer + offset)));
      this->names_version |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->names_version |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->names_version |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->names_version);
     return offset;
    }

    virtual const char * getType() override { return "plotjuggler_msgs/StatisticsNames"; };
    virtual const char * getMD5() override { return "bece3d42a81d5c50cd68f110cf17bf55"; };

  };

}
#endif
