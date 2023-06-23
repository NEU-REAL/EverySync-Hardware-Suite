#ifndef _ROS_plotjuggler_msgs_StatisticsValues_h
#define _ROS_plotjuggler_msgs_StatisticsValues_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace plotjuggler_msgs
{

  class StatisticsValues : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t values_length;
      typedef float _values_type;
      _values_type st_values;
      _values_type * values;
      typedef uint32_t _names_version_type;
      _names_version_type names_version;

    StatisticsValues():
      header(),
      values_length(0), st_values(), values(nullptr),
      names_version(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values_length);
      for( uint32_t i = 0; i < values_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->values[i]);
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
      uint32_t values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->values_length);
      if(values_lengthT > values_length)
        this->values = (float*)realloc(this->values, values_lengthT * sizeof(float));
      values_length = values_lengthT;
      for( uint32_t i = 0; i < values_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_values));
        memcpy( &(this->values[i]), &(this->st_values), sizeof(float));
      }
      this->names_version =  ((uint32_t) (*(inbuffer + offset)));
      this->names_version |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->names_version |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->names_version |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->names_version);
     return offset;
    }

    virtual const char * getType() override { return "plotjuggler_msgs/StatisticsValues"; };
    virtual const char * getMD5() override { return "44646896ace86f96c24fbb63054eeee8"; };

  };

}
#endif
