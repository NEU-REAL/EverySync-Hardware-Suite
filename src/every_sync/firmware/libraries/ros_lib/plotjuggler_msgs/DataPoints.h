#ifndef _ROS_plotjuggler_msgs_DataPoints_h
#define _ROS_plotjuggler_msgs_DataPoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "plotjuggler_msgs/DataPoint.h"

namespace plotjuggler_msgs
{

  class DataPoints : public ros::Msg
  {
    public:
      typedef uint32_t _dictionary_uuid_type;
      _dictionary_uuid_type dictionary_uuid;
      uint32_t samples_length;
      typedef plotjuggler_msgs::DataPoint _samples_type;
      _samples_type st_samples;
      _samples_type * samples;

    DataPoints():
      dictionary_uuid(0),
      samples_length(0), st_samples(), samples(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dictionary_uuid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dictionary_uuid >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dictionary_uuid >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dictionary_uuid >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dictionary_uuid);
      *(outbuffer + offset + 0) = (this->samples_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->samples_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->samples_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->samples_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->samples_length);
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->samples[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->dictionary_uuid =  ((uint32_t) (*(inbuffer + offset)));
      this->dictionary_uuid |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dictionary_uuid |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->dictionary_uuid |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->dictionary_uuid);
      uint32_t samples_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->samples_length);
      if(samples_lengthT > samples_length)
        this->samples = (plotjuggler_msgs::DataPoint*)realloc(this->samples, samples_lengthT * sizeof(plotjuggler_msgs::DataPoint));
      samples_length = samples_lengthT;
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->st_samples.deserialize(inbuffer + offset);
        memcpy( &(this->samples[i]), &(this->st_samples), sizeof(plotjuggler_msgs::DataPoint));
      }
     return offset;
    }

    virtual const char * getType() override { return "plotjuggler_msgs/DataPoints"; };
    virtual const char * getMD5() override { return "14e65e7956023a9a11291bc53d5d695a"; };

  };

}
#endif
