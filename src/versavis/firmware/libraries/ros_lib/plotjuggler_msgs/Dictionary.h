#ifndef _ROS_plotjuggler_msgs_Dictionary_h
#define _ROS_plotjuggler_msgs_Dictionary_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace plotjuggler_msgs
{

  class Dictionary : public ros::Msg
  {
    public:
      typedef uint32_t _dictionary_uuid_type;
      _dictionary_uuid_type dictionary_uuid;
      uint32_t names_length;
      typedef char* _names_type;
      _names_type st_names;
      _names_type * names;

    Dictionary():
      dictionary_uuid(0),
      names_length(0), st_names(), names(nullptr)
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
     return offset;
    }

    virtual const char * getType() override { return "plotjuggler_msgs/Dictionary"; };
    virtual const char * getMD5() override { return "12d13553d8d6a9826829b71cac454ebe"; };

  };

}
#endif
