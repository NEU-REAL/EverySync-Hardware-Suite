#ifndef _ROS_SERVICE_DeviceInfo_h
#define _ROS_SERVICE_DeviceInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense2_camera
{

static const char DEVICEINFO[] = "realsense2_camera/DeviceInfo";

  class DeviceInfoRequest : public ros::Msg
  {
    public:

    DeviceInfoRequest()
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

    virtual const char * getType() override { return DEVICEINFO; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DeviceInfoResponse : public ros::Msg
  {
    public:
      typedef const char* _device_name_type;
      _device_name_type device_name;
      typedef const char* _serial_number_type;
      _serial_number_type serial_number;
      typedef const char* _firmware_version_type;
      _firmware_version_type firmware_version;
      typedef const char* _usb_type_descriptor_type;
      _usb_type_descriptor_type usb_type_descriptor;
      typedef const char* _firmware_update_id_type;
      _firmware_update_id_type firmware_update_id;
      typedef const char* _sensors_type;
      _sensors_type sensors;

    DeviceInfoResponse():
      device_name(""),
      serial_number(""),
      firmware_version(""),
      usb_type_descriptor(""),
      firmware_update_id(""),
      sensors("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_device_name = strlen(this->device_name);
      varToArr(outbuffer + offset, length_device_name);
      offset += 4;
      memcpy(outbuffer + offset, this->device_name, length_device_name);
      offset += length_device_name;
      uint32_t length_serial_number = strlen(this->serial_number);
      varToArr(outbuffer + offset, length_serial_number);
      offset += 4;
      memcpy(outbuffer + offset, this->serial_number, length_serial_number);
      offset += length_serial_number;
      uint32_t length_firmware_version = strlen(this->firmware_version);
      varToArr(outbuffer + offset, length_firmware_version);
      offset += 4;
      memcpy(outbuffer + offset, this->firmware_version, length_firmware_version);
      offset += length_firmware_version;
      uint32_t length_usb_type_descriptor = strlen(this->usb_type_descriptor);
      varToArr(outbuffer + offset, length_usb_type_descriptor);
      offset += 4;
      memcpy(outbuffer + offset, this->usb_type_descriptor, length_usb_type_descriptor);
      offset += length_usb_type_descriptor;
      uint32_t length_firmware_update_id = strlen(this->firmware_update_id);
      varToArr(outbuffer + offset, length_firmware_update_id);
      offset += 4;
      memcpy(outbuffer + offset, this->firmware_update_id, length_firmware_update_id);
      offset += length_firmware_update_id;
      uint32_t length_sensors = strlen(this->sensors);
      varToArr(outbuffer + offset, length_sensors);
      offset += 4;
      memcpy(outbuffer + offset, this->sensors, length_sensors);
      offset += length_sensors;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_device_name;
      arrToVar(length_device_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_device_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_device_name-1]=0;
      this->device_name = (char *)(inbuffer + offset-1);
      offset += length_device_name;
      uint32_t length_serial_number;
      arrToVar(length_serial_number, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_serial_number; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_serial_number-1]=0;
      this->serial_number = (char *)(inbuffer + offset-1);
      offset += length_serial_number;
      uint32_t length_firmware_version;
      arrToVar(length_firmware_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_firmware_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_firmware_version-1]=0;
      this->firmware_version = (char *)(inbuffer + offset-1);
      offset += length_firmware_version;
      uint32_t length_usb_type_descriptor;
      arrToVar(length_usb_type_descriptor, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_usb_type_descriptor; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_usb_type_descriptor-1]=0;
      this->usb_type_descriptor = (char *)(inbuffer + offset-1);
      offset += length_usb_type_descriptor;
      uint32_t length_firmware_update_id;
      arrToVar(length_firmware_update_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_firmware_update_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_firmware_update_id-1]=0;
      this->firmware_update_id = (char *)(inbuffer + offset-1);
      offset += length_firmware_update_id;
      uint32_t length_sensors;
      arrToVar(length_sensors, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensors; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensors-1]=0;
      this->sensors = (char *)(inbuffer + offset-1);
      offset += length_sensors;
     return offset;
    }

    virtual const char * getType() override { return DEVICEINFO; };
    virtual const char * getMD5() override { return "914e9cfa74a4f66f08c3fe1016943c1b"; };

  };

  class DeviceInfo {
    public:
    typedef DeviceInfoRequest Request;
    typedef DeviceInfoResponse Response;
  };

}
#endif
