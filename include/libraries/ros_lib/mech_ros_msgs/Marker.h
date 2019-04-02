#ifndef _ROS_mech_ros_msgs_Marker_h
#define _ROS_mech_ros_msgs_Marker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mech_ros_msgs/MarkerPose.h"

namespace mech_ros_msgs
{

  class Marker : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _id_type;
      _id_type id;
      typedef float _surface_type;
      _surface_type surface;
      typedef mech_ros_msgs::MarkerPose _pose_type;
      _pose_type pose;

    Marker():
      header(),
      id(""),
      surface(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      union {
        float real;
        uint32_t base;
      } u_surface;
      u_surface.real = this->surface;
      *(outbuffer + offset + 0) = (u_surface.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_surface.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_surface.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_surface.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->surface);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      union {
        float real;
        uint32_t base;
      } u_surface;
      u_surface.base = 0;
      u_surface.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_surface.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_surface.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_surface.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->surface = u_surface.real;
      offset += sizeof(this->surface);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mech_ros_msgs/Marker"; };
    const char * getMD5(){ return "794a4c56c0cf545ba5b6cdb8b3cc6114"; };

  };

}
#endif