#ifndef _ROS_fiducial_detector_AprilTagList_h
#define _ROS_fiducial_detector_AprilTagList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "fiducial_detector/AprilTag.h"

namespace fiducial_detector
{

  class AprilTagList : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t april_tags_length;
      typedef fiducial_detector::AprilTag _april_tags_type;
      _april_tags_type st_april_tags;
      _april_tags_type * april_tags;

    AprilTagList():
      header(),
      april_tags_length(0), april_tags(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->april_tags_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->april_tags_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->april_tags_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->april_tags_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->april_tags_length);
      for( uint32_t i = 0; i < april_tags_length; i++){
      offset += this->april_tags[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t april_tags_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      april_tags_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      april_tags_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      april_tags_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->april_tags_length);
      if(april_tags_lengthT > april_tags_length)
        this->april_tags = (fiducial_detector::AprilTag*)realloc(this->april_tags, april_tags_lengthT * sizeof(fiducial_detector::AprilTag));
      april_tags_length = april_tags_lengthT;
      for( uint32_t i = 0; i < april_tags_length; i++){
      offset += this->st_april_tags.deserialize(inbuffer + offset);
        memcpy( &(this->april_tags[i]), &(this->st_april_tags), sizeof(fiducial_detector::AprilTag));
      }
     return offset;
    }

    const char * getType(){ return "fiducial_detector/AprilTagList"; };
    const char * getMD5(){ return "ddebcf771f5571c6f521a27b6a3f73b1"; };

  };

}
#endif