#ifndef _ROS_perception_msgs_MarkerList_h
#define _ROS_perception_msgs_MarkerList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "perception_msgs/Marker.h"

namespace perception_msgs
{

  class MarkerList : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _imageTopicName_type;
      _imageTopicName_type imageTopicName;
      uint32_t markers_length;
      typedef perception_msgs::Marker _markers_type;
      _markers_type st_markers;
      _markers_type * markers;

    MarkerList():
      header(),
      imageTopicName(""),
      markers_length(0), markers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_imageTopicName = strlen(this->imageTopicName);
      varToArr(outbuffer + offset, length_imageTopicName);
      offset += 4;
      memcpy(outbuffer + offset, this->imageTopicName, length_imageTopicName);
      offset += length_imageTopicName;
      *(outbuffer + offset + 0) = (this->markers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->markers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->markers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->markers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->markers_length);
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_imageTopicName;
      arrToVar(length_imageTopicName, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_imageTopicName; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_imageTopicName-1]=0;
      this->imageTopicName = (char *)(inbuffer + offset-1);
      offset += length_imageTopicName;
      uint32_t markers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      markers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->markers_length);
      if(markers_lengthT > markers_length)
        this->markers = (perception_msgs::Marker*)realloc(this->markers, markers_lengthT * sizeof(perception_msgs::Marker));
      markers_length = markers_lengthT;
      for( uint32_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(perception_msgs::Marker));
      }
     return offset;
    }

    const char * getType(){ return "perception_msgs/MarkerList"; };
    const char * getMD5(){ return "9f0edb9bb4a3a12d927b2fa0af3294e1"; };

  };

}
#endif