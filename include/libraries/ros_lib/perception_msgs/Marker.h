#ifndef _ROS_perception_msgs_Marker_h
#define _ROS_perception_msgs_Marker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "perception_msgs/LabeledPointInImage.h"
#include "geometry_msgs/PoseWithCovariance.h"

namespace perception_msgs
{

  class Marker : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _id_type;
      _id_type id;
      uint32_t size_length;
      typedef float _size_type;
      _size_type st_size;
      _size_type * size;
      uint32_t labeledPointsInImage_length;
      typedef perception_msgs::LabeledPointInImage _labeledPointsInImage_type;
      _labeledPointsInImage_type st_labeledPointsInImage;
      _labeledPointsInImage_type * labeledPointsInImage;
      typedef bool _is3dReconstructed_type;
      _is3dReconstructed_type is3dReconstructed;
      typedef geometry_msgs::PoseWithCovariance _pose_type;
      _pose_type pose;
      typedef float _confidence_type;
      _confidence_type confidence;

    Marker():
      header(),
      id(""),
      size_length(0), size(NULL),
      labeledPointsInImage_length(0), labeledPointsInImage(NULL),
      is3dReconstructed(0),
      pose(),
      confidence(0)
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
      *(outbuffer + offset + 0) = (this->size_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->size_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->size_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->size_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size_length);
      for( uint32_t i = 0; i < size_length; i++){
      union {
        float real;
        uint32_t base;
      } u_sizei;
      u_sizei.real = this->size[i];
      *(outbuffer + offset + 0) = (u_sizei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sizei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sizei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sizei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size[i]);
      }
      *(outbuffer + offset + 0) = (this->labeledPointsInImage_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labeledPointsInImage_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labeledPointsInImage_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labeledPointsInImage_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labeledPointsInImage_length);
      for( uint32_t i = 0; i < labeledPointsInImage_length; i++){
      offset += this->labeledPointsInImage[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_is3dReconstructed;
      u_is3dReconstructed.real = this->is3dReconstructed;
      *(outbuffer + offset + 0) = (u_is3dReconstructed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is3dReconstructed);
      offset += this->pose.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->confidence);
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
      uint32_t size_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      size_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      size_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      size_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->size_length);
      if(size_lengthT > size_length)
        this->size = (float*)realloc(this->size, size_lengthT * sizeof(float));
      size_length = size_lengthT;
      for( uint32_t i = 0; i < size_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_size;
      u_st_size.base = 0;
      u_st_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_size = u_st_size.real;
      offset += sizeof(this->st_size);
        memcpy( &(this->size[i]), &(this->st_size), sizeof(float));
      }
      uint32_t labeledPointsInImage_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labeledPointsInImage_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labeledPointsInImage_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labeledPointsInImage_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labeledPointsInImage_length);
      if(labeledPointsInImage_lengthT > labeledPointsInImage_length)
        this->labeledPointsInImage = (perception_msgs::LabeledPointInImage*)realloc(this->labeledPointsInImage, labeledPointsInImage_lengthT * sizeof(perception_msgs::LabeledPointInImage));
      labeledPointsInImage_length = labeledPointsInImage_lengthT;
      for( uint32_t i = 0; i < labeledPointsInImage_length; i++){
      offset += this->st_labeledPointsInImage.deserialize(inbuffer + offset);
        memcpy( &(this->labeledPointsInImage[i]), &(this->st_labeledPointsInImage), sizeof(perception_msgs::LabeledPointInImage));
      }
      union {
        bool real;
        uint8_t base;
      } u_is3dReconstructed;
      u_is3dReconstructed.base = 0;
      u_is3dReconstructed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is3dReconstructed = u_is3dReconstructed.real;
      offset += sizeof(this->is3dReconstructed);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->confidence));
     return offset;
    }

    const char * getType(){ return "perception_msgs/Marker"; };
    const char * getMD5(){ return "da8deb44ff01a0f90906148027b06f04"; };

  };

}
#endif