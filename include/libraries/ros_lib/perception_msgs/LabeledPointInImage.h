#ifndef _ROS_perception_msgs_LabeledPointInImage_h
#define _ROS_perception_msgs_LabeledPointInImage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "perception_msgs/PointInImage.h"

namespace perception_msgs
{

  class LabeledPointInImage : public ros::Msg
  {
    public:
      typedef const char* _value_type;
      _value_type value;
      typedef perception_msgs::PointInImage _pointsInImage_type;
      _pointsInImage_type pointsInImage;

    LabeledPointInImage():
      value(""),
      pointsInImage()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_value = strlen(this->value);
      varToArr(outbuffer + offset, length_value);
      offset += 4;
      memcpy(outbuffer + offset, this->value, length_value);
      offset += length_value;
      offset += this->pointsInImage.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_value;
      arrToVar(length_value, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_value-1]=0;
      this->value = (char *)(inbuffer + offset-1);
      offset += length_value;
      offset += this->pointsInImage.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "perception_msgs/LabeledPointInImage"; };
    const char * getMD5(){ return "1e84abf3e87ce958922be1dad2422c33"; };

  };

}
#endif