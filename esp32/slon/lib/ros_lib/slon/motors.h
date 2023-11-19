#ifndef _ROS_slon_motors_h
#define _ROS_slon_motors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace slon
{

  class motors : public ros::Msg
  {
    public:
      typedef int32_t _l_speed_type;
      _l_speed_type l_speed;
      typedef int32_t _r_speed_type;
      _r_speed_type r_speed;

    motors():
      l_speed(0),
      r_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_l_speed;
      u_l_speed.real = this->l_speed;
      *(outbuffer + offset + 0) = (u_l_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_r_speed;
      u_r_speed.real = this->r_speed;
      *(outbuffer + offset + 0) = (u_r_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_l_speed;
      u_l_speed.base = 0;
      u_l_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_speed = u_l_speed.real;
      offset += sizeof(this->l_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_r_speed;
      u_r_speed.base = 0;
      u_r_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_speed = u_r_speed.real;
      offset += sizeof(this->r_speed);
     return offset;
    }

    virtual const char * getType() override { return "slon/motors"; };
    virtual const char * getMD5() override { return "a478e7978c119070292f7b47b188302b"; };

  };

}
#endif
