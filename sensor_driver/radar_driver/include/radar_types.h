#ifndef __RADAR_TYPES__H
#define __RADAR_TYPES__H

struct RadarDataType{
    int32_t id;
    float x;
    float y;
    float z;
    float vx = 0.0;
    float vy = 0.0;
    float ax;
    float ay;
    float ang;
    float  length;
    float  width;
    int8_t type;
};

#endif