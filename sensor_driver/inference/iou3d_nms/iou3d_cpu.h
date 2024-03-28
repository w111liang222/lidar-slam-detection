#ifndef IOU3D_CPU_H
#define IOU3D_CPU_H

#include <vector>

float iou_bev(const float *box_a, const float *box_b);
float overlap_bev(const float *box_a, const float *box_b);
float union_bev(const float *box_a, const float *box_b);

#endif
