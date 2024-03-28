/*
3D IoU Calculation and Rotated NMS(modified from 2D NMS written by others)
Written by Shaoshuai Shi
All Rights Reserved 2019-2020.
*/


#include <stdio.h>
#define THREADS_PER_BLOCK 16
#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

// #define DEBUG
const int THREADS_PER_BLOCK_NMS = sizeof(unsigned long long) * 8;
const float EPS = 1e-8;
struct Point {
    float x, y;
    __device__ Point() {}
    __device__ Point(double _x, double _y){
        x = _x, y = _y;
    }

    __device__ void set(float _x, float _y){
        x = _x; y = _y;
    }

    __device__ Point operator +(const Point &b)const{
        return Point(x + b.x, y + b.y);
    }

    __device__ Point operator -(const Point &b)const{
        return Point(x - b.x, y - b.y);
    }

    __device__ int operator <(const Point &b)const{
        if (x == b.x){
            return y >= b.y;
        }
        return x >= b.x;
    }
};

__device__ inline float cross(const Point &a, const Point &b){
    return a.x * b.y - a.y * b.x;
}

__device__ inline float cross(const Point &p1, const Point &p2, const Point &p0){
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

__device__ int check_rect_cross(const Point &p1, const Point &p2, const Point &q1, const Point &q2){
    int ret = min(p1.x,p2.x) <= max(q1.x,q2.x)  &&
              min(q1.x,q2.x) <= max(p1.x,p2.x) &&
              min(p1.y,p2.y) <= max(q1.y,q2.y) &&
              min(q1.y,q2.y) <= max(p1.y,p2.y);
    return ret;
}

__device__ inline int check_in_box2d(const float *box, const Point &p){
    //params: (7) [x, y, z, dx, dy, dz, heading]
    const float MARGIN = 1e-2;

    float center_x = box[0], center_y = box[1];
    float angle_cos = cos(-box[6]), angle_sin = sin(-box[6]);  // rotate the point in the opposite direction of box
    float rot_x = (p.x - center_x) * angle_cos + (p.y - center_y) * (-angle_sin);
    float rot_y = (p.x - center_x) * angle_sin + (p.y - center_y) * angle_cos;

    return (fabs(rot_x) < box[3] / 2 + MARGIN && fabs(rot_y) < box[4] / 2 + MARGIN);
}

__device__ inline int intersection(const Point &p1, const Point &p0, const Point &q1, const Point &q0, Point &ans){
    // fast exclusion
    if (check_rect_cross(p0, p1, q0, q1) == 0) return 0;

    // check cross standing
    float s1 = cross(q0, p1, p0);
    float s2 = cross(p1, q1, p0);
    float s3 = cross(p0, q1, q0);
    float s4 = cross(q1, p1, q0);

    if (!(s1 * s2 > 0 && s3 * s4 > 0)) return 0;

    // calculate intersection of two lines
    float s5 = cross(q1, p1, p0);
    if(fabs(s5 - s1) > EPS){
        ans.x = (s5 * q0.x - s1 * q1.x) / (s5 - s1);
        ans.y = (s5 * q0.y - s1 * q1.y) / (s5 - s1);

    }
    else{
        float a0 = p0.y - p1.y, b0 = p1.x - p0.x, c0 = p0.x * p1.y - p1.x * p0.y;
        float a1 = q0.y - q1.y, b1 = q1.x - q0.x, c1 = q0.x * q1.y - q1.x * q0.y;
        float D = a0 * b1 - a1 * b0;

        ans.x = (b0 * c1 - b1 * c0) / D;
        ans.y = (a1 * c0 - a0 * c1) / D;
    }

    return 1;
}

__device__ inline void rotate_around_center(const Point &center, const float angle_cos, const float angle_sin, Point &p){
    float new_x = (p.x - center.x) * angle_cos + (p.y - center.y) * (-angle_sin) + center.x;
    float new_y = (p.x - center.x) * angle_sin + (p.y - center.y) * angle_cos + center.y;
    p.set(new_x, new_y);
}

__device__ inline int point_cmp(const Point &a, const Point &b, const Point &center){
    return atan2(a.y - center.y, a.x - center.x) > atan2(b.y - center.y, b.x - center.x);
}

__device__ inline float box_overlap(const float *box_a, const float *box_b){
    // params box_a: [x, y, z, dx, dy, dz, heading]
    // params box_b: [x, y, z, dx, dy, dz, heading]

    float a_angle = box_a[6], b_angle = box_b[6];
    float a_dx_half = box_a[3] / 2, b_dx_half = box_b[3] / 2, a_dy_half = box_a[4] / 2, b_dy_half = box_b[4] / 2;
    float a_x1 = box_a[0] - a_dx_half, a_y1 = box_a[1] - a_dy_half;
    float a_x2 = box_a[0] + a_dx_half, a_y2 = box_a[1] + a_dy_half;
    float b_x1 = box_b[0] - b_dx_half, b_y1 = box_b[1] - b_dy_half;
    float b_x2 = box_b[0] + b_dx_half, b_y2 = box_b[1] + b_dy_half;

    Point center_a(box_a[0], box_a[1]);
    Point center_b(box_b[0], box_b[1]);

#ifdef DEBUG
    printf("a: (%.3f, %.3f, %.3f, %.3f, %.3f), b: (%.3f, %.3f, %.3f, %.3f, %.3f)\n", a_x1, a_y1, a_x2, a_y2, a_angle,
           b_x1, b_y1, b_x2, b_y2, b_angle);
    printf("center a: (%.3f, %.3f), b: (%.3f, %.3f)\n", center_a.x, center_a.y, center_b.x, center_b.y);
#endif

    Point box_a_corners[5];
    box_a_corners[0].set(a_x1, a_y1);
    box_a_corners[1].set(a_x2, a_y1);
    box_a_corners[2].set(a_x2, a_y2);
    box_a_corners[3].set(a_x1, a_y2);

    Point box_b_corners[5];
    box_b_corners[0].set(b_x1, b_y1);
    box_b_corners[1].set(b_x2, b_y1);
    box_b_corners[2].set(b_x2, b_y2);
    box_b_corners[3].set(b_x1, b_y2);

    // get oriented corners
    float a_angle_cos = cos(a_angle), a_angle_sin = sin(a_angle);
    float b_angle_cos = cos(b_angle), b_angle_sin = sin(b_angle);

    for (int k = 0; k < 4; k++){
#ifdef DEBUG
        printf("before corner %d: a(%.3f, %.3f), b(%.3f, %.3f) \n", k, box_a_corners[k].x, box_a_corners[k].y, box_b_corners[k].x, box_b_corners[k].y);
#endif
        rotate_around_center(center_a, a_angle_cos, a_angle_sin, box_a_corners[k]);
        rotate_around_center(center_b, b_angle_cos, b_angle_sin, box_b_corners[k]);
#ifdef DEBUG
        printf("corner %d: a(%.3f, %.3f), b(%.3f, %.3f) \n", k, box_a_corners[k].x, box_a_corners[k].y, box_b_corners[k].x, box_b_corners[k].y);
#endif
    }

    box_a_corners[4] = box_a_corners[0];
    box_b_corners[4] = box_b_corners[0];

    // get intersection of lines
    Point cross_points[16];
    Point poly_center;
    int cnt = 0, flag = 0;

    poly_center.set(0, 0);
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            flag = intersection(box_a_corners[i + 1], box_a_corners[i], box_b_corners[j + 1], box_b_corners[j], cross_points[cnt]);
            if (flag){
                poly_center = poly_center + cross_points[cnt];
                cnt++;
#ifdef DEBUG
                printf("Cross points (%.3f, %.3f): a(%.3f, %.3f)->(%.3f, %.3f), b(%.3f, %.3f)->(%.3f, %.3f) \n",
                    cross_points[cnt - 1].x, cross_points[cnt - 1].y,
                    box_a_corners[i].x, box_a_corners[i].y, box_a_corners[i + 1].x, box_a_corners[i + 1].y,
                    box_b_corners[i].x, box_b_corners[i].y, box_b_corners[i + 1].x, box_b_corners[i + 1].y);
#endif
            }
        }
    }

    // check corners
    for (int k = 0; k < 4; k++){
        if (check_in_box2d(box_a, box_b_corners[k])){
            poly_center = poly_center + box_b_corners[k];
            cross_points[cnt] = box_b_corners[k];
            cnt++;
#ifdef DEBUG
                printf("b corners in a: corner_b(%.3f, %.3f)", cross_points[cnt - 1].x, cross_points[cnt - 1].y);
#endif
        }
        if (check_in_box2d(box_b, box_a_corners[k])){
            poly_center = poly_center + box_a_corners[k];
            cross_points[cnt] = box_a_corners[k];
            cnt++;
#ifdef DEBUG
                printf("a corners in b: corner_a(%.3f, %.3f)", cross_points[cnt - 1].x, cross_points[cnt - 1].y);
#endif
        }
    }

    poly_center.x /= cnt;
    poly_center.y /= cnt;

    // sort the points of polygon
    Point temp;
    for (int j = 0; j < cnt - 1; j++){
        for (int i = 0; i < cnt - j - 1; i++){
            if (point_cmp(cross_points[i], cross_points[i + 1], poly_center)){
                temp = cross_points[i];
                cross_points[i] = cross_points[i + 1];
                cross_points[i + 1] = temp;
            }
        }
    }

#ifdef DEBUG
    printf("cnt=%d\n", cnt);
    for (int i = 0; i < cnt; i++){
        printf("All cross point %d: (%.3f, %.3f)\n", i, cross_points[i].x, cross_points[i].y);
    }
#endif

    // get the overlap areas
    float area = 0;
    for (int k = 0; k < cnt - 1; k++){
        area += cross(cross_points[k] - cross_points[0], cross_points[k + 1] - cross_points[0]);
    }

    return fabs(area) / 2.0;
}

__device__ inline float box_union(const float *box_a, const float *box_b){

    float a_angle = box_a[6], b_angle = box_b[6];
    float a_dx_half = box_a[3] / 2, b_dx_half = box_b[3] / 2, a_dy_half = box_a[4] / 2, b_dy_half = box_b[4] / 2;
    float a_x1 = box_a[0] - a_dx_half, a_y1 = box_a[1] - a_dy_half;
    float a_x2 = box_a[0] + a_dx_half, a_y2 = box_a[1] + a_dy_half;
    float b_x1 = box_b[0] - b_dx_half, b_y1 = box_b[1] - b_dy_half;
    float b_x2 = box_b[0] + b_dx_half, b_y2 = box_b[1] + b_dy_half;

    Point center_a(box_a[0], box_a[1]);
    Point center_b(box_b[0], box_b[1]);

    Point box_a_corners[4];
    box_a_corners[0].set(a_x1, a_y1);
    box_a_corners[1].set(a_x2, a_y1);
    box_a_corners[2].set(a_x2, a_y2);
    box_a_corners[3].set(a_x1, a_y2);

    Point box_b_corners[4];
    box_b_corners[0].set(b_x1, b_y1);
    box_b_corners[1].set(b_x2, b_y1);
    box_b_corners[2].set(b_x2, b_y2);
    box_b_corners[3].set(b_x1, b_y2);

    // get oriented corners
    float a_angle_cos = cos(a_angle), a_angle_sin = sin(a_angle);
    float b_angle_cos = cos(b_angle), b_angle_sin = sin(b_angle);

    for (int k = 0; k < 4; k++){
        rotate_around_center(center_a, a_angle_cos, a_angle_sin, box_a_corners[k]);
        rotate_around_center(center_b, b_angle_cos, b_angle_sin, box_b_corners[k]);
    }

    Point total_corners[8];
    for (int k = 0; k < 4; k++){
        total_corners[k] = box_a_corners[k];
        total_corners[k + 4] = box_b_corners[k];
    }

    Point temp;
    for (int k = 0; k < 8; k++){
        for (int i = 0; i < 8 - k - 1; i++){
            if (total_corners[i] < total_corners[i+1]){
                temp = total_corners[i];
                total_corners[i] = total_corners[i + 1];
                total_corners[i + 1] = temp;
            }
        }
    }
    // for (int k = 0; k < 8; k++){
    //     printf("All point %d: (%.3f, %.3f)\n", k, total_corners[k].x, total_corners[k].y);
    // }

    int pos = 1;
    int hull[9] = {0};
    int used[8] = {0};

    for (int k = 1; k < 8; k++){
        while(pos > 1 && cross(total_corners[hull[pos - 1]], total_corners[k], total_corners[hull[pos - 2]]) <= 0){
            used[hull[pos -1]] = 0;
            pos--;
        }
        used[k] = 1;
        hull[pos++] = k;
    }

    int m = pos;
    for (int k = 8 - 2; k >= 0; k--){
        if (!used[k]){
            while(pos > m && cross(total_corners[hull[pos - 1]], total_corners[k], total_corners[hull[pos - 2]]) <= 0){
                used[hull[pos -1]] = 0;
                pos--;
            }
            used[k] = 1;
            hull[pos++] = k;
        }
    }

    pos--;
    Point hull_pts[8];
    for (int k = 0; k < pos; k++){
        hull_pts[k] = total_corners[hull[k]];
        // printf("convex hull toto:%d %d %d: (%.3f, %.3f)\n", k, used[k], hull[k], hull_pts[k].x, hull_pts[k].y);
    }

    float area = 0;
    for (int k = 0; k < pos - 1; k++){
        area += cross(hull_pts[k] - hull_pts[0], hull_pts[k + 1] - hull_pts[0]);
    }

    return fabs(area) / 2.0;
}

__device__ inline float iou_bev(const float *box_a, const float *box_b){
    // params box_a: [x, y, z, dx, dy, dz, heading]
    // params box_b: [x, y, z, dx, dy, dz, heading]
    float sa = box_a[3] * box_a[4];
    float sb = box_b[3] * box_b[4];
    float s_overlap = box_overlap(box_a, box_b);
    return s_overlap / fmaxf(sa + sb - s_overlap, EPS);
}

__global__ void boxes_overlap_kernel(const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_overlap){
    // params boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
    // params boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]
    const int a_idx = blockIdx.y * THREADS_PER_BLOCK + threadIdx.y;
    const int b_idx = blockIdx.x * THREADS_PER_BLOCK + threadIdx.x;

    if (a_idx >= num_a || b_idx >= num_b){
        return;
    }
    const float * cur_box_a = boxes_a + a_idx * 7;
    const float * cur_box_b = boxes_b + b_idx * 7;
    float s_overlap = box_overlap(cur_box_a, cur_box_b);
    ans_overlap[a_idx * num_b + b_idx] = s_overlap;
}

__global__ void boxes_union_kernel(const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_union){
    // params boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
    // params boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]
    const int a_idx = blockIdx.y * THREADS_PER_BLOCK + threadIdx.y;
    const int b_idx = blockIdx.x * THREADS_PER_BLOCK + threadIdx.x;

    if (a_idx >= num_a || b_idx >= num_b){
        return;
    }

    const float * cur_box_a = boxes_a + a_idx * 7;
    const float * cur_box_b = boxes_b + b_idx * 7;
    float s_union = box_union(cur_box_a, cur_box_b);

    // printf("%d %d area: %.3f\n", a_idx, b_idx, s_union);
    ans_union[a_idx * num_b + b_idx] = s_union;
}

__global__ void nms_kernel(const int boxes_num, const float nms_overlap_thresh,
                           const float *boxes, unsigned long long *mask){
    //params: boxes (N, 7) [x, y, z, dx, dy, dz, heading]
    //params: mask (N, N/THREADS_PER_BLOCK_NMS)

    const int row_start = blockIdx.y;
    const int col_start = blockIdx.x;

    // if (row_start > col_start) return;

    const int row_size = fminf(boxes_num - row_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);
    const int col_size = fminf(boxes_num - col_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);

    __shared__ float block_boxes[THREADS_PER_BLOCK_NMS * 7];

    if (threadIdx.x < col_size) {
        block_boxes[threadIdx.x * 7 + 0] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 0];
        block_boxes[threadIdx.x * 7 + 1] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 1];
        block_boxes[threadIdx.x * 7 + 2] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 2];
        block_boxes[threadIdx.x * 7 + 3] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 3];
        block_boxes[threadIdx.x * 7 + 4] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 4];
        block_boxes[threadIdx.x * 7 + 5] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 5];
        block_boxes[threadIdx.x * 7 + 6] = boxes[(THREADS_PER_BLOCK_NMS * col_start + threadIdx.x) * 7 + 6];
    }
    __syncthreads();

    if (threadIdx.x < row_size) {
        const int cur_box_idx = THREADS_PER_BLOCK_NMS * row_start + threadIdx.x;
        const float *cur_box = boxes + cur_box_idx * 7;

        int i = 0;
        unsigned long long t = 0;
        int start = 0;
        if (row_start == col_start) {
          start = threadIdx.x + 1;
        }
        for (i = start; i < col_size; i++) {
            if (iou_bev(cur_box, block_boxes + i * 7) > nms_overlap_thresh){
                t |= 1ULL << i;
            }
        }
        const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);
        mask[cur_box_idx * col_blocks + col_start] = t;
    }
}


void nmsLauncher(const float *boxes, unsigned long long * mask, int boxes_num, float nms_overlap_thresh){
    dim3 blocks(DIVUP(boxes_num, THREADS_PER_BLOCK_NMS),
                DIVUP(boxes_num, THREADS_PER_BLOCK_NMS));
    dim3 threads(THREADS_PER_BLOCK_NMS);
    nms_kernel<<<blocks, threads>>>(boxes_num, nms_overlap_thresh, boxes, mask);
}

void boxesoverlapLauncher(const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_overlap){

    dim3 blocks(DIVUP(num_b, THREADS_PER_BLOCK), DIVUP(num_a, THREADS_PER_BLOCK));  // blockIdx.x(col), blockIdx.y(row)
    dim3 threads(THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    boxes_overlap_kernel<<<blocks, threads>>>(num_a, boxes_a, num_b, boxes_b, ans_overlap);
}

void boxesunionLauncher(const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_union){

    dim3 blocks(DIVUP(num_b, THREADS_PER_BLOCK), DIVUP(num_a, THREADS_PER_BLOCK));  // blockIdx.x(col), blockIdx.y(row)
    dim3 threads(THREADS_PER_BLOCK, THREADS_PER_BLOCK);
    boxes_union_kernel<<<blocks, threads>>>(num_a, boxes_a, num_b, boxes_b, ans_union);
}