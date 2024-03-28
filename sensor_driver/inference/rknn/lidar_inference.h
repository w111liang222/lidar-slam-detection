#include <memory>

#include "rknn_api.h"
#include "SystemUtils.h"
#include "Logger.h"
#include "half.hpp"

using half_float::half;
using namespace half_float::literal;

struct LidarEngineParameter {
  std::string rknn_file;
  std::vector<float> pointcloud_range;
  std::vector<float> voxel_size;
  std::vector<int>   grid_size;
  int num_proposal;
  int num_feature;
};

class LidarInference {
  private:
    LidarEngineParameter parameter_;
    half* input_data_;
    uint8_t* points_num_;

    unsigned int input_data_size_;
    unsigned int points_num_size_;
    // const paramaters
    float points_num_map_[64];
    std::vector<float> xs_;
    std::vector<float> ys_;
    int feature_stride_;
    int num_class_;

    // rknn
    bool valid_ = false;
    rknn_context ctx;
    rknn_input_output_num io_num;

  public:
    LidarInference(LidarEngineParameter parameter);
    ~LidarInference();

    void reset();
    int forward(const float* points, int point_num, const float* motion, bool runtime);
    void get_output(float* cls_preds, float* box_preds, int* label_preds, float* freespace);
    void pre_process(const float* points, int point_num);
    void post_process(half *heatmap, half *box_feat, float* cls_preds, float* box_preds, int* label_preds);
};

std::shared_ptr<LidarInference> create_engine(LidarEngineParameter parameter);