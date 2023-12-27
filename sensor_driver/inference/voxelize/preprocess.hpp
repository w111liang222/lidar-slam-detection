#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include <memory>
#include <vector>

#include "dtype.hpp"
#include "check.hpp"

struct PreprocessParameter {
  int max_points;
  int num_feature;
  int max_frame_num;
};

class Preprocess {
 public:
  virtual void forward(const float* points, int num_points, const float* motion, bool realtime, void* stream) = 0;

  virtual float* get_points() = 0;
  virtual int get_points_num() = 0;
};

std::shared_ptr<Preprocess> create_preprocess(PreprocessParameter parameter);

#endif  // __PREPROCESS_HPP__