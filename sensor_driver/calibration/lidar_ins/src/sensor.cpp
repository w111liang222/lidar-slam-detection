#include "sensor.h"

OdomTformData::OdomTformData(Timestamp timestamp_us, Transform T_o0_ot)
    : timestamp_us_(timestamp_us), T_o0_ot_(T_o0_ot) {}

const Transform& OdomTformData::getTransform() const { return T_o0_ot_; }

const Timestamp& OdomTformData::getTimestamp() const { return timestamp_us_; }

void Odom::addTransformData(const Timestamp& timestamp_us, const Transform& T) {
  data_.emplace_back(timestamp_us, T);
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t* match_idx) const {
  size_t idx = start_idx;

  while ((idx < (data_.size() - 1)) &&
         (timestamp_us > data_[idx].getTimestamp())) {
    ++idx;
  }
  if (idx > 0) {
    --idx;
  }

  if (match_idx != nullptr) {
    *match_idx = idx;
  }

  // interpolate
  double t_diff_ratio =
      static_cast<double>(timestamp_us - data_[idx].getTimestamp()) /
      static_cast<double>(data_[idx + 1].getTimestamp() -
                          data_[idx].getTimestamp());

  Transform::Vector6 diff_vector =
      (data_[idx].getTransform().inverse() * data_[idx + 1].getTransform())
          .log();
  Transform out =
      data_[idx].getTransform() * Transform::exp(t_diff_ratio * diff_vector);

  return out;
}

const size_t Odom::getNumberOfOdoms() const { return data_.size(); }

Scan::Scan(const Pointcloud& in, Timestamp &time, const Config& config)
    : timestamp_us_(time), odom_transform_set_(false) {
  static int kTargetNum = 6000;
  std::default_random_engine generator(time);
  std::uniform_real_distribution<float> distribution(0, 1);

  float keep_points_ratio = std::min(1.0f, (float(kTargetNum) / (in.size() + 1)));
  for (const Point& point : in) {
    if (distribution(generator) < keep_points_ratio) {
        Point store_point;
        store_point.x = point.x;
        store_point.y = point.y;
        store_point.z = point.z;
        store_point.intensity = 0;
        raw_points_.push_back(store_point);
    }
  }
}

Scan::Config Scan::getConfig() {
  Scan::Config config;
  return config;
}

void Scan::setOdomTransform(const Odom& odom, const double time_offset,
                            const size_t start_idx, size_t* match_idx) {

  // NOTE: This static cast is really really important. Without it the
  // timestamp_us will be cast to a float, as it is a very large number it
  // will have quite low precision and when it is cast back to a long int
  // will be a very different value (about 2 to 3 million lower in some
  // quick tests). This difference will then break everything.
  Timestamp point_ts_us = timestamp_us_ +
                          static_cast<Timestamp>(1000000.0 * time_offset);

  T_o0_ot_ = odom.getOdomTransform(point_ts_us, start_idx, match_idx);

  odom_transform_set_ = true;
}

void Scan::getTimeAlignedPointcloud(const Transform& T_o_l,
                                    Pointcloud* pointcloud) const {
  for (size_t i = 0; i < raw_points_.size(); ++i) {
    Transform T_o_lt = T_o0_ot_ * T_o_l;

    Eigen::Affine3f pcl_transform;

    pcl_transform.matrix() = T_o_lt.matrix();
    pointcloud->push_back(pcl::transformPoint(raw_points_[i], pcl_transform));
  }
}

const size_t Lidar::getNumberOfScans() const { return scans_.size(); }

void Lidar::addPointcloud(const std::vector<float> &pointcloud, Timestamp time,
                          const Scan::Config& config) {
  Pointcloud cloud;
  for (size_t i = 0; i < pointcloud.size() / 3; i++) {
    Point point;
    point.x = pointcloud[i * 3 + 0];
    point.y = pointcloud[i * 3 + 1];
    point.z = pointcloud[i * 3 + 2];
    point.intensity = 0;
    cloud.push_back(point);
  }
  scans_.emplace_back(cloud, time, config);
}

void Lidar::getCombinedPointcloud(Pointcloud* pointcloud) const {
  for (const Scan& scan : scans_) {
    scan.getTimeAlignedPointcloud(getOdomLidarTransform(), pointcloud);
  }
}

void Lidar::setOdomOdomTransforms(const Odom& odom, const double time_offset) {
  size_t idx = 0;
  for (Scan& scan : scans_) {
    scan.setOdomTransform(odom, time_offset, idx, &idx);
  }
}

void Lidar::setOdomLidarTransform(const Transform& T_o_l) { T_o_l_ = T_o_l; }

void Lidar::setBestOdomLidarTransform(const Transform& T_o_l) { bestT_o_l_ = T_o_l; }

const Transform& Lidar::getOdomLidarTransform() const { return T_o_l_; }

const Transform& Lidar::getBestOdomLidarTransform() const { return bestT_o_l_; }