// SPDX-License-Identifier: BSD-2-Clause

#ifndef OVERLAP_REMOVAL_HPP
#define OVERLAP_REMOVAL_HPP

#include "slam_base.h"
#include "Logger.h"

class OverlapRemover {
public:

  /**
   * @brief constructor
   */
  OverlapRemover() {
    distance_thresh = 4.0;
    timestamp_thresh = 10000000;
  }

  std::vector<std::shared_ptr<KeyFrame>> detect(const std::vector<std::shared_ptr<KeyFrame>> &keyframes) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pose_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pose_cloud->width = static_cast<int>(keyframes.size());
    pose_cloud->height = 1;
    pose_cloud->points.resize(pose_cloud->width * pose_cloud->height);
    for (size_t i = 0; i < keyframes.size(); i++) {
      pose_cloud->points[i].x = keyframes[i]->mOdom(0, 3);
      pose_cloud->points[i].y = keyframes[i]->mOdom(1, 3);
      pose_cloud->points[i].z = keyframes[i]->mOdom(2, 3);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pose_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    pose_kdtree->setInputCloud(pose_cloud);

    std::map<uint64_t, std::shared_ptr<KeyFrame>> stamp_frames;
    for(size_t i = 0; i < keyframes.size(); i++) {
      stamp_frames[keyframes[i]->mTimestamp] = keyframes[i];
    }

    std::map<int, std::shared_ptr<KeyFrame>> duplicate_map;
    for(auto it = stamp_frames.rbegin(); it != stamp_frames.rend(); it++) {
      if (duplicate_map.find(it->second->mId) != duplicate_map.end()) {
        continue;
      }
      pcl::PointXYZ pose;
      pose.x = it->second->mOdom(0, 3);
      pose.y = it->second->mOdom(1, 3);
      pose.z = it->second->mOdom(2, 3);

      std::vector<int> pointIdx(1);
      std::vector<float> pointDistance(1);
      if (pose_kdtree->radiusSearch(pose, distance_thresh, pointIdx, pointDistance) > 1) {
        for(size_t j = 1; j < pointIdx.size(); j++) {
          int64_t timestamp_elapse = int64_t(it->first) - int64_t(keyframes[pointIdx[j]]->mTimestamp);
          if (timestamp_elapse > timestamp_thresh) {
            duplicate_map[keyframes[pointIdx[j]]->mId] = keyframes[pointIdx[j]];
          }
        }
      }
    }

    std::vector<std::shared_ptr<KeyFrame>> duplicate_frames;
    for(auto frame : duplicate_map) {
      duplicate_frames.push_back(frame.second);
    }
    return duplicate_frames;
  }

  std::vector<std::shared_ptr<KeyFrame>> remove(const std::vector<std::shared_ptr<KeyFrame>> &keyframes, const std::vector<std::shared_ptr<KeyFrame>> &duplicate_frames) {
    std::map<long, std::shared_ptr<KeyFrame>> stamp_frames;
    for(size_t i = 0; i < keyframes.size(); i++) {
      stamp_frames[keyframes[i]->mId] = keyframes[i];
    }

    for(size_t i = 0; i < duplicate_frames.size(); i++) {
      stamp_frames.erase(duplicate_frames[i]->mId);
    }

    std::vector<std::shared_ptr<KeyFrame>> remain_frames;
    for(auto frame : stamp_frames) {
      remain_frames.push_back(frame.second);
    }
    return remain_frames;
  }

private:
  double distance_thresh;
  int64_t timestamp_thresh;
};

#endif  // OVERLAP_REMOVAL_HPP
