//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <execution>
#include <list>
#include <thread>

#include "eigen_types.h"
#include "ivox3d_node.hpp"

namespace faster_lio {

enum class IVoxNodeType {
    DEFAULT,  // linear ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6,
        NEARBY18,
        NEARBY26,
        NEARBY74,
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
        double max_distance_ = 100.0;                   // d_distance > max_distance_ will be erased
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    void SetNearByType(const NearbyType& nearby_type) {
        options_.nearby_type_ = nearby_type;
        GenerateNearbyGrids();
    }

    NearbyType GetNearByType() const {
        return options_.nearby_type_;
    }

    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add, const double& distance);

    /// get nn
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_square_range = 5.0);

    /// get nn in cloud
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /// get number of points
    size_t NumPoints() const;

    /// get number of valid grids
    size_t NumValidGrids() const;

    // get all points
    void GetAllPoints(PointVector& points_out) const;

    /// get statistics of the points
    std::vector<float> StatGridPoints() const;

   private:
    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, hash_vec<dim>>
        grids_map_;                                        // voxel hash map
    std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
    std::vector<KeyType> nearby_grids_;                    // nearbys
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_square_range) {
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(ToEigen<float, dim>(pt));

    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            auto tmp = iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_square_range);
        }
    }

    if (candidates.empty()) {
        return false;
    }

    if (candidates.size() <= max_num) {
    } else {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

    closest_pt.clear();
    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY74) {
        for (int i = -2; i <= 2; i++) {
            for (int j = -2; j <= 2; j++) {
                for (int k = -1; k <= 1; k++) {
                    nearby_grids_.emplace_back(KeyType(i, j, k));
                }
            }
        }
    } else {
        nearby_grids_ = {};
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
    std::vector<size_t> index(cloud.size());
    for (int i = 0; i < cloud.size(); ++i) {
        index[i] = i;
    }
    closest_cloud.resize(cloud.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloud, &closest_cloud, this](size_t idx) {
        PointType pt;
        if (GetClosestPoint(cloud[idx], pt)) {
            closest_cloud[idx] = pt;
        } else {
            closest_cloud[idx] = PointType();
        }
    });
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add, const double& distance) {
    std::for_each(std::execution::unseq, points_to_add.begin(), points_to_add.end(), [&](const auto& pt) {
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            grids_cache_.push_front({key, NodeType(distance)});
            grids_map_.insert({key, grids_cache_.begin()});

            grids_cache_.front().second.InsertPoint(pt);
        } else {
            iter->second->second.InsertPoint(pt);
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();
        }

        if (grids_map_.size() > options_.capacity_ && (distance - grids_cache_.back().second.GetDistance()) > options_.max_distance_) {
            grids_map_.erase(grids_cache_.back().first);
            grids_cache_.pop_back();
        }
    });
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().round().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GetAllPoints(PointVector& points_out) const {
    for (auto it = grids_map_.begin(); it != grids_map_.end(); it++) {
        const auto& points = it->second->second.GetAllPoints();
        points_out.insert(points_out.end(), points.begin(), points.end());
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
    int num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
    int sum = 0, sum_square = 0;
    for (auto& it : grids_cache_) {
        int s = it.second.Size();
        valid_num += s > 0;
        max = s > max ? s : max;
        min = s < min ? s : min;
        sum += s;
        sum_square += s * s;
    }
    float ave = float(sum) / num;
    float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;
    return std::vector<float>{valid_num, ave, max, min, stddev};
}

}  // namespace faster_lio

#endif
