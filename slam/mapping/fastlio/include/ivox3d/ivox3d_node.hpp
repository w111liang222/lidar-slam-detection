#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

namespace faster_lio {

// squared distance of two pcl points
template <typename PointT>
inline double distance2(const PointT& pt1, const PointT& pt2) {
    Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
    return d.squaredNorm();
}

// convert from pcl point to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt) {
    return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZINormal>(const pcl::PointXYZINormal& pt) {
    return pt.getVector3fMap();
}

template <typename PointT, int dim = 3>
class IVoxNode {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct DistPoint;

    IVoxNode() = default;
    IVoxNode(const double& distance) : distance_(distance) {}

    void InsertPoint(const PointT& pt);

    inline bool Empty() const;

    inline std::size_t Size() const;

    inline PointT GetPoint(const std::size_t idx) const;

    inline const double GetDistance() const {
        return distance_;
    }

    inline const std::vector<PointType>& GetAllPoints() const {
        return points_;
    }

    int KNNPointByCondition(std::vector<DistPoint>& dis_points, const PointT& point, const int& K,
                            const double& max_range);

   private:
    double distance_;
    std::vector<PointT> points_;
};

template <typename PointT, int dim>
struct IVoxNode<PointT, dim>::DistPoint {
    double dist = 0;
    IVoxNode* node = nullptr;
    int idx = 0;

    DistPoint() = default;
    DistPoint(const double d, IVoxNode* n, const int i) : dist(d), node(n), idx(i) {}

    PointT Get() { return node->GetPoint(idx); }

    inline bool operator()(const DistPoint& p1, const DistPoint& p2) { return p1.dist < p2.dist; }

    inline bool operator<(const DistPoint& rhs) { return dist < rhs.dist; }
};

template <typename PointT, int dim>
void IVoxNode<PointT, dim>::InsertPoint(const PointT& pt) {
    points_.template emplace_back(pt);
}

template <typename PointT, int dim>
bool IVoxNode<PointT, dim>::Empty() const {
    return points_.empty();
}

template <typename PointT, int dim>
std::size_t IVoxNode<PointT, dim>::Size() const {
    return points_.size();
}

template <typename PointT, int dim>
PointT IVoxNode<PointT, dim>::GetPoint(const std::size_t idx) const {
    return points_[idx];
}

template <typename PointT, int dim>
int IVoxNode<PointT, dim>::KNNPointByCondition(std::vector<DistPoint>& dis_points, const PointT& point, const int& K,
                                               const double& max_square_range) {
    std::size_t old_size = dis_points.size();

    for (const auto& pt : points_) {
        double d = distance2(pt, point);
        if (d < max_square_range) {
            dis_points.template emplace_back(DistPoint(d, this, &pt - points_.data()));
        }
    }

    // sort by distance
    if (old_size + K >= dis_points.size()) {
    } else {
        std::nth_element(dis_points.begin() + old_size, dis_points.begin() + old_size + K - 1, dis_points.end());
        dis_points.resize(old_size + K);
    }

    return dis_points.size();
}

}  // namespace faster_lio
