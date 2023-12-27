#ifndef __SLAM_KEYFRAME_H
#define __SLAM_KEYFRAME_H

#include "mapping_types.h"

struct KeyFrame {
  public:
    KeyFrame(uint64_t stamp, long id, Eigen::Isometry3d odom, PointCloud::Ptr points);
    KeyFrame(long id, const std::string& data_path, bool is_load_graph);
    virtual ~KeyFrame();

    bool loadOdom();
    bool loadMeta();
    bool loadPcd();
    bool loadImage();
    void save(const std::string& directory);

    void downsample(double resolution);
    void transformPoints();
    cv::Mat imageColorDecode(std::string image_name);
    void computeDescriptor();

    long id() const;
  public:
    long mId;
    bool mIsLoadGraph;
    uint64_t mTimestamp;
    Eigen::Isometry3d mOdom;
    PointCloud::Ptr mPoints;
    PointCloud::Ptr mDownsamplePoints;
    PointCloud::Ptr mTransfromPoints;

    std::vector<std::string> mImageName;
    std::map<std::string, std::vector<char>> mImages;
    std::map<std::string, cv::Mat> mImageCV;

    std::string mDataPath;
    std::string mOdomPath;
    std::string mPcdPath;

    // features
    Eigen::MatrixXd mSc;
    Eigen::MatrixXd mRingkey;
    Eigen::MatrixXd mSectorkey;
};

#endif  // __SLAM_KEYFRAME_H