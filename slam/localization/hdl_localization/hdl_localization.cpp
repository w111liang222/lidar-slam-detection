#include "hdl_localization.h"

using namespace Locate;

// hdl localization nodelet
void init_hdl_localization_node(InitParameter &param);
void deinit_hdl_localization_node();
void set_imu_extrinic_hdl_localization(Eigen::Matrix4d extrinic);
void set_camera_param_hdl_localization(const CamParamType &param);
void set_initpose_hdl_localization(uint64_t stamp, const Eigen::Matrix4d& pose);
void set_map_hdl_localization(PointCloud::Ptr& cloud);
void enqueue_ins_hdl_localization(std::shared_ptr<RTKType> &ins);
void enqueue_imu_hdl_localization(ImuType& imu);
bool get_timed_pose_hdl_localization(uint64_t timestamp, Eigen::Matrix4d &pose);
bool get_timed_pose_hdl_localization(RTKType &ins, Eigen::Matrix4d &pose);
void get_color_map_hdl_localization(PointCloudRGB::Ptr &points);
LocType enqueue_hdl_localization(PointCloudAttrPtr& cloud, cv::Mat& image, Eigen::Isometry3d& pose);

HdlLocalization::HdlLocalization(const std::string &imageName) : LocalizationBase() {
    mImageName = imageName;
}

HdlLocalization::~HdlLocalization() {
    deinit_hdl_localization_node();
}

bool HdlLocalization::init(InitParameter &param) {
    Eigen::Matrix4d imu_extrinic = mImuStaticTrans * mStaticTrans.inverse();
    CamParamType camera_param = CamParamType();
    if (mCameraParam.find(mImageName) != mCameraParam.end()) {
        camera_param = mCameraParam[mImageName];
        camera_param.staticTrans = camera_param.staticTrans * mStaticTrans.inverse();
    }

    set_imu_extrinic_hdl_localization(imu_extrinic);
    set_camera_param_hdl_localization(camera_param);
    init_hdl_localization_node(param);
    return true;
}

void HdlLocalization::setInitPose(uint64_t stamp, const Eigen::Matrix4d& pose) {
    set_initpose_hdl_localization(stamp, pose);
}

void HdlLocalization::updateLocalMap(PointCloud::Ptr& cloud) {
    set_map_hdl_localization(cloud);
}

void HdlLocalization::feedInsData(std::shared_ptr<RTKType> ins) {
    enqueue_ins_hdl_localization(ins);
}

void HdlLocalization::feedImuData(ImuType &imu) {
    enqueue_imu_hdl_localization(imu);
}

LocType HdlLocalization::localize(PointCloudAttrPtr& cloud, cv::Mat& image, Eigen::Isometry3d& pose) {
    return enqueue_hdl_localization(cloud, image, pose);
}

bool HdlLocalization::getTimedPose(uint64_t timestamp, Eigen::Matrix4d &pose) {
    return get_timed_pose_hdl_localization(timestamp, pose);
}

bool HdlLocalization::getTimedPose(RTKType &ins, Eigen::Matrix4d &pose) {
    return get_timed_pose_hdl_localization(ins, pose);
}

void HdlLocalization::getColorMap(PointCloudRGB::Ptr &points) {
    get_color_map_hdl_localization(points);
};