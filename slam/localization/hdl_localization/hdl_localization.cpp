#include "hdl_localization.h"

using namespace Locate;

// hdl localization nodelet
void init_hdl_localization_node(InitParameter &param);
void deinit_hdl_localization_node();
void set_imu_extrinic_hdl_localization(Eigen::Matrix4d extrinic);
void set_initpose_hdl_localization(uint64_t stamp, const Eigen::Matrix4d& pose);
void set_map_hdl_localization(PointCloud::Ptr& cloud);
void enqueue_ins_hdl_localization(std::shared_ptr<RTKType> &ins);
void enqueue_imu_hdl_localization(ImuType& imu);
bool get_timed_pose_hdl_localization(uint64_t timestamp, Eigen::Matrix4d &pose);
bool get_timed_pose_hdl_localization(RTKType &ins, Eigen::Matrix4d &pose);
LocType enqueue_hdl_localization(PointCloudAttrPtr& cloud, ImageType& image, Eigen::Isometry3d& pose);

HdlLocalization::HdlLocalization() : LocalizationBase() {}

HdlLocalization::~HdlLocalization() {
    deinit_hdl_localization_node();
}

bool HdlLocalization::init(InitParameter &param) {
    Eigen::Matrix4d imu_extrinic = mImuStaticTrans * mStaticTrans.inverse();

    set_imu_extrinic_hdl_localization(imu_extrinic);
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

LocType HdlLocalization::localize(PointCloudAttrPtr& cloud, ImageType& image, Eigen::Isometry3d& pose) {
    return enqueue_hdl_localization(cloud, image, pose);
}

bool HdlLocalization::getTimedPose(uint64_t timestamp, Eigen::Matrix4d &pose) {
    return get_timed_pose_hdl_localization(timestamp, pose);
}

bool HdlLocalization::getTimedPose(RTKType &ins, Eigen::Matrix4d &pose) {
    return get_timed_pose_hdl_localization(ins, pose);
}
