#include <hdl_localization/delta_estimater.hpp>

using namespace hdl_localization;

// fastlio nodelet
int  fast_lio_init(std::vector<double> &extT, std::vector<double>& extR, int filter_num, int max_point_num, bool undistort);
void fastlio_imu_enqueue(ImuType &imu);
void fastlio_pcl_enqueue(PointCloudAttrPtr &points, bool sync);
bool fastlio_main();
std::vector<double> get_fastlio_odom();

DeltaEstimater::DeltaEstimater(): mLastodom(Eigen::Matrix4d::Identity()) {

}

DeltaEstimater::~DeltaEstimater() {
    mThreadStart = false;
    mLioThread->join();
    mLioThread.reset(nullptr);
}

void DeltaEstimater::init(Eigen::Matrix4d &extrinic) {
    mImuInsStaticTrans = extrinic;
    std::vector<double> extrinsicT = {mImuInsStaticTrans(0, 3), mImuInsStaticTrans(1, 3), mImuInsStaticTrans(2, 3)};
    std::vector<double> extrinsicR = {mImuInsStaticTrans(0, 0), mImuInsStaticTrans(0, 1), mImuInsStaticTrans(0, 2),
                                      mImuInsStaticTrans(1, 0), mImuInsStaticTrans(1, 1), mImuInsStaticTrans(1, 2),
                                      mImuInsStaticTrans(2, 0), mImuInsStaticTrans(2, 1), mImuInsStaticTrans(2, 2)};
    fast_lio_init(extrinsicT, extrinsicR, 1, 10000, false);

    mThreadStart = true;
    mLioThread.reset(new std::thread(&DeltaEstimater::runLio, this));
}

void DeltaEstimater::feedImuData(ImuType &imu) {
    fastlio_imu_enqueue(imu);
}

void DeltaEstimater::feedPointData(PointCloudAttrPtr &points) {
    fastlio_pcl_enqueue(points, true);
}

bool DeltaEstimater::getDeltaOdom(Eigen::Matrix4d &delta) {
    return mOdomQueue.wait_dequeue_timed(delta, 1000000);
}

void DeltaEstimater::runLio() {
    while (mThreadStart) {
        if (fastlio_main()) {
            std::vector<double> state = get_fastlio_odom();

            Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
            odom.topRightCorner<3, 1>() = Eigen::Vector3d(state[0], state[1], state[2]);
            odom.topLeftCorner<3, 3>()  = Eigen::Quaterniond(state[6], state[3], state[4], state[5]).normalized().toRotationMatrix();
            odom = mImuInsStaticTrans.inverse() * odom * mImuInsStaticTrans;

            Eigen::Matrix4d delta_odom = mLastodom.inverse() * odom;
            mLastodom = odom;
            mOdomQueue.enqueue(delta_odom);
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}