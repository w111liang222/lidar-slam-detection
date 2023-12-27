#include "calib_lidar_imu.h"
#include <utility>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "Logger.h"

using namespace hdl_graph_slam;

CalibLidarImu::CalibLidarImu()
{
    imu_buffer_.clear();
    no_rtk_flag = false;

    // init downsample object
    downer_.setLeafSize(0.2, 0.2, 0.2);

// init register object
#ifdef HAVE_CUDA_ENABLE
        register_ = select_registration_method("FAST_VGICP_CUDA");
#else
        register_ = select_registration_method("FAST_VGICP");
#endif
}

CalibLidarImu::~CalibLidarImu()
{
}

Eigen::Matrix4d CalibLidarImu::get_lidar_T_R(const LidarData &data)
{
    // if no rtk singal, the flag set true
    no_rtk_flag =true;

    if (!data.cloud || data.cloud->size() == 0)
    {
        LOG_WARN("no cloud in lidar data");
        return Eigen::Matrix4d::Identity();
    }

    if (!register_)
    {
        LOG_WARN("register no initialize !!!");
        return Eigen::Matrix4d::Identity();
    }

    //downsample lidar cloud for save align time
    CloudT::Ptr downed_cloud(new CloudT);
    downer_.setInputCloud(data.cloud);
    downer_.filter(*downed_cloud);

    if (!local_map_)
    {
        local_map_.reset(new CloudT);
        *local_map_ += *(data.cloud);

        register_->setInputTarget(local_map_);
        last_frame_pose_ = Eigen::Matrix4d::Identity();

        return Eigen::Matrix4d::Identity();
    }

    // get transform between frame and local map
    register_->setInputSource(downed_cloud);
    CloudT::Ptr aligned(new CloudT);
    register_->align(*aligned, last_frame_pose_.cast<float>());
    if (!register_->hasConverged())
    {
        LOG_WARN("register cant converge, please check initial value !!!");
        return Eigen::Matrix4d::Identity();
    }
    Eigen::Matrix4d T_l_m = (register_->getFinalTransformation()).cast<double>();
    last_frame_pose_ = T_l_m;

    return T_l_m;
}

void CalibLidarImu::addLidarData(const LidarData &data, Eigen::Matrix4d &global_T, Eigen::Matrix4d &delta_T)
{
    LidarFrame frame;
    frame.stamp = data.stamp;
    frame.T = delta_T;
    frame.gT = global_T;
    frame.cloud = data.cloud;
    lidar_buffer_.push_back(move(frame));

    //update local_map_
    if (no_rtk_flag)
    {
        CloudT::Ptr key_lidar_frame(new CloudT());
        pcl::transformPointCloud(*(data.cloud), *key_lidar_frame, global_T);

        // downsample key_lidar_frame
        downer_.setInputCloud(key_lidar_frame);
        downer_.filter(*key_lidar_frame);

        // add key lidar data to update local_map_
        *local_map_ += *key_lidar_frame;

        // downsample local map for save align time
        CloudT::Ptr downed_map(new CloudT());
        downer_.setInputCloud(local_map_);
        downer_.filter(*downed_map);
        register_->setInputTarget(downed_map);
    }
}

void CalibLidarImu::addImuData(const ImuData &data)
{
    imu_buffer_.push_back(data);
}

Eigen::Quaterniond CalibLidarImu::getInterpolatedAttitude(const Eigen::Quaterniond &q_s_w, const Eigen::Quaterniond &q_e_w, double scale)
{
    if (0 == scale || scale > 1)
        return move(Eigen::Quaterniond().Identity());

    // calculate angleaxis difference
    Eigen::Quaterniond q_e_s = q_s_w.inverse() * q_e_w;
    q_e_s.normalize();
    Eigen::AngleAxisd diff_angle_axis(q_e_s);

    // interpolated attitude by scale
    double interpolated_angle = diff_angle_axis.angle() * scale;
    Eigen::Quaterniond q_ie_s(Eigen::AngleAxisd(interpolated_angle, diff_angle_axis.axis()).toRotationMatrix());
    Eigen::Quaterniond q_ie_w = q_s_w * q_ie_s;
    q_ie_w.normalize();

    return move(q_ie_w);
}

Eigen::Quaterniond CalibLidarImu::solve(const vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> &corres)
{
    if (corres.size() == 0)
    {
        LOG_WARN("Imu calibration has no constraint");
        return move(Eigen::Quaterniond().Identity());
    }

    LOG_INFO("Imu calibration constraints size {}", corres.size());

    // transform quaternion to skew symmetric matrix
    auto toSkewSymmetric = [](const Eigen::Vector3d &q) -> Eigen::Matrix3d {
        Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
        mat(0, 1) = -q.z();
        mat(0, 2) = q.y();
        mat(1, 0) = q.z();
        mat(1, 2) = -q.x();
        mat(2, 0) = -q.y();
        mat(2, 1) = q.x();

        return move(mat);
    };

    // create homogeneous linear equations
    Eigen::MatrixXd A(corres.size() * 4, 4);
    for (int i = 0; i < corres.size(); i++)
    {
        // get relative transform
        const auto &q_l2_l1 = corres[i].first;
        const auto &q_b2_b1 = corres[i].second;

        // get left product matrix
        Eigen::Vector3d q_b2_b1_vec = q_b2_b1.vec();
        Eigen::Matrix4d left_Q_b2_b1 = Eigen::Matrix4d::Zero();
        left_Q_b2_b1.block<1, 3>(0, 1) = -q_b2_b1_vec.transpose();
        left_Q_b2_b1.block<3, 1>(1, 0) = q_b2_b1_vec;
        left_Q_b2_b1.block<3, 3>(1, 1) = toSkewSymmetric(q_b2_b1_vec);
        left_Q_b2_b1 += q_b2_b1.w() * Eigen::Matrix4d::Identity();

        // get right product matrix
        Eigen::Vector3d q_l2_l1_vec = q_l2_l1.vec();
        Eigen::Matrix4d right_Q_l2_l1 = Eigen::Matrix4d::Zero();
        right_Q_l2_l1.block<1, 3>(0, 1) = -q_l2_l1_vec.transpose();
        right_Q_l2_l1.block<3, 1>(1, 0) = q_l2_l1_vec;
        right_Q_l2_l1.block<3, 3>(1, 1) = -toSkewSymmetric(q_l2_l1_vec);
        right_Q_l2_l1 += q_l2_l1.w() * Eigen::Matrix4d::Identity();

        // add loss function
        double angle_distance = 180.0 / M_PI * q_b2_b1.angularDistance(q_l2_l1);
        double huber = angle_distance > 2.0 ? 2.0 / angle_distance : 1.0;

        A.block<4, 4>(i * 4, 0) = huber * (left_Q_b2_b1 - right_Q_l2_l1);
    }

    // solve homogeneous linear equations by svd method
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
    Eigen::Quaterniond q_l_b(x(0), x(1), x(2), x(3));

    LOG_INFO("Imu calibration solve SVD done");
    return move(q_l_b);
}

#ifdef do_optimize
void CalibLidarImu::optimize()
{
    if (aligned_lidar_imu_buffer_.size() == 0 || !register_)
    {
        cout << "no aligned data or register !!!" << endl;
        return;
    }

    // clear local map and initialize
    if (local_map_)
        local_map_->clear();
    else
        local_map_.reset(new CloudT);
    *local_map_ += *(aligned_lidar_imu_buffer_[0].first.cloud);

    // use scan2match with estimated initial value to update lidar frame
    for (int i = 1; i < aligned_lidar_imu_buffer_.size(); i++)
    {
        // get front and back frames
        auto &aligned1 = aligned_lidar_imu_buffer_[i - 1];
        auto &aligned2 = aligned_lidar_imu_buffer_[i];

        // downsample local map and lidar cloud for save align time
        CloudT::Ptr downed_map(new CloudT);
        downer_.setInputCloud(local_map_);
        downer_.filter(*downed_map);

        // calculate estimated T_l_m
        Eigen::Matrix3d R_l1_m = aligned1.first.gT.block<3, 3>(0, 0);
        Eigen::Quaterniond q_b1_w = aligned1.second;
        Eigen::Quaterniond q_b2_w = aligned2.second;
        Eigen::Quaterniond est_q_b2_b1 = q_b1_w.inverse() * q_b2_w;
        Eigen::Matrix3d est_R_l2_l1 = Eigen::Matrix3d(q_l_b_.inverse() * est_q_b2_b1 * q_l_b_);
        Eigen::Matrix3d est_R_l2_m = R_l1_m * est_R_l2_l1;
        Eigen::Matrix4d est_T_l2_m = Eigen::Matrix4d::Identity();
        est_T_l2_m.block<3, 3>(0, 0) = est_R_l2_m;
        est_T_l2_m.block<3, 1>(0, 3) = aligned2.first.gT.block<3, 1>(0, 3);

        // register
        register_->setInputSource(aligned2.first.cloud);
        register_->setInputTarget(downed_map);
        CloudT::Ptr aligned(new CloudT);
        register_->align(*aligned, est_T_l2_m.cast<float>());
        if (!register_->hasConverged())
        {
            cout << "register cant converge, please check initial value !!!" << endl;
            return;
        }
        Eigen::Matrix4d T_l2_m = (register_->getFinalTransformation()).cast<double>();

        // update lidar frame
        aligned2.first.gT = T_l2_m;
        Eigen::Matrix4d T_l1_m = aligned1.first.gT;
        aligned2.first.T = T_l1_m.inverse() * T_l2_m;

        // update local map
        *local_map_ += *aligned;
    }

    // generate constraints
    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres;
    for (int i = 1; i < aligned_lidar_imu_buffer_.size(); i++)
    {
        // get neighbor frame
        const auto &aligned1 = aligned_lidar_imu_buffer_[i - 1];
        const auto &aligned2 = aligned_lidar_imu_buffer_[i];

        // calculate relative transform between neighbor lidar
        Eigen::Quaterniond q_l2_l1 = Eigen::Quaterniond(aligned2.first.T.block<3, 3>(0, 0));

        // calculate relative transform between neighbor interpolated imu
        Eigen::Quaterniond q_b1_w = aligned1.second;
        Eigen::Quaterniond q_b2_w = aligned2.second;
        Eigen::Quaterniond q_b2_b1 = q_b1_w.inverse() * q_b2_w;

        corres.push_back(move(pair<Eigen::Quaterniond, Eigen::Quaterniond>(q_l2_l1, q_b2_b1)));
        corres2_ = corres;
    }

    Eigen::Quaterniond result = solve(corres);
    result.normalize();

    // check whether optimize fail
    double angle = fabs(q_l_b_.angularDistance(result));
    if (angle > 0.5236)
    {
       cout << "the difference between before and after optimze is " << angle << " which greater than given threshold 0.5236 !!!" << endl;
       return;
    }
    q_l_b_ = result;
}
#endif

Eigen::Matrix3d CalibLidarImu::calib(bool integration)
{
    // integration rotation of imu, when raw imu attitude has big error
    if (integration)
    {
        imu_buffer_[0].rot = Eigen::Quaterniond::Identity();
        for (int i = 1; i < imu_buffer_.size(); i++)
        {
            Eigen::Vector3d bar_gyr = 0.5 * (imu_buffer_[i - 1].gyr + imu_buffer_[i].gyr);
            Eigen::Vector3d angle_inc = bar_gyr * (imu_buffer_[i].stamp - imu_buffer_[i - 1].stamp);
            Eigen::Quaterniond rot_inc = Eigen::Quaterniond(1.0, 0.5 * angle_inc[0], 0.5 * angle_inc[1], 0.5 * angle_inc[2]);
            imu_buffer_[i].rot = imu_buffer_[i - 1].rot * rot_inc;
        }
    }

    // move invalid lidar frame which got before first imu frame
    auto invalid_lidar_it = lidar_buffer_.begin();
    for (; invalid_lidar_it != lidar_buffer_.end(); invalid_lidar_it++)
    {
        if (invalid_lidar_it->stamp >= imu_buffer_[0].stamp)
            break;
    }
    if (invalid_lidar_it != lidar_buffer_.begin())
        lidar_buffer_.erase(lidar_buffer_.begin(), invalid_lidar_it);
    if (lidar_buffer_.size() == 0)
    {
        return Eigen::Matrix3d();
    }

    // get time-aligned lidar odometry rotation and imu integration rotation
    auto last_imu_it = imu_buffer_.begin();
    for (int i = 0; i < lidar_buffer_.size(); i++)
    {
        // get lidar information
        const auto &lidar_frame = lidar_buffer_[i];

        // get last imu frame which before current lidar frame
        for (; last_imu_it != imu_buffer_.end(); last_imu_it++)
        {
            if (last_imu_it->stamp >= lidar_frame.stamp)
                break;
        }
        if (last_imu_it != imu_buffer_.begin())
            last_imu_it--;

        // get interpolated imu attitude at lidar stamp
        auto imu_it1 = last_imu_it;
        auto imu_it2 = last_imu_it + 1;
        if (imu_buffer_.end() == imu_it2)
            break;
        assert(imu_it2->stamp >= lidar_frame.stamp || imu_it1->stamp < imu_it2->stamp); // this shouldnt happen
        Eigen::Quaterniond q_b1_w = imu_it1->rot;
        Eigen::Quaterniond q_b2_w = imu_it2->rot;
        double scale = (lidar_frame.stamp - imu_it1->stamp) / (imu_it2->stamp - imu_it1->stamp);
        Eigen::Quaterniond q_inter_w = getInterpolatedAttitude(q_b1_w, q_b2_w, scale);

        // buffer aligned information
        aligned_lidar_imu_buffer_.push_back(move(pair<LidarFrame, Eigen::Quaterniond>(lidar_frame, q_inter_w)));
    }

    // solve initial transform between lidar and imu
    vector<pair<Eigen::Quaterniond, Eigen::Quaterniond>> corres(0);
    for (int i = 1; i < aligned_lidar_imu_buffer_.size(); i++)
    {
        // get neighbor aligned frame
        const auto &aligned1 = aligned_lidar_imu_buffer_[i - 1];
        const auto &aligned2 = aligned_lidar_imu_buffer_[i];

        // get initial relative transform between neighbor lidar
        Eigen::Quaterniond q_l2_l1 = Eigen::Quaterniond(aligned_lidar_imu_buffer_[i].first.T.block<3, 3>(0, 0));

        // calculate relative transform between neighbor interpolated imu
        Eigen::Quaterniond q_b1_w = aligned1.second;
        Eigen::Quaterniond q_b2_w = aligned2.second;
        Eigen::Quaterniond q_b2_b1 = q_b1_w.inverse() * q_b2_w;

        corres.push_back(move(pair<Eigen::Quaterniond, Eigen::Quaterniond>(q_l2_l1, q_b2_b1)));
        corres1_ = corres;
    }
    q_l_b_ = solve(corres);

    // if (no_rtk_flag)
    // {
    //     // optimize: use initial result to estimate transform between neighbor lidar frame for improving matching precise
    //     optimize();
    // }

    return q_l_b_.normalized().toRotationMatrix();
}

std::vector<Eigen::Matrix4d> CalibLidarImu::getLidarPose()
{
    std::vector<Eigen::Matrix4d> poses;
    Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
    for (int i = 1; i < aligned_lidar_imu_buffer_.size(); i++)
    {
        current_pose = current_pose * aligned_lidar_imu_buffer_[i].first.T;
        poses.push_back(current_pose);
    }
    return poses;
}

std::vector<Eigen::Matrix4d> CalibLidarImu::getImuPose()
{
    // q_l_b_ = Eigen::Quaterniond::Identity();

    std::vector<Eigen::Matrix4d> poses;
    Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
    for (int i = 1; i < aligned_lidar_imu_buffer_.size(); i++)
    {
        // get front and back frames
        auto &aligned1 = aligned_lidar_imu_buffer_[i - 1];
        auto &aligned2 = aligned_lidar_imu_buffer_[i];
        Eigen::Quaterniond q_b1_w = aligned1.second;
        Eigen::Quaterniond q_b2_w = aligned2.second;
        Eigen::Quaterniond est_q_b2_b1 = q_b1_w.inverse() * q_b2_w;
        Eigen::Matrix3d est_R_l2_l1 = Eigen::Matrix3d(q_l_b_.inverse() * est_q_b2_b1 * q_l_b_);

        Eigen::Matrix4d l2_l1_T = aligned_lidar_imu_buffer_[i].first.T;
        l2_l1_T.block<3, 3>(0, 0) = est_R_l2_l1;

        current_pose = current_pose * l2_l1_T;
        poses.push_back(current_pose);
    }
    return poses;
}