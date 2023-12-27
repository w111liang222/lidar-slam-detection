#include "visual_odometry.h"
#include <pcl/common/transforms.h>

#include "slam_utils.h"

const bool EstimateIntrinsic = false;
const bool EstimateL2CExtrinsic = true;
const int  EsikfIterTimes = 5;

void setInitialStateCov(StatesGroup &state) {
    // Set cov
    state.cov = state.cov.setIdentity() * INIT_COV;
    state.cov.block( 0, 0, 3, 3 ) = mat_3_3::Identity() * 1e-4;   // R
    state.cov.block( 3, 3, 3, 3 ) = mat_3_3::Identity() * 1e-4;   // T
    state.cov.block( 6, 6, 3, 3 ) = mat_3_3::Identity() * 1e-4;   // vel
    state.cov.block( 9, 9, 3, 3 ) = mat_3_3::Identity() * 1e-3;   // bias_g
    state.cov.block( 12, 12, 3, 3 ) = mat_3_3::Identity() * 1e-1; // bias_a
    state.cov.block( 15, 15, 3, 3 ) = mat_3_3::Identity() * 1e-5; // Gravity
    state.cov( 24, 24 ) = 1e-4;
    state.cov.block( 18, 18, 6, 6 ) = state.cov.block( 18, 18, 6, 6 ).setIdentity() *  1e-3; // Extrinsic between camera and IMU.
    state.cov.block( 25, 25, 4, 4 ) = state.cov.block( 25, 25, 4, 4 ).setIdentity() *  1e-3; // Camera intrinsic.
}

void VisualOdometry::setInitialCameraParameter(StatesGroup &state) {
    Eigen::Matrix4d ext_i2c = mCameraParam.staticTrans.inverse();
    state.rot_ext_i2c = ext_i2c.topLeftCorner<3, 3>();
    state.pos_ext_i2c = ext_i2c.topRightCorner<3, 1>();

    mInitialRotExtrinicL2c = state.rot_ext_i2c;
    mInitialPosExtrinicL2c = state.pos_ext_i2c;
    state.cam_intrinsic( 0 ) = mCameraParam.K.at<float>(0, 0);
    state.cam_intrinsic( 1 ) = mCameraParam.K.at<float>(1, 1);
    state.cam_intrinsic( 2 ) = mCameraParam.K.at<float>(0, 2);
    state.cam_intrinsic( 3 ) = mCameraParam.K.at<float>(1, 2);
    setInitialStateCov( state );
}

void VisualOdometry::setImagePose(std::shared_ptr<Image_frame> &image_pose, const StatesGroup &state) {
    mat_3_3 rot_mat = state.rot_end;
    vec_3   t_vec = state.pos_end;
    vec_3   pose_t = rot_mat * state.pos_ext_i2c + t_vec;
    mat_3_3 R_w2c = rot_mat * state.rot_ext_i2c;

    image_pose->set_pose( eigen_q( R_w2c ), pose_t );
    image_pose->fx = state.cam_intrinsic( 0 );
    image_pose->fy = state.cam_intrinsic( 1 );
    image_pose->cx = state.cam_intrinsic( 2 );
    image_pose->cy = state.cam_intrinsic( 3 );

    image_pose->m_cam_K << image_pose->fx, 0, image_pose->cx, 0, image_pose->fy, image_pose->cy, 0, 0, 1;
}

double get_huber_loss_scale( double reprojection_error, double outlier_threshold = 1.0 )
{
    // http://ceres-solver.org/nnls_modeling.html#lossfunction
    double scale = 1.0;
    if ( reprojection_error / outlier_threshold < 1.0 )
    {
        scale = 1.0;
    }
    else
    {
        scale = ( 2 * sqrt( reprojection_error ) / sqrt( outlier_threshold ) - 1.0 ) / reprojection_error;
    }
    return scale;
}

const int minimum_iteration_pts = 10;
bool      VisualOdometry::vio_esikf( StatesGroup &state_in, Rgbmap_tracker &op_track )
{
    StatesGroup state_iter = state_in;
    if ( !EstimateIntrinsic ) // When disable the online intrinsic calibration.
    {
        state_iter.cam_intrinsic << mCamK( 0, 0 ), mCamK( 1, 1 ), mCamK( 0, 2 ), mCamK( 1, 2 );
    }

    if ( !EstimateL2CExtrinsic )
    {
        state_iter.pos_ext_i2c = mInitialPosExtrinicL2c;
        state_iter.rot_ext_i2c = mInitialRotExtrinicL2c;
    }

    Eigen::Matrix< double, -1, -1 >                       H_mat;
    Eigen::Matrix< double, -1, 1 >                        meas_vec;
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > G, H_T_H, I_STATE;
    Eigen::Matrix< double, DIM_OF_STATES, 1 >             solution;
    Eigen::Matrix< double, -1, -1 >                       K, KH;
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > K_1;

    Eigen::SparseMatrix< double > H_mat_spa, H_T_H_spa, K_spa, KH_spa, vec_spa, I_STATE_spa;
    I_STATE.setIdentity();
    I_STATE_spa = I_STATE.sparseView();
    double fx, fy, cx, cy, time_td;

    int                   total_pt_size = op_track.m_map_rgb_pts_in_current_frame_pos.size();
    std::vector< double > last_reprojection_error_vec( total_pt_size ), current_reprojection_error_vec( total_pt_size );

    if ( total_pt_size < minimum_iteration_pts )
    {
        state_in = state_iter;
        return false;
    }
    H_mat.resize( total_pt_size * 2, DIM_OF_STATES );
    meas_vec.resize( total_pt_size * 2, 1 );
    double last_repro_err = 3e8;
    int    avail_pt_count = 0;
    double last_avr_repro_err = 0;

    double acc_reprojection_error = 0;
    double img_res_scale = 1.0;
    for ( int iter_count = 0; iter_count < EsikfIterTimes; iter_count++ )
    {
        mat_3_3 R_imu = state_iter.rot_end;
        vec_3   t_imu = state_iter.pos_end;
        vec_3   t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
        mat_3_3 R_c2w = R_imu * state_iter.rot_ext_i2c; // world to camera frame

        fx = state_iter.cam_intrinsic( 0 );
        fy = state_iter.cam_intrinsic( 1 );
        cx = state_iter.cam_intrinsic( 2 );
        cy = state_iter.cam_intrinsic( 3 );
        time_td = state_iter.td_ext_i2c_delta;

        vec_3   t_w2c = -R_c2w.transpose() * t_c2w;
        mat_3_3 R_w2c = R_c2w.transpose();
        int     pt_idx = -1;
        acc_reprojection_error = 0;
        vec_3               pt_3d_w, pt_3d_cam;
        vec_2               pt_img_measure, pt_img_proj, pt_img_vel;
        eigen_mat_d< 2, 3 > mat_pre;
        eigen_mat_d< 3, 3 > mat_A, mat_B, mat_C, mat_D, pt_hat;
        H_mat.setZero();
        solution.setZero();
        meas_vec.setZero();
        avail_pt_count = 0;
        for ( auto it = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); it != op_track.m_map_rgb_pts_in_last_frame_pos.end(); it++ )
        {
            pt_3d_w = ( ( RGB_pts * ) it->first )->get_pos();
            pt_img_vel = ( ( RGB_pts * ) it->first )->m_img_vel;
            pt_img_measure = vec_2( it->second.x, it->second.y );
            pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
            pt_img_proj = vec_2( fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ) + cx, fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 ) + cy ) + time_td * pt_img_vel;
            double repro_err = ( pt_img_proj - pt_img_measure ).norm();
            double huber_loss_scale = get_huber_loss_scale( repro_err );
            pt_idx++;
            acc_reprojection_error += repro_err;
            last_reprojection_error_vec[ pt_idx ] = repro_err;
            avail_pt_count++;
            // Appendix E of r2live_Supplementary_material.
            // https://github.com/hku-mars/r2live/blob/master/supply/r2live_Supplementary_material.pdf
            mat_pre << fx / pt_3d_cam( 2 ), 0, -fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ), 0, fy / pt_3d_cam( 2 ), -fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 );

            pt_hat = Sophus::SO3d::hat( ( R_imu.transpose() * ( pt_3d_w - t_imu ) ) );
            mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
            mat_B = -state_iter.rot_ext_i2c.transpose() * ( R_imu.transpose() );
            mat_C = Sophus::SO3d::hat( pt_3d_cam );
            mat_D = -state_iter.rot_ext_i2c.transpose();
            meas_vec.block( pt_idx * 2, 0, 2, 1 ) = ( pt_img_proj - pt_img_measure ) * huber_loss_scale / img_res_scale;

            H_mat.block( pt_idx * 2, 0, 2, 3 ) = mat_pre * mat_A * huber_loss_scale;
            H_mat.block( pt_idx * 2, 3, 2, 3 ) = mat_pre * mat_B * huber_loss_scale;
            if ( DIM_OF_STATES > 24 )
            {
                // Estimate time td.
                H_mat.block( pt_idx * 2, 24, 2, 1 ) = pt_img_vel * huber_loss_scale;
            }
            if ( EstimateL2CExtrinsic )
            {
                H_mat.block( pt_idx * 2, 18, 2, 3 ) = mat_pre * mat_C * huber_loss_scale;
                H_mat.block( pt_idx * 2, 21, 2, 3 ) = mat_pre * mat_D * huber_loss_scale;
            }

            if ( EstimateIntrinsic )
            {
                H_mat( pt_idx * 2, 25 ) = pt_3d_cam( 0 ) / pt_3d_cam( 2 ) * huber_loss_scale;
                H_mat( pt_idx * 2 + 1, 26 ) = pt_3d_cam( 1 ) / pt_3d_cam( 2 ) * huber_loss_scale;
                H_mat( pt_idx * 2, 27 ) = 1 * huber_loss_scale;
                H_mat( pt_idx * 2 + 1, 28 ) = 1 * huber_loss_scale;
            }
        }
        H_mat = H_mat / img_res_scale;
        acc_reprojection_error /= total_pt_size;

        last_avr_repro_err = acc_reprojection_error;
        if ( avail_pt_count < minimum_iteration_pts )
        {
            break;
        }

        H_mat_spa = H_mat.sparseView();
        Eigen::SparseMatrix< double > Hsub_T_temp_mat = H_mat_spa.transpose();
        vec_spa = ( state_iter - state_in ).sparseView();
        H_T_H_spa = Hsub_T_temp_mat * H_mat_spa;
        // Notice that we have combine some matrix using () in order to boost the matrix multiplication.
        Eigen::SparseMatrix< double > temp_inv_mat =
            ( ( H_T_H_spa.toDense() + eigen_mat< -1, -1 >( state_in.cov * mCamMeasurementWeight ).inverse() ).inverse() ).sparseView();
        KH_spa = temp_inv_mat * ( Hsub_T_temp_mat * H_mat_spa );
        solution = ( temp_inv_mat * ( Hsub_T_temp_mat * ( ( -1 * meas_vec.sparseView() ) ) ) - ( I_STATE_spa - KH_spa ) * vec_spa ).toDense();

        state_iter = state_iter + solution;

        if ( fabs( acc_reprojection_error - last_repro_err ) < 0.001 )
        {
            break;
        }
        last_repro_err = acc_reprojection_error;
    }

    if ( avail_pt_count >= minimum_iteration_pts )
    {
        state_iter.cov = ( ( I_STATE_spa - KH_spa ) * state_iter.cov.sparseView() ).toDense();
    }

    state_iter.td_ext_i2c += state_iter.td_ext_i2c_delta;
    state_iter.td_ext_i2c_delta = 0;
    state_in = state_iter;
    return true;
}

bool VisualOdometry::vio_photometric( StatesGroup &state_in, Rgbmap_tracker &op_track, std::shared_ptr< Image_frame > &image )
{
    StatesGroup state_iter = state_in;
    if ( !EstimateIntrinsic )     // When disable the online intrinsic calibration.
    {
        state_iter.cam_intrinsic << mCamK( 0, 0 ), mCamK( 1, 1 ), mCamK( 0, 2 ), mCamK( 1, 2 );
    }
    if ( !EstimateL2CExtrinsic ) // When disable the online extrinsic calibration.
    {
        state_iter.pos_ext_i2c = mInitialPosExtrinicL2c;
        state_iter.rot_ext_i2c = mInitialRotExtrinicL2c;
    }
    Eigen::Matrix< double, -1, -1 >                       H_mat, R_mat_inv;
    Eigen::Matrix< double, -1, 1 >                        meas_vec;
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > G, H_T_H, I_STATE;
    Eigen::Matrix< double, DIM_OF_STATES, 1 >             solution;
    Eigen::Matrix< double, -1, -1 >                       K, KH;
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > K_1;
    Eigen::SparseMatrix< double >                         H_mat_spa, H_T_H_spa, R_mat_inv_spa, K_spa, KH_spa, vec_spa, I_STATE_spa;
    I_STATE.setIdentity();
    I_STATE_spa = I_STATE.sparseView();
    double fx, fy, cx, cy, time_td;

    int                   total_pt_size = op_track.m_map_rgb_pts_in_current_frame_pos.size();
    std::vector< double > last_reprojection_error_vec( total_pt_size ), current_reprojection_error_vec( total_pt_size );
    if ( total_pt_size < minimum_iteration_pts )
    {
        state_in = state_iter;
        return false;
    }

    int err_size = 3;
    H_mat.resize( total_pt_size * err_size, DIM_OF_STATES );
    meas_vec.resize( total_pt_size * err_size, 1 );
    R_mat_inv.resize( total_pt_size * err_size, total_pt_size * err_size );

    double last_repro_err = 3e8;
    int    avail_pt_count = 0;
    double last_avr_repro_err = 0;
    int    if_esikf = 1;

    double acc_photometric_error = 0;
    for ( int iter_count = 0; iter_count < EsikfIterTimes; iter_count++ )
    {
        mat_3_3 R_imu = state_iter.rot_end;
        vec_3   t_imu = state_iter.pos_end;
        vec_3   t_c2w = R_imu * state_iter.pos_ext_i2c + t_imu;
        mat_3_3 R_c2w = R_imu * state_iter.rot_ext_i2c; // world to camera frame

        fx = state_iter.cam_intrinsic( 0 );
        fy = state_iter.cam_intrinsic( 1 );
        cx = state_iter.cam_intrinsic( 2 );
        cy = state_iter.cam_intrinsic( 3 );
        time_td = state_iter.td_ext_i2c_delta;

        vec_3   t_w2c = -R_c2w.transpose() * t_c2w;
        mat_3_3 R_w2c = R_c2w.transpose();
        int     pt_idx = -1;
        acc_photometric_error = 0;
        vec_3               pt_3d_w, pt_3d_cam;
        vec_2               pt_img_measure, pt_img_proj, pt_img_vel;
        eigen_mat_d< 2, 3 > mat_pre;
        eigen_mat_d< 3, 2 > mat_photometric;
        eigen_mat_d< 3, 3 > mat_d_pho_d_img;
        eigen_mat_d< 3, 3 > mat_A, mat_B, mat_C, mat_D, pt_hat;
        R_mat_inv.setZero();
        H_mat.setZero();
        solution.setZero();
        meas_vec.setZero();
        avail_pt_count = 0;
        int iter_layer = 0;
        for ( auto it = op_track.m_map_rgb_pts_in_last_frame_pos.begin(); it != op_track.m_map_rgb_pts_in_last_frame_pos.end(); it++ )
        {
            if ( ( ( RGB_pts * ) it->first )->m_N_rgb < 3 )
            {
                continue;
            }
            pt_idx++;
            pt_3d_w = ( ( RGB_pts * ) it->first )->get_pos();
            pt_img_vel = ( ( RGB_pts * ) it->first )->m_img_vel;
            pt_img_measure = vec_2( it->second.x, it->second.y );
            pt_3d_cam = R_w2c * pt_3d_w + t_w2c;
            pt_img_proj = vec_2( fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ) + cx, fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 ) + cy ) + time_td * pt_img_vel;

            vec_3   pt_rgb = ( ( RGB_pts * ) it->first )->get_rgb();
            mat_3_3 pt_rgb_info = mat_3_3::Zero();
            mat_3_3 pt_rgb_cov = ( ( RGB_pts * ) it->first )->get_rgb_cov();
            for ( int i = 0; i < 3; i++ )
            {
                pt_rgb_info( i, i ) = 1.0 / pt_rgb_cov( i, i ) ;
                R_mat_inv( pt_idx * err_size + i, pt_idx * err_size + i ) = pt_rgb_info( i, i );
            }
            vec_3  obs_rgb_dx, obs_rgb_dy;
            vec_3  obs_rgb = image->get_rgb( pt_img_proj( 0 ), pt_img_proj( 1 ), 0, &obs_rgb_dx, &obs_rgb_dy );
            vec_3  photometric_err_vec = ( obs_rgb - pt_rgb );
            double huber_loss_scale = get_huber_loss_scale( photometric_err_vec.norm() );
            photometric_err_vec *= huber_loss_scale;
            double photometric_err = photometric_err_vec.transpose() * pt_rgb_info * photometric_err_vec;

            acc_photometric_error += photometric_err;

            last_reprojection_error_vec[ pt_idx ] = photometric_err;

            mat_photometric.setZero();
            mat_photometric.col( 0 ) = obs_rgb_dx;
            mat_photometric.col( 1 ) = obs_rgb_dy;

            avail_pt_count++;
            mat_pre << fx / pt_3d_cam( 2 ), 0, -fx * pt_3d_cam( 0 ) / pt_3d_cam( 2 ), 0, fy / pt_3d_cam( 2 ), -fy * pt_3d_cam( 1 ) / pt_3d_cam( 2 );
            mat_d_pho_d_img = mat_photometric * mat_pre;

            pt_hat = Sophus::SO3d::hat( ( R_imu.transpose() * ( pt_3d_w - t_imu ) ) );
            mat_A = state_iter.rot_ext_i2c.transpose() * pt_hat;
            mat_B = -state_iter.rot_ext_i2c.transpose() * ( R_imu.transpose() );
            mat_C = Sophus::SO3d::hat( pt_3d_cam );
            mat_D = -state_iter.rot_ext_i2c.transpose();
            meas_vec.block( pt_idx * 3, 0, 3, 1 ) = photometric_err_vec ;

            H_mat.block( pt_idx * 3, 0, 3, 3 ) = mat_d_pho_d_img * mat_A * huber_loss_scale;
            H_mat.block( pt_idx * 3, 3, 3, 3 ) = mat_d_pho_d_img * mat_B * huber_loss_scale;
            if ( EstimateL2CExtrinsic )
            {
                H_mat.block( pt_idx * 3, 18, 3, 3 ) = mat_d_pho_d_img * mat_C * huber_loss_scale;
                H_mat.block( pt_idx * 3, 21, 3, 3 ) = mat_d_pho_d_img * mat_D * huber_loss_scale;
            }
        }
        R_mat_inv_spa = R_mat_inv.sparseView();

        last_avr_repro_err = acc_photometric_error;
        if ( avail_pt_count < minimum_iteration_pts )
        {
            break;
        }
        // Esikf
        if ( if_esikf )
        {
            H_mat_spa = H_mat.sparseView();
            Eigen::SparseMatrix< double > Hsub_T_temp_mat = H_mat_spa.transpose();
            vec_spa = ( state_iter - state_in ).sparseView();
            H_T_H_spa = Hsub_T_temp_mat * R_mat_inv_spa * H_mat_spa;
            Eigen::SparseMatrix< double > temp_inv_mat =
                ( H_T_H_spa.toDense() + ( state_in.cov * mCamMeasurementWeight ).inverse() ).inverse().sparseView();
            // ( H_T_H_spa.toDense() + ( state_in.cov ).inverse() ).inverse().sparseView();
            Eigen::SparseMatrix< double > Ht_R_inv = ( Hsub_T_temp_mat * R_mat_inv_spa );
            KH_spa = temp_inv_mat * Ht_R_inv * H_mat_spa;
            solution = ( temp_inv_mat * ( Ht_R_inv * ( ( -1 * meas_vec.sparseView() ) ) ) - ( I_STATE_spa - KH_spa ) * vec_spa ).toDense();
        }
        state_iter = state_iter + solution;
        acc_photometric_error /= avail_pt_count;

        // if ( ( acc_photometric_error / total_pt_size ) < 10 ) // By experience.
        // {
        //     break;
        // }
        if ( fabs( acc_photometric_error - last_repro_err ) < 0.001 )
        {
            break;
        }
        last_repro_err = acc_photometric_error;
    }
    if ( if_esikf && avail_pt_count >= minimum_iteration_pts )
    {
        state_iter.cov = ( ( I_STATE_spa - KH_spa ) * state_iter.cov.sparseView() ).toDense();
    }
    state_iter.td_ext_i2c += state_iter.td_ext_i2c_delta;
    state_iter.td_ext_i2c_delta = 0;
    state_in = state_iter;
    return true;
}

Eigen::Matrix4d toMatrix4d(const cv::Mat &R, const cv::Mat &t) {
  Eigen::Matrix4d M;
  M << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
       R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
       R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2),
       0                 , 0                 , 0                 , 1              ;
  return M;
}

void toCvMat(const Eigen::Matrix4d &T, cv::Mat &r_vec, cv::Mat &t_vec) {
  cv::Mat R(3, 3, CV_64F);
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      R.at<double>(i, j) = T(i, j);
    }
  }
  cv::Rodrigues(R, r_vec);
  t_vec = cv::Mat(3, 1, CV_64F);
  t_vec.at<double>(0) = T(0, 3);
  t_vec.at<double>(1) = T(1, 3);
  t_vec.at<double>(2) = T(2, 3);
}

VisualOdometry::VisualOdometry() {
  mCameraParam = CamParamType();
  mCol = 0;
  mRow = 0;

  mInitialized = false;
  mThreadStart = false;
  mLastTimestamp = 0;

  mState = StatesGroup();
  mFrameIdx = 0;

  mGlobalRGBMap.set_minmum_dis(0.025);
  mGlobalRGBMap.m_minimum_depth_for_projection = 0.1;
  mGlobalRGBMap.m_maximum_depth_for_projection = 200.0;
  mTracker.m_maximum_vio_tracked_pts = 600;
}

VisualOdometry::~VisualOdometry() {
  mThreadStart = false;
  if (mProcessThread != nullptr) {
    mProcessThread->join();
    mProcessThread.reset(nullptr);
  }
}

void VisualOdometry::setParameter(const CamParamType &camParam) {
  mCameraParam = camParam;
  mCamK << mCameraParam.K.at<float>(0, 0), mCameraParam.K.at<float>(0, 1), mCameraParam.K.at<float>(0, 2),
           mCameraParam.K.at<float>(1, 0), mCameraParam.K.at<float>(1, 1), mCameraParam.K.at<float>(1, 2),
           mCameraParam.K.at<float>(2, 0), mCameraParam.K.at<float>(2, 1), mCameraParam.K.at<float>(2, 2);
  mCamDist << 0, 0, 0, 0, 0;
}

bool VisualOdometry::isInitialized() {
  return mInitialized;
}

void VisualOdometry::initialize(const uint64_t& stamp, const Eigen::Matrix4d &t, cv::Mat &image, PointCloud::Ptr& cloud) {
  if (mInitialized || stamp <= mLastTimestamp || image.empty()) {
    return;
  }
  mLastTimestamp = stamp;

  // wait for loop quit
  mThreadStart = false;
  if (mProcessThread != nullptr) {
    mProcessThread->join();
    mProcessThread.reset(nullptr);
  }

  // append pointcloud to global map
  appendPoints(t, cloud);

  // initialize pose
  setInitialCameraParameter(mState);
  mState.rot_end = t.topLeftCorner<3, 3>();
  mState.pos_end = t.topRightCorner<3, 1>();

  // initialize tracker
  mImagePose = preprocessImage(stamp, image);
  mTracker.set_intrinsic(mCamK, mCamDist, cv::Size(mCol, mRow));

  // initialize tracking points
  std::vector<cv::Point2f>                pts_2d_vec;
  std::vector<std::shared_ptr<RGB_pts>>   rgb_pts_vec;
  mGlobalRGBMap.selection_points_for_projection(mImagePose, &rgb_pts_vec, &pts_2d_vec, 40);
  mTracker.init(mImagePose, rgb_pts_vec, pts_2d_vec);

  // start process loop
  mInitialized = true;
  mThreadStart = true;
  mProcessThread.reset(new std::thread(&VisualOdometry::processLoop, this));
  LOG_INFO("start visual odometry");
}

void VisualOdometry::feedImuData(ImuType &imu) {
}

bool VisualOdometry::feedImageData(const uint64_t& stamp, const Eigen::Matrix4d &t, cv::Mat &image) {
  if (!mInitialized || stamp <= mLastTimestamp || image.empty()) {
    return false;
  }
  mLastTimestamp = stamp;

  mState.rot_end = t.topLeftCorner<3, 3>();
  mState.pos_end = t.topRightCorner<3, 1>();

  std::pair<uint64_t, cv::Mat> data(stamp, image);
  mImageQueue.enqueue(data);
  return true;
}

bool VisualOdometry::getPose(Eigen::Matrix4d &pose) {
  mPoseQueue.wait_dequeue(pose);
  return true;
}

void VisualOdometry::updateMap(const Eigen::Matrix4d &t, PointCloud::Ptr &cloud) {
  if (!mInitialized) {
    return;
  }

  appendPoints(t, cloud);
}

void VisualOdometry::getColorMap(PointCloudRGB::Ptr &points) {
  mGlobalRGBMap.m_mutex_pts_vec->lock();
  int pts_size = mGlobalRGBMap.m_rgb_pts_vec.size();
  for (size_t i = 0; i < pts_size; i++) {
    if (mGlobalRGBMap.m_rgb_pts_vec[i]->m_N_rgb < 3) {
      continue;
    }
    pcl::PointXYZRGB p;
    p.x = mGlobalRGBMap.m_rgb_pts_vec[ i ]->m_pos[ 0 ];
    p.y = mGlobalRGBMap.m_rgb_pts_vec[ i ]->m_pos[ 1 ];
    p.z = mGlobalRGBMap.m_rgb_pts_vec[ i ]->m_pos[ 2 ];
    p.r = mGlobalRGBMap.m_rgb_pts_vec[ i ]->m_rgb[ 2 ];
    p.g = mGlobalRGBMap.m_rgb_pts_vec[ i ]->m_rgb[ 1 ];
    p.b = mGlobalRGBMap.m_rgb_pts_vec[ i ]->m_rgb[ 0 ];
    points->points.push_back(p);
  }
  mGlobalRGBMap.m_mutex_pts_vec->unlock();

  points->width = points->points.size();
  points->height = 1;
}

void VisualOdometry::processLoop() {
  while (mThreadStart) {
    std::pair<uint64_t, cv::Mat> image;
    if (false == mImageQueue.wait_dequeue_timed(image, 10000)) {
      continue;
    }

    mImagePose = preprocessImage(image.first, image.second);

    // LK optical flow, PnP
    mTracker.track_img(mImagePose, -20);
    mTracker.remove_outlier_using_ransac_pnp(mImagePose);

    // esikf
    bool res_esikf = true, res_photometric = true;
    res_esikf = vio_esikf(mState, mTracker);
    res_photometric = vio_photometric(mState, mTracker, mImagePose);
    setImagePose(mImagePose, mState);

    render_pts_in_voxels(mImagePose, &mGlobalRGBMap.m_voxels_recent_visited, mImagePose->m_timestamp);

    mGlobalRGBMap.update_pose_for_projection(mImagePose, -0.4);
    mTracker.update_and_append_track_pts(mImagePose, mGlobalRGBMap, 40, 0);
    mGlobalRGBMap.remove_points_from_global_map(mImagePose->m_timestamp - 30.0);

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.topLeftCorner<3, 3>()  = mState.rot_end;
    pose.topRightCorner<3, 1>() = mState.pos_end;

    mPoseQueue.enqueue(pose);
  }
}

void VisualOdometry::appendPoints(const Eigen::Matrix4d &t, PointCloud::Ptr cloud) {
  PointCloud::Ptr world_cloud(new PointCloud());
  pcl::transformPointCloud(*cloud, *world_cloud, t);

  std::vector<std::shared_ptr<RGB_pts>> pts_last_hitted;
  int number_of_new_visited_voxel = mGlobalRGBMap.append_points_to_global_map(*world_cloud, cloud->header.stamp / 1000000.0, &pts_last_hitted, 4);
  mGlobalRGBMap.m_pts_last_hitted = pts_last_hitted;
  mCamMeasurementWeight = std::max( 0.001, std::min( 5.0 / number_of_new_visited_voxel, 0.01 ) );
}

std::shared_ptr<Image_frame> VisualOdometry::preprocessImage(uint64_t stamp, cv::Mat &image) {
  // I420 to BGR
  imageCvtColor(image);
  mCol = image.cols;
  mRow = image.rows;

  std::shared_ptr<Image_frame> img_pose = std::make_shared<Image_frame>(mCamK);
  img_pose->m_img = image;
  img_pose->m_timestamp = stamp / 1000000.0;
  img_pose->init_cubic_interpolation();
  img_pose->image_equalize();
  img_pose->set_frame_idx(mFrameIdx);
  mFrameIdx++;

  setImagePose(img_pose, mState);
  return img_pose;
}
