#include "global_localization.h"

#include <omp.h>
#include <hdl_graph_slam/registrations.hpp>

using namespace Locate;
using namespace hdl_graph_slam;

const double fitness_score_max_range = 25; // 5 m
static pcl::Registration<Point, Point>::Ptr mRegistration;
static std::unique_ptr<ORB_SLAM::ORBVocabulary> mVocabulary(new ORB_SLAM::ORBVocabulary());

GlobalLocalization::GlobalLocalization(InitParameter &param, std::string cameraName, CamParamType cameraParam) : mGraphKDTree(nullptr) {
    mInitialized = false;
    mLastFrame = PointCloud::Ptr(new PointCloud());
    if (mRegistration == nullptr) {
#ifdef HAVE_CUDA_ENABLE
        mRegistration = select_registration_method("FAST_VGICP_CUDA");
#else
        mRegistration = select_registration_method("FAST_VGICP");
#endif
    }

    if (param.resolution >= 0.1) {
        mDownsampler.reset(new pcl::VoxelGrid<Point>());
        mDownsampler->setLeafSize(param.resolution, param.resolution, param.resolution);
    }
    mImageName = cameraName;
    mCameraParams = cameraParam;
    if (mImageName.length() > 0) {
        mScManager.SC_DIST_THRES = 0.25; // sensor fusion relocalization, set high scancontext threshold to 0.25
    } else {
        mScManager.SC_DIST_THRES = 0.30; // pointcloud only, set to 0.30
    }
}

GlobalLocalization::~GlobalLocalization() {
    stopSearchPose();
    mKeyFrames.clear();
    mSearchIdxMap.clear();
    clear();
}

void GlobalLocalization::clear() {
    mORBConnections.clear();
    mORBKeyFrameDatabase.reset(nullptr);
    mORBextractor.reset(nullptr);
    if (mORBMap != nullptr) {
        mORBMap->clear(); // Clear Map (this erase MapPoints and KeyFrames)
        mORBMap.reset(nullptr);
    }
}

void GlobalLocalization::feedInsData(std::shared_ptr<RTKType> &ins) {
    RTKType* ins_ptr = new RTKType();
    (*ins_ptr) = *ins;
    mInsQueue.enqueue(ins_ptr);
}

void GlobalLocalization::stopSearchPose() {
    mThreadStart = false;
    if (mSearchThread != nullptr) {
        mSearchThread->join();
        mSearchThread.reset(nullptr);
    }

    PointCloudImageType *points_image = nullptr;
    while (mPointImageQueue.try_dequeue(points_image)) {
        delete points_image;
    }
    RTKType *ins = nullptr;
    while (mInsQueue.try_dequeue(ins)) {
        delete ins;
    }
}

bool GlobalLocalization::initializePose(PointCloud::Ptr points, const std::pair<std::string, ImageType> &image, Eigen::Isometry3d &pose) {
    auto filtered = downsample(points);

    std::lock_guard<std::mutex> lock(mMutex);
    mLastFrame = filtered;
    if (!mInitialized) {
        mPointImageQueue.enqueue(new PointCloudImageType(filtered, image));
    } else {
        mPoseQueue.wait_dequeue(pose);

        mRegistration->setInputSource(filtered);
        PointCloud::Ptr aligned(new PointCloud());
        mRegistration->align(*aligned);
        Eigen::Matrix4f delta = mRegistration->getFinalTransformation();
        pose = pose * delta.cast<double>();
    }
    return mInitialized;
}

void GlobalLocalization::setInitPoseRange(PoseRange &r) {
    std::lock_guard<std::mutex> lock(mMutex);
    stopSearchPose();

    // reset variables
    mRange = r;
    mInitialized = false;

    // prepare search map
    mSearchIdxMap.clear();
    KeyMat polarcontext_invkeys_mat;
    std::vector<Eigen::MatrixXd> polarcontexts;
    for (size_t i = 0; i < mKeyFrames.size(); i++) {
        Eigen::Isometry3d loc = mKeyFrames[i]->mOdom;
        if (loc(0, 3) < mRange.x_min || loc(0, 3) > mRange.x_max ||
            loc(1, 3) < mRange.y_min || loc(1, 3) > mRange.y_max) {
            continue;
        }
        polarcontext_invkeys_mat.push_back(eig2stdvec(mKeyFrames[i]->mRingkey));
        polarcontexts.push_back(mKeyFrames[i]->mSc);
        mSearchIdxMap.push_back(i);
    }
    mScManager.buildRingKeyKDTree(polarcontext_invkeys_mat, polarcontexts);
    LOG_INFO("Localization: use {} point frames for global localization", polarcontext_invkeys_mat.size());
    // prepare camera search
    if (mImageName.length() > 0) {
        if (mVocabulary->empty()) {
            LOG_INFO("Localization: loading ORB Vocabulary");
            mVocabulary->loadFromTextFile("slam/data/ORBvoc.txt");
            LOG_INFO("Localization: Vocabulary loaded!");
        }

        if (mORBMap == nullptr) {
            mORBConnections.clear();
            mORBKeyFrameDatabase.reset(new ORB_SLAM::KeyFrameDatabase(*mVocabulary));
            mORBextractor.reset(new ORB_SLAM::ORBextractor(2000, 1.2, 8, 20, 7));
            mORBMap.reset(new ORB_SLAM::Map());

            LOG_INFO("Localization: start processing camera keyframe");
            mORBFrames = std::vector<ORB_SLAM::Frame>(mKeyFrames.size());
            std::vector<ORB_SLAM::KeyFrame*> pKeyFrames(mKeyFrames.size(), nullptr);
            #pragma omp parallel for num_threads(4)
            for (size_t i = 0; i < mKeyFrames.size(); i++) {
                // decode keyframe image to BGR -> Gray
                cv::Mat image_color = mKeyFrames[i]->imageColorDecode(mImageName);
                if (image_color.empty()) {
                    continue;
                }
                cv::Mat gray;
                cv::cvtColor(image_color, gray, cv::COLOR_BGR2GRAY);

                // build keyframe database
                ORB_SLAM::ORBextractor extractor(2000, 1.2, 8, 20, 7);
                mORBFrames[i] = ORB_SLAM::Frame(gray, 0, &extractor, mVocabulary.get(), mCameraParams.K, mCameraParams.DistCoef, 0, 0);
                mORBFrames[i].mnId = mKeyFrames[i]->mId;
                Eigen::Matrix4d Tcw = mCameraParams.staticTrans * mKeyFrames[i]->mOdom.matrix() * mCameraParams.staticTrans.inverse();
                mORBFrames[i].SetPose(ORB_SLAM::Converter::toCvMat(Tcw));

                pKeyFrames[i] = new ORB_SLAM::KeyFrame(mORBFrames[i], mORBMap.get(), mORBKeyFrameDatabase.get());
                pKeyFrames[i]->mnId = i;
                pKeyFrames[i]->ComputeBoW();
            }

            std::map<int, ORB_SLAM::KeyFrame*> ORBFramesHash;
            for (size_t i = 0; i < pKeyFrames.size(); i++) {
                if (pKeyFrames[i] == nullptr) {
                    continue;
                }
                mORBMap->AddKeyFrame(pKeyFrames[i]);
                mORBKeyFrameDatabase->add(pKeyFrames[i]);
                ORBFramesHash[pKeyFrames[i]->mnFrameId] = pKeyFrames[i];
                mORBConnections[pKeyFrames[i]->mnFrameId] = std::vector<ORB_SLAM::KeyFrame*>();
            }

            for (size_t i = 0; i < mMapConnections.size(); i++) {
                const EdgeType &conn = mMapConnections[i];
                if (ORBFramesHash.find(conn.prev) == ORBFramesHash.end() || ORBFramesHash.find(conn.next) == ORBFramesHash.end()) {
                    continue;
                }
                mORBConnections[conn.prev].push_back(ORBFramesHash[conn.next]);
                mORBConnections[conn.next].push_back(ORBFramesHash[conn.prev]);
            }
        }
    }

    // start search thread
    LOG_INFO("Localization: search range x: [{}, {}] y: [{}, {}]", mRange.x_min, mRange.x_max, mRange.y_min, mRange.y_max);
    mThreadStart = true;
    mSearchThread.reset(new std::thread(&GlobalLocalization::runSearchPose, this));
}

void GlobalLocalization::setInitPose(const Eigen::Matrix4d &t) {
    std::lock_guard<std::mutex> lock(mMutex);
    stopSearchPose();

    mInitialized = true;
    mRegistration->setInputTarget(mLastFrame);
    mPoseQueue.enqueue(Eigen::Isometry3d(t));
}

int GlobalLocalization::getEstimatePose(Eigen::Matrix4d &t) {
    std::lock_guard<std::mutex> lock(mMutex);
    stopSearchPose();

    // reset variables
    mInitialized = false;

    std::vector<int> pointIdx(5);
    std::vector<float> pointDistance(5);
    pcl::PointXYZ searchPoint;
    searchPoint.x = t(0, 3);
    searchPoint.y = t(1, 3);
    searchPoint.z = t(2, 3);
    if (mGraphKDTree->nearestKSearch(searchPoint, 5, pointIdx, pointDistance) <= 0) {
        LOG_ERROR("getEstimatePose: kdtree search error");
        return 0;
    }

    PointCloud::Ptr localMap = PointCloud::Ptr(new PointCloud());
    for (size_t i = 0; i < pointIdx.size(); ++i) {
        if (pointDistance[i] > 400) { // 20m
            break;
        }
        *localMap += *(mKeyFrames[pointIdx[i]]->mTransfromPoints);
    }
    if (localMap->empty() || mLastFrame->empty()) {
        LOG_ERROR("getEstimatePose: data is empty");
        return 0;
    }

    mRegistration->setInputTarget(localMap);
    mRegistration->setInputSource(mLastFrame);

    double localMapHeightMean = 0;
    for (size_t i = 0; i < localMap->points.size(); i++) {
        localMapHeightMean += localMap->points[i].z;
    }
    localMapHeightMean = localMapHeightMean / localMap->points.size();

    double height_mean = 0;
    for (size_t i = 0; i < mLastFrame->points.size(); i++) {
        height_mean += mLastFrame->points[i].z;
    }
    height_mean = height_mean / mLastFrame->points.size();
    t(2, 3) = localMapHeightMean - height_mean;

    PointCloud::Ptr aligned(new PointCloud());
    mRegistration->align(*aligned, t.cast<float>());
    if(!mRegistration->hasConverged()) {
        LOG_WARN("global localization registration has not converged!!");
        return 0;
    }

    int result = 0;
    double score = mRegistration->getFitnessScore(fitness_score_max_range);
    if (score < 5.0) {
        t = mRegistration->getFinalTransformation().cast<double>();
    }
    if (score < 0.5) {
        result = 1;
        mInitialized = true;
        mRegistration->setInputTarget(mLastFrame);
        mPoseQueue.enqueue(Eigen::Isometry3d(t));
    }

    return result;
}

void GlobalLocalization::setGraphMap(std::vector<std::shared_ptr<KeyFrame>> &map, std::vector<EdgeType> &connections) {
    mKeyFrames = map;
    buildGraphKDTree();
    mMapConnections = connections;
    PoseRange r(-10000, 10000, -10000, 10000);
    setInitPoseRange(r);
}

void GlobalLocalization::runSearchPose() {
    LOG_INFO("Localization: start pose search thread");
    while (mThreadStart) {
        PointCloudImageType *points_image = nullptr;
        if (false == mPointImageQueue.wait_dequeue_timed(points_image, 100000)) {
            continue;
        }
        // flush the points/image queue to get lastest pointcloud/Image
        PointCloudImageType *points_image_latest = nullptr;
        while (mPointImageQueue.try_dequeue(points_image_latest)) {
            delete points_image;
            points_image = points_image_latest;
        }
        std::shared_ptr<PointCloudImageType> pointImagePtr = std::shared_ptr<PointCloudImageType>(points_image);
        PointCloud::Ptr cloud = pointImagePtr->points;
        // try get latest ins data
        RTKType *ins = nullptr;
        RTKType *ins_latest = nullptr;
        while (mInsQueue.try_dequeue(ins_latest)) {
            if (ins != nullptr) {
                delete ins;
            }
            ins = ins_latest;
        }
        std::shared_ptr<RTKType> insPtr = std::shared_ptr<RTKType>(ins);

        // process
        auto clock = std::chrono::steady_clock::now();
        // initialize variables
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

        // pointcloud search
        std::pair<int, float> init_match{-1, 0.0};
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

        // image search
        std::pair<int, float> camera_match{-1, -1.0};
        Eigen::Matrix4f camera_guess = Eigen::Matrix4f::Identity();
        std::unique_ptr<std::thread> thread_image(nullptr);
        if (insPtr != nullptr) {
            init_match = localSearch(cloud, insPtr);
            if (insPtr->dimension == 6 && init_match.first != -1) {
                init_guess = mKeyFrames[init_match.first]->mOdom.cast<float>().inverse() * insPtr->T.cast<float>();
                init_guess(2, 3) = 0; // the altitude of INS is not reliable, set to z=0
            } else {
                init_guess = yaw2matrix(-init_match.second);
            }
        } else {
            thread_image.reset(new std::thread(&GlobalLocalization::imageSearch, this, std::ref(cloud), std::ref(pointImagePtr->image), std::ref(camera_match), std::ref(camera_guess)));
            init_match = globalSearch(cloud);
            init_guess = yaw2matrix(-init_match.second);
        }

        // scancontext match success
        if (init_match.first != -1) {
            mInitialized = registrationAlign("SC", init_match, init_guess, cloud, pose);
        }

        if (thread_image != nullptr) {
            thread_image->join();
            // BoW match success
            if (mInitialized == false && camera_match.first != -1) {
                mInitialized = registrationAlign("BoW", camera_match, camera_guess, cloud, pose);
            }
        }

        auto elapseMs = since(clock).count();
        LOG_DEBUG("Initial pose search: cost {} ms", elapseMs);

        if (mInitialized) {
            mPoseQueue.enqueue(pose);
            LOG_INFO("Localization: initialize pose success");
            std::cout << pose.matrix() << std::endl;
            break;
        }
    }
}

std::pair<int, float> GlobalLocalization::localSearch(PointCloud::Ptr &cloud, std::shared_ptr<RTKType> &ins) {
    std::pair<int, float> best_match{-1, 0.0};

    std::vector<int> pointIdx(10);
    std::vector<float> pointDistance(10);
    pcl::PointXYZ searchPoint;
    searchPoint.x = ins->T(0, 3);
    searchPoint.y = ins->T(1, 3);
    searchPoint.z = 0; // ins->T(2, 3);

    if (mGraphKDTree->nearestKSearch(searchPoint, 10, pointIdx, pointDistance) <= 0) {
        LOG_ERROR("global localization kdtree search error");
        return best_match;
    }

    Eigen::MatrixXd sc = mScManager.makeScancontext(*cloud);
    double min_dist = 1.0;
    for (size_t i = 0; i < pointIdx.size(); ++i) {
        if (best_match.first == -1 && pointDistance[i] > (ins->precision * ins->precision)) {
            LOG_WARN("Localization: current position for initial search is far away from the map, distance {} ", std::sqrt(pointDistance[i]));
            return best_match;
        }
        if (best_match.first != -1 && pointDistance[i] > (ins->precision * ins->precision) / 10.0) {
            break;
        }
        std::pair<double, int> sc_dist_result = mScManager.distanceBtnScanContext(sc, mKeyFrames[pointIdx[i]]->mSc);
        if( sc_dist_result.first < min_dist ) {
            min_dist = sc_dist_result.first;
            best_match.first = pointIdx[i];
            best_match.second = (sc_dist_result.second * mScManager.PC_UNIT_SECTORANGLE) * M_PI / 180.0;
        }
    }
    LOG_INFO("initialize local search, sc score {}", min_dist);

    return best_match;
}

std::pair<int, float> GlobalLocalization::globalSearch(PointCloud::Ptr &cloud) {
    std::vector<std::pair<double, double>> search_trans = {
        {0, 0}, {-4, 0}, {4, 0}, {0, -4}, {0, 4}, {-4, -4}, {-4, 4}, {4, -4}, {4, 4}
    };
    double min_dist = std::numeric_limits<double>::max();
    std::pair<int, float> best_match{-1, 0.0};
    for (auto &t : search_trans) {
        Eigen::MatrixXd sc = mScManager.makeScancontext(*cloud, t.first, t.second);
        std::vector<float> ringkey = eig2stdvec(mScManager.makeRingkeyFromScancontext(sc));
        Eigen::MatrixXd sectorkey = mScManager.makeSectorkeyFromScancontext(sc);
        double sc_dist = 1.0;
        auto match = mScManager.detectClosestMatch(sc, ringkey, sectorkey, sc_dist);
        if (sc_dist < min_dist) {
            min_dist = sc_dist;
            best_match = match;
        }
    }

    int match_idx = best_match.first;
    if (match_idx != -1) {
        LOG_INFO("scancontext search success, score {}", min_dist);
        best_match.first = mSearchIdxMap[match_idx];
    } else {
        LOG_WARN("scancontext search fail, score {}", min_dist);
    }
    return best_match;
}

void GlobalLocalization::imageSearch(PointCloud::Ptr &cloud, std::pair<std::string, ImageType> &im, std::pair<int, float> &init_match, Eigen::Matrix4f &init_guess) {
    std::string image_name = im.first;
    cv::Mat image = im.second.image;
    if (image.empty()) {
        LOG_DEBUG("global localization, empty image input");
        return;
    }

    cv::Mat gray;
    if (image.type() == CV_8UC1) {
        cv::cvtColor(image, gray, cv::COLOR_YUV2GRAY_I420);
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    ORB_SLAM::Frame currentFrame = ORB_SLAM::Frame(gray, 0, mORBextractor.get(), mVocabulary.get(), mCameraParams.K, mCameraParams.DistCoef, 0, 0);
    currentFrame.ComputeBoW();

    ORB_SLAM::KeyFrame* bestMatch = mORBKeyFrameDatabase->DetectBestCandidates(&currentFrame, mORBConnections, init_match.second);
    if (bestMatch != nullptr) {
        std::vector<std::pair<double, double>> search_trans = {
            {0, 0}, {-4, 0}, {4, 0}, {0, -4}, {0, 4}, {-4, -4}, {-4, 4}, {4, -4}, {4, 4}
        };
        std::pair<double, int> best_sc_dist{1.0, 0};
        for (auto &t : search_trans) {
            Eigen::MatrixXd sc = mScManager.makeScancontext(*cloud, t.first, t.second);
            std::pair<double, int> sc_dist = mScManager.distanceBtnScanContext(sc, mKeyFrames[bestMatch->mnId]->mSc);
            if (sc_dist.first < best_sc_dist.first) {
                best_sc_dist = sc_dist;
            }
        }

        float estimate_yaw = best_sc_dist.second * mScManager.PC_UNIT_SECTORANGLE;
        init_match.first = bestMatch->mnId;
        init_guess = yaw2matrix(-estimate_yaw);
        LOG_INFO("BoW search, score {}, sc-score {}", init_match.second, best_sc_dist.first);
        if (best_sc_dist.first > (mScManager.SC_DIST_THRES * 1.5)) {
            init_match.first = -1;
        }
    }
}

bool GlobalLocalization::registrationAlign(const std::string &sensor, std::pair<int, float> &init_match, Eigen::Matrix4f &init_guess, PointCloud::Ptr &cloud, Eigen::Isometry3d &pose) {
    mRegistration->setInputTarget(mKeyFrames[init_match.first]->mPoints);
    mRegistration->setInputSource(cloud);

    PointCloud::Ptr aligned(new PointCloud());
    mRegistration->align(*aligned, init_guess);
    if(!mRegistration->hasConverged()) {
        LOG_WARN("global localization registration has not converged!!");
        return false;
    }

    double score = mRegistration->getFitnessScore(fitness_score_max_range);
    Eigen::Matrix4f delta = mRegistration->getFinalTransformation();
    Eigen::Matrix4f deltadelta = init_guess.inverse() * delta;
    float dx = Eigen::Isometry3f(deltadelta).translation().norm();
    float da = Eigen::AngleAxisf(Eigen::Isometry3f(deltadelta).linear()).angle() / M_PI * 180;

    if (score < 1.5 && dx < 10.0 && da < 30.0) {
        LOG_INFO("global registration, {} match success, score {}, dx {}, da {}", sensor, score, dx, da);
        pose = mKeyFrames[init_match.first]->mOdom * delta.cast<double>();
        mRegistration->setInputTarget(cloud);
        return true;
    } else {
        LOG_WARN("global registration, {} not match, score {}, dx {}, da {}", sensor, score, dx, da);
        return false;
    }
}

void GlobalLocalization::buildGraphKDTree() {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = static_cast<int>(mKeyFrames.size());
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < mKeyFrames.size(); i++) {
        cloud->points[i].x = mKeyFrames[i]->mOdom(0, 3);
        cloud->points[i].y = mKeyFrames[i]->mOdom(1, 3);
        cloud->points[i].z = 0;
    }
    mGraphKDTree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    mGraphKDTree->setInputCloud(cloud);
}

PointCloud::Ptr GlobalLocalization::downsample(PointCloud::Ptr& cloud) {
    if(!mDownsampler) {
      return cloud;
    }

    PointCloud::Ptr filtered(new PointCloud());
    mDownsampler->setInputCloud(cloud);
    mDownsampler->filter(*filtered);
    filtered->header = cloud->header;
    return filtered;
}