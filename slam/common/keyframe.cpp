#include "keyframe.h"

#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "Logger.h"
#include "pcd_writer.h"
#include "Scancontext/Scancontext.h"

KeyFrame::KeyFrame(uint64_t stamp, long id, Eigen::Isometry3d odom, PointCloud::Ptr points) {
    mId = id;
    mTimestamp = stamp;
    mOdom = odom;
    mPoints = points;
}

KeyFrame::KeyFrame(long id, const std::string& data_path, bool is_load_graph)
 : mOdom(Eigen::Isometry3d::Identity()), mPoints(new PointCloud()), mDownsamplePoints(new PointCloud()), mTransfromPoints(new PointCloud()) {
    mId = id;
    mIsLoadGraph = is_load_graph;
    mTimestamp = 0;
    mDataPath = data_path;
    if (mIsLoadGraph) {
        mOdomPath = data_path + "/data";
        mPcdPath  = data_path + "/cloud.pcd";
    } else {
        mOdomPath = data_path + ".odom";
        mPcdPath  = data_path + ".pcd";
        // get timestamp
        size_t found = data_path.find_last_of("/");
        if (found != std::string::npos) {
            uint64_t sec, nsec;
            sscanf(data_path.substr(found + 1).c_str(), "%lu_%lu", &sec, &nsec);
            mTimestamp = sec * 1000000ULL + nsec / 1000ULL;
        }
    }
}

bool KeyFrame::loadOdom() {
    std::ifstream ifs(mOdomPath);
    if(!ifs) {
        return false;
    }
    if (mIsLoadGraph) {
        while(!ifs.eof()) {
            std::string token;
            ifs >> token;
            if(token.compare("stamp") == 0) {
                uint64_t sec, nsec;
                ifs >> sec;
                ifs >> nsec;
                mTimestamp = sec * 1000000ULL + nsec / 1000ULL;
            } else if(token.compare("estimate") == 0) {
                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
                        ifs >> mOdom(i, j);
                    }
                }
            } else if(token.compare("id") == 0) {
                ifs >> mId;
            }
        }
    } else {
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                ifs >> mOdom(i, j);
            }
        }
    }
    return true;
}

bool KeyFrame::loadMeta() {
    std::ifstream ifs(mDataPath + "/meta");
    if(!ifs) {
        return false;
    }
    while(!ifs.eof()) {
        std::string token;
        ifs >> token;
        if(token.compare("image") == 0) {
            int num;
            ifs >> num;
            for (int i = 0; i < num; i++) {
                std::string name;
                ifs >> name;
                mImageName.push_back(name);
            }
        }
    }
    return true;
}

bool KeyFrame::loadPcd() {
    pcl::PCDReader reader;
    int result = reader.read(mPcdPath, *mPoints);
    if (result == -1) {
        return false;
    }
    for (auto &p : mPoints->points) {
        p.intensity = p.intensity / 255.0f;
    }
    mPoints->header.stamp = mTimestamp;
    return true;
}

bool KeyFrame::loadImage() {
    for (auto name : mImageName) {
        auto image_file = mDataPath + "/" + name + ".jpg";
        std::ifstream f(image_file, std::ios_base::binary);
        mImages[name] = std::vector<char>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    return true;
}

void KeyFrame::save(const std::string& directory) {
    // write binary pcd
    savePCDFile(directory + "/cloud.pcd", *mPoints);

    // write data
    std::ofstream ofs(directory + "/data");
    if(!ofs) {
        return;
    }
    uint64_t timestamp_sec  = mTimestamp / 1000000ULL;
    uint64_t timestamp_nsec = mTimestamp % 1000000ULL * 1000;
    ofs << "stamp " << timestamp_sec << " " << timestamp_nsec << std::endl;
    ofs << "estimate" << std::endl << mOdom.matrix() << std::endl;
    ofs << "odom " << std::endl << mOdom.matrix() << std::endl;
    ofs << "id " << mId << std::endl;
}

void KeyFrame::downsample(double resolution) {
    pcl::VoxelGrid<Point> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);

    voxelgrid.setInputCloud(mPoints);
    voxelgrid.filter(*mDownsamplePoints);
    mDownsamplePoints->header = mPoints->header;
}

void KeyFrame::transformPoints() {
    pcl::transformPointCloud(*mPoints, *mTransfromPoints, mOdom.matrix());
}

cv::Mat KeyFrame::imageColorDecode(std::string image_name) {
    if (mImageCV.find(image_name) != mImageCV.end()) {
        return mImageCV[image_name];
    }
    if (mImages.find(image_name) == mImages.end()) {
        LOG_WARN("Image: {} not found in KeyFrame: {}", image_name, mId);
        return cv::Mat();
    }

    cv::Mat image_bytes(1, mImages[image_name].size(), CV_8UC1);
    memcpy(image_bytes.data, mImages[image_name].data(), sizeof(char) * mImages[image_name].size());
    mImageCV[image_name] = cv::imdecode(image_bytes, cv::IMREAD_COLOR);
    return mImageCV[image_name];
}

void KeyFrame::computeDescriptor() {
    SCManager scManager;
    mSc = scManager.makeScancontext(*(mPoints));
    mRingkey = scManager.makeRingkeyFromScancontext(mSc);
    mSectorkey = scManager.makeSectorkeyFromScancontext(mSc);
}

KeyFrame::~KeyFrame() {}

long KeyFrame::id() const {
  return mId;
}