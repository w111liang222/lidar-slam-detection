#ifndef COMMON_H
#define COMMON_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <map>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

// #include <tf/tf.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#define SENSOR_HEIGHT 1.73

#define UNLABELED 0
#define OUTLIER 1
#define NUM_ALL_CLASSES 34
#define ROAD 40
#define PARKING 44
#define SIDEWALKR 48
#define OTHER_GROUND 49
#define BUILDING 50
#define FENSE 51
#define LANE_MARKING 60
#define VEGETATION 70
#define TERRAIN 72

#define TRUEPOSITIVE 3
#define TRUENEGATIVE 2
#define FALSEPOSITIVE 1
#define FALSENEGATIVE 0

  /** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint16_t, id, id))

void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr dst);

int CountNumGround(const pcl::PointCloud<PointXYZILID>& pc);

std::map<int, int> SetInitialGtCounts(std::vector<int>& gt_classes);

std::map<int, int> CountNumEachClass(const pcl::PointCloud<PointXYZILID>& pc);

int CountNumOutliers(const pcl::PointCloud<PointXYZILID>& pc);

void DiscernGround(const pcl::PointCloud<PointXYZILID>& src, 
                   pcl::PointCloud<PointXYZILID>& ground, 
                   pcl::PointCloud<PointXYZILID>& non_ground);

void CalculatePrecisionRecall(const pcl::PointCloud<PointXYZILID>& pc_curr,
                                pcl::PointCloud<PointXYZILID>& ground_estimated,
                                double & precision,
                                double& recall,
                                bool consider_outliers);

void SaveAllLabels(const pcl::PointCloud<PointXYZILID>& pc, 
                  std::string ABS_DIR, std::string seq, int count);

void SaveAllAccuracy(const pcl::PointCloud<PointXYZILID>& pc_curr,
                      pcl::PointCloud<PointXYZILID>& ground_estimated, 
                      std::string acc_filename, double& accuracy, 
                      std::map<int, int>&pc_curr_gt_counts, 
                      std::map<int, int>&g_est_gt_counts);

void pc2pcdfile(const pcl::PointCloud<PointXYZILID>& TP, const pcl::PointCloud<PointXYZILID>& FP,
                const pcl::PointCloud<PointXYZILID>& FN, const pcl::PointCloud<PointXYZILID>& TN,
                std::string pcd_filename);


inline Eigen::MatrixXf LeastSquares(std::vector<std::vector<double>> input);

template <typename DataType> // Eigen::Vector3d
inline double GetAngle3d(const DataType v1, const DataType v2);

template <typename T>
inline double xy2radius(const T &x, const T &y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}

template <typename T>
inline double xy2theta(const T &x, const T &y) {
  auto atan_value = std::atan2(y,x); // EDITED!
  return atan_value > 0 ? atan_value : atan_value + 2 * M_PI; // EDITED!
}

struct Timing {
  // typedef std::chrono::steady_clock clock_t;
  typedef std::chrono::high_resolution_clock clock_t;
  typedef std::chrono::milliseconds ms;  
  clock_t::time_point start_point;
  clock_t::time_point stop_point;  
  inline double get_time() { 
    return std::chrono::duration_cast<ms>(stop_point - start_point).count(); 
  }  
  inline void start() { start_point = clock_t::now(); }  
  inline void stop() { stop_point = clock_t::now(); }
};


// int main() {
//     // txt点数据文件路径
//     const string fileName = "./points.txt";
    
//     // 设置是几次拟合
//     const int N = 2;
    
//     // 创建两个vector
//     vector<float> x, y;
    
//     // 读取文件
//     ifstream f(fileName);
    
//     if (!f) {
//         cout << "数据文件打开失败" << endl;
//         exit(EXIT_FAILURE);
//     }
    
//     float tempx, tempy;
    
//     while (f >> tempx >> tempy) {
//         x.push_back(tempx);
//         y.push_back(tempy);
//     }
    
//     if (x.size() != y.size()) {
//         cout << "数据文件内容有误" << endl;
//         exit(EXIT_FAILURE);
//     }
    
//     // 创建A矩阵
//     Eigen::MatrixXd A(x.size(), N + 1);
    
//     for (unsigned int i = 0; i < x.size(); ++i) {  // 遍历所有点
        
//         for (int n = N, dex = 0; n >= 1; --n, ++dex) {  // 遍历N到1阶
//             A(i, dex) = pow(x[i], n);
//         }
        
//         A(i, N) = 1;  //
//     }
    
//     // 创建B矩阵
//     Eigen::MatrixXd B(y.size(), 1);
    
//     for (unsigned int i = 0; i < y.size(); ++i) {
//         B(i, 0) = y[i];
//     }
  	
//     // 创建矩阵W
//     Eigen::MatrixXd W;
//     W = (A.transpose() * A).inverse() * A.transpose() * B;
    
//   	// 打印W结果
//     cout << W << endl;
// }
#endif // 