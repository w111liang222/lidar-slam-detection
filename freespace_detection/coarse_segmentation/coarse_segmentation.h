#pragma once
#include <memory>

#include <pcl/common/transforms.h>

#include "../common/parameters.h"
#include "../common/types.h"

struct CellInfo {
  bool isPassable;
  int count;
};

struct GridInfo {
  GridInfo() {
    xmin = 1e4;
    xmax = 1e4;
    ymin = 1e4;
    ymax = 1e4;
    zmin = 1e4;
    zmax = 1e4;
    gridres = 1.0;
    xnum = 0;
    ynum = 0;
    gridnum = 0;
  }
  GridInfo(double xMin, double xMax, double yMin, double yMax, 
            double zMin, double zMax, double gridRes) {
    xmin = xMin;
    xmax = xMax;
    ymin = yMin;
    ymax = yMax;
    zmin = zMin;
    zmax = zMax;
    gridres = gridRes;
    xnum = floor((xmax - xmin) / gridres);
    ynum = floor((ymax - ymin) / gridres);
    gridnum = xnum * ynum;
    cells.resize(gridnum);
  }
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double zmin;
  double zmax;
  double gridres;
  int xnum;
  int ynum;
  int gridnum;
  std::vector<CellInfo> cells;
  std::vector<u_int8_t> is_Passable;
};


class CoarseSegmentation {
 public:
  CoarseSegmentation(std::shared_ptr<Parameters> params_ptr);
  ~CoarseSegmentation();
  void Run(PointCloudPtr n_ground_pd);
  MyGridMap GetGridMap() { return grid_map_; }
  PolarVector GetPolarVector() { return polar_vector_; }
  GridInfo GetGridInfo() { return grid_info_; }

 private:
  void CreateGridMap();
  // void Segmentation() {}
  // 将点云坐标系由grid_map中心点转换至grid_map左上角顶点，方便构建grid_map
  void Rotate();

 private:
  int dim_x_; // gridmap  parameter
  int dim_y_; // gridmap  parameter
  int polar_num_; // polarmap parameter
  AttitudeAngles to_grid_angles_;  //由grid_map中心点转换至grid_map左上角顶点
  Translation to_grid_pose_; //由grid_map中心点转换至grid_map左上角顶点
  PointCloudPtr n_ground_pd_;
  PointCloudPtr transform_pd_;
  // PointCloudPtr segmentation_pd_;
  std::shared_ptr<CoarseSegmentationParam> param_ptr_;
  MyGridMap grid_map_;
  PolarVector polar_vector_;
  GridInfo grid_info_;
  // std::shared_ptr<GridMap> gridmap_ptr_;
};



