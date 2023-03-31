#include "coarse_segmentation.h"

CoarseSegmentation::CoarseSegmentation(std::shared_ptr<Parameters> params_ptr) {
  param_ptr_ = params_ptr->segmentation_param_;
  transform_pd_ = PointCloudPtr(new PointCloud());
  dim_x_ = (param_ptr_->grid_max_x - param_ptr_->grid_min_x) / 
           param_ptr_->grid_size_x;
  dim_y_ = (param_ptr_->grid_max_y - param_ptr_->grid_min_y) / 
           param_ptr_->grid_size_y;  
  grid_map_ = MyGridMap(dim_x_, std::vector<Grid>(dim_y_));
  to_grid_pose_.delta_x = -param_ptr_->grid_min_x;
  to_grid_pose_.delta_y = -param_ptr_->grid_min_y;
  to_grid_pose_.delta_z = 0;
  to_grid_angles_.pitch = 0;
  to_grid_angles_.roll = 0;
  to_grid_angles_.yaw = 0;

  polar_num_ = std::ceil(param_ptr_->lidar_fov / param_ptr_->theta_divider);
  polar_vector_ = PolarVector(polar_num_);

  grid_info_.xmin = param_ptr_->grid_min_x;
  grid_info_.xmax = param_ptr_->grid_max_x;
  grid_info_.ymin = param_ptr_->grid_min_y;
  grid_info_.ymax = param_ptr_->grid_max_y;
  grid_info_.zmin = param_ptr_->grid_min_z;
  grid_info_.zmax = param_ptr_->grid_max_z;
  grid_info_.gridres = param_ptr_->grid_size_x;
  grid_info_.xnum = dim_x_;
  grid_info_.ynum = dim_y_;
  grid_info_.gridnum = dim_x_ * dim_y_;
  grid_info_.cells = std::vector<CellInfo>(grid_info_.gridnum);
  grid_info_.is_Passable = std::vector<u_int8_t>(grid_info_.gridnum, 1);
  // gridmap_ptr_ = std::make_shared<GridMap>();
}

CoarseSegmentation::~CoarseSegmentation() {}

void CoarseSegmentation::Run(PointCloudPtr n_ground_pd) { 
  n_ground_pd_ = n_ground_pd;
  transform_pd_.reset(new PointCloud());
  CreateGridMap();
}

void CoarseSegmentation::CreateGridMap() {
  // gridmap_ptr_->clear();
  MyGridMap(dim_x_, std::vector<Grid>(dim_y_)).swap(grid_map_);
  PolarVector(polar_num_).swap(polar_vector_);
  // tranlation to right_down conner
  Rotate();
  int n = transform_pd_->size();
  for (int i = 0; i < n; ++i) {
    int index_x = transform_pd_->points[i].x / param_ptr_->grid_size_x;
    int index_y = transform_pd_->points[i].y / param_ptr_->grid_size_y;
    index_x = index_x >= dim_x_ ? dim_x_ - 1 : index_x;
    index_y = index_y >= dim_y_ ? dim_y_ - 1 : index_y;

    grid_map_[index_x][index_y].points.push_back(i);
    // 旋转后的点云用于计算在gridmap中位置，实际计算仍用原始点云计算
    grid_map_[index_x][index_y].min_z = grid_map_[index_x][index_y].min_z
                                        < n_ground_pd_->points[i].z? 
                                        grid_map_[index_x][index_y].min_z : 
                                        n_ground_pd_->points[i].z;
    grid_map_[index_x][index_y].max_z = grid_map_[index_x][index_y].max_z
                                        > n_ground_pd_->points[i].z? 
                                        grid_map_[index_x][index_y].max_z : 
                                        n_ground_pd_->points[i].z;   

    grid_map_[index_x][index_y].sum_x += n_ground_pd_->points[i].x;                                                                                 
    grid_map_[index_x][index_y].sum_y += n_ground_pd_->points[i].y;                                                                                 
    grid_map_[index_x][index_y].sum_z += n_ground_pd_->points[i].z;  
    grid_map_[index_x][index_y].count++;
    grid_map_[index_x][index_y].points.push_back(i);
    if (grid_map_[index_x][index_y].count >= param_ptr_->num_thres) {
      grid_map_[index_x][index_y].is_occupied = true;
    }
  }
  for (int i = 0; i < dim_x_; ++i) {
    for (int j = 0; j < dim_y_; ++j) {
      if (grid_map_[i][j].is_occupied) {
        if (grid_map_[i][j].min_z > param_ptr_->min_z_thres) {
          grid_map_[i][j].is_occupied = false;
        }
        float range_z = grid_map_[i][j].max_z - grid_map_[i][j].min_z;
        float mean_z = grid_map_[i][j].sum_z / grid_map_[i][j].count; // count >= num_thres
        if (range_z < 0.15 && mean_z > -0.2 && mean_z < 0.2) {
          grid_map_[i][j].is_occupied = false;
        }
      }     
      int cell_index = i * dim_y_ + j;
      grid_info_.cells[cell_index].isPassable = !grid_map_[i][j].is_occupied;
      grid_info_.is_Passable[cell_index] = !grid_map_[i][j].is_occupied;
      grid_info_.cells[cell_index].count = grid_map_[i][j].count;
      grid_map_[i][j].center.x = i * param_ptr_->grid_size_x + 
                      param_ptr_->grid_size_x / 2 + param_ptr_->grid_min_x;
      grid_map_[i][j].center.y = j * param_ptr_->grid_size_y + 
                      param_ptr_->grid_size_y / 2 + param_ptr_->grid_min_y; 
      grid_map_[i][j].center.z = 0;  
      float radius = sqrt(std::pow(grid_map_[i][j].center.x, 2)
                     + std::pow(grid_map_[i][j].center.y, 2));
      float theta = atan2(grid_map_[i][j].center.y, grid_map_[i][j].center.x) 
                    * 180 / M_PI;
      theta = theta < 0? theta + 360 : theta;
      int index = theta / param_ptr_->theta_divider; 
      if (radius > polar_vector_[index].max_radius) {
        polar_vector_[index].max_radius = radius;
        polar_vector_[index].farthest_point.x = grid_map_[i][j].center.x;
        polar_vector_[index].farthest_point.y = grid_map_[i][j].center.y;
        polar_vector_[index].farthest_point.z = grid_map_[i][j].center.z;
      }  
      if (grid_map_[i][j].is_occupied) {
        polar_vector_[index].grids.push_back(grid_map_[i][j]);
        polar_vector_[index].is_empty = false;
        if (radius < polar_vector_[index].min_radius) {
          polar_vector_[index].min_radius = radius;
          polar_vector_[index].nearest_point.x = grid_map_[i][j].sum_x / 
                                                 grid_map_[i][j].count;
          polar_vector_[index].nearest_point.y = grid_map_[i][j].sum_y / 
                                                 grid_map_[i][j].count;
          polar_vector_[index].nearest_point.z = grid_map_[i][j].sum_z / 
                                                 grid_map_[i][j].count;
        }   
      }                               
    }
  }
}

void CoarseSegmentation::Rotate() {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << to_grid_pose_.delta_x, 
                                    to_grid_pose_.delta_y, 
                                    to_grid_pose_.delta_z;
  transform_matrix.rotate(Eigen::AngleAxisf(to_grid_angles_.yaw, 
                                            Eigen::Vector3f::UnitZ()));
  transform_matrix.rotate(Eigen::AngleAxisf(to_grid_angles_.pitch, 
                                            Eigen::Vector3f::UnitY()));
  transform_matrix.rotate(Eigen::AngleAxisf(to_grid_angles_.roll, 
                                             Eigen::Vector3f::UnitX())); 
  pcl::transformPointCloud(*n_ground_pd_, *transform_pd_, transform_matrix);
}