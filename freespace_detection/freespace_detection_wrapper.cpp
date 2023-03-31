#include <dirent.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "freespace_detection_wrapper.h"
#include "detection.pb.h"
namespace py=pybind11;

static double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_, resolution_;
FreespaceDetPtr freespace_detection_ptr;
std::shared_ptr<Parameters> params_ptr_ = std::make_shared<Parameters>();

void set_grid_detection_params(double x_min, double x_max, double y_min, 
    double y_max, double z_min, double z_max, double grid_resolution) {
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  z_min_ = z_min;
  z_max_ = z_max;
  resolution_ = grid_resolution;

  params_ptr_->preprocess_param_->range_front_x = x_max_;
  params_ptr_->preprocess_param_->range_rear_x = x_min_; 
  params_ptr_->preprocess_param_->range_left_y = y_max_;
  params_ptr_->preprocess_param_->range_right_y = y_min_;
  params_ptr_->preprocess_param_->range_up_z = z_max_;
  params_ptr_->preprocess_param_->range_down_z = z_min_;

  params_ptr_->remove_ground_param_->range_front_x = x_max_;
  params_ptr_->remove_ground_param_->range_rear_x = x_min_;
  params_ptr_->remove_ground_param_->range_left_y = y_max_;
  params_ptr_->remove_ground_param_->range_right_y = y_min_;

  params_ptr_->segmentation_param_->grid_max_x = x_max_;
  params_ptr_->segmentation_param_->grid_min_x = x_min_;
  params_ptr_->segmentation_param_->grid_max_y = y_max_;
  params_ptr_->segmentation_param_->grid_min_y = y_min_;
  params_ptr_->segmentation_param_->grid_max_z = z_max_;
  params_ptr_->segmentation_param_->grid_min_z = z_min_;                 
  params_ptr_->segmentation_param_->grid_size_x = resolution_;
  params_ptr_->segmentation_param_->grid_size_y = resolution_;    
  
  freespace_detection_ptr = std::make_shared<FreespaceDetection>(params_ptr_);  

}

py::bytes detection(py::array_t<float> &input, int num) {
  GridInfo gridInfo = GridInfo(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_, resolution_);
  Freespace freespace;
  auto info = freespace.mutable_info();
  info->set_x_min(x_min_);
  info->set_x_max(x_max_);
  info->set_y_min(y_min_);
  info->set_y_max(y_max_);
  info->set_z_min(z_min_);
  info->set_z_max(z_max_);
  info->set_resolution(resolution_);
  int xnum = floor((x_max_ - x_min_) / resolution_);
  int ynum = floor((y_max_ - y_min_) / resolution_);
  info->set_x_num(xnum);
  info->set_y_num(ynum);
  int gridnum = xnum * ynum;

  for (auto& cell : gridInfo.cells) {
    cell.count = 0;
    cell.isPassable = true;
  }
  float zMin = z_min_, zMax = z_max_;
  // int passableCnt = gridnum;

  auto ref_input = input.unchecked<2>();
  for (int i = 0; i < num; i++) {
    float x = ref_input(i, 0);
    float y = ref_input(i, 1);
    float z = ref_input(i, 2);
    if (x > gridInfo.xmin && x < gridInfo.xmax && y < gridInfo.ymax && y > gridInfo.ymin) {
      int idx = (int)(floor((x + gridInfo.xmax) / gridInfo.gridres)
                      + floor((y - gridInfo.ymin) / gridInfo.gridres) * gridInfo.xnum);
      if (idx >= gridInfo.gridnum) {
        continue;
      }
      if (z > zMin && z < zMax) {
        if (idx == 0) {
          gridInfo.cells[idx].count++;

          gridInfo.cells[idx + 1].count ++;

          gridInfo.cells[idx + gridInfo.xnum + 1].count ++;

          gridInfo.cells[idx + gridInfo.xnum - 1].count ++;

          gridInfo.cells[idx + gridInfo.xnum].count ++;
        } else if (idx == gridInfo.gridnum) {
            gridInfo.cells[idx].count ++;

            gridInfo.cells[idx - 1].count ++;

            gridInfo.cells[idx - gridInfo.xnum - 1].count ++;

            gridInfo.cells[idx - gridInfo.xnum + 1].count ++;

            gridInfo.cells[idx - gridInfo.xnum].count ++;
        } else if (idx < gridInfo.xnum + 1) {
            gridInfo.cells[idx].count ++;

            gridInfo.cells[idx - 1].count ++;

            gridInfo.cells[idx + 1].count ++;

            gridInfo.cells[idx + gridInfo.xnum + 1].count ++;

            gridInfo.cells[idx + gridInfo.xnum - 1].count ++;

            gridInfo.cells[idx + gridInfo.xnum].count ++;
        } else if (idx > gridInfo.gridnum - gridInfo.xnum - 1) {
            gridInfo.cells[idx].count ++;
            
            gridInfo.cells[idx - 1].count ++;

            if (idx + 1 < gridInfo.gridnum) {
              gridInfo.cells[idx + 1].count ++;
            }
            gridInfo.cells[idx - gridInfo.xnum - 1].count ++;
            
            gridInfo.cells[idx - gridInfo.xnum + 1].count ++;
            
            gridInfo.cells[idx - gridInfo.xnum].count ++;
        } else {
            gridInfo.cells[idx].count ++;

            gridInfo.cells[idx - 1].count ++;

            gridInfo.cells[idx + 1].count ++;

            gridInfo.cells[idx - gridInfo.xnum - 1].count ++;

            gridInfo.cells[idx - gridInfo.xnum + 1].count ++;

            gridInfo.cells[idx - gridInfo.xnum].count ++;

            if (idx + gridInfo.xnum + 1 < gridInfo.gridnum) {
              gridInfo.cells[idx + gridInfo.xnum + 1].count ++;
            }

            gridInfo.cells[idx + gridInfo.xnum - 1].count ++;

            gridInfo.cells[idx + gridInfo.xnum].count ++;
        }
      } else {
        // TO DO
      }            
    }
  }

  std::string cells(gridnum, '0');
  int cellIdx = 0;
  for(int i = gridInfo.ynum - 1; i >= 0; i--) {
    for(int j = 0; j < gridInfo.xnum; j++) {
      int idx = i * gridInfo.xnum + j;
      if (gridInfo.cells[idx].count > 1) {
        gridInfo.cells[idx].isPassable = false;
        cells[cellIdx++] = '1';
      } else {
        cells[cellIdx++] = '0';
      }
    }
  }
  freespace.set_cells(cells);

  std::string output_str;
  freespace.SerializeToString(&output_str);
  return py::bytes(output_str);
}


py::bytes detection_new(py::array_t<float> &input, int num) {
  // Timing all_time;
  // all_time.start();   
  GridInfo gridInfo = GridInfo(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_, resolution_);
  Freespace freespace;
  auto info = freespace.mutable_info();
  info->set_x_min(x_min_);
  info->set_x_max(x_max_);
  info->set_y_min(y_min_);
  info->set_y_max(y_max_);
  info->set_z_min(z_min_);
  info->set_z_max(z_max_);
  info->set_resolution(resolution_);
  int xnum = floor((x_max_ - x_min_) / resolution_);
  int ynum = floor((y_max_ - y_min_) / resolution_);
  info->set_x_num(xnum);
  info->set_y_num(ynum);
  int gridnum = xnum * ynum;

  for (auto& cell : gridInfo.cells) {
    cell.count = 0;
    cell.isPassable = true;
  }

  auto ref_input = input.unchecked<2>();
  PointCloudPtr origin_pd_ptr;
  origin_pd_ptr.reset(new PointCloud());
  pcl::PointXYZI point;
  for (int i = 0; i < num; i++) {
    point.x = ref_input(i, 0);
    point.y = ref_input(i, 1);
    point.z = ref_input(i, 2);
    origin_pd_ptr->points.push_back(point);
  }
  // Timing freespace_time;
  // freespace_time.start();  
  AttitudeAngles attitude_angles;
  freespace_detection_ptr->Run(origin_pd_ptr, attitude_angles);
  gridInfo = freespace_detection_ptr->GetGridInfo();
  // freespace_time.stop();


  std::string cells(gridnum, '0');
  int cellIdx = 0;
  for (int i = 0; i < gridnum; ++i) {
    int x = i / ynum;
    int y = i % ynum;
    y = ynum - 1 - y;
    int index = y * xnum + x;
    gridInfo.is_Passable[i] == 0 ? cells[index] = '1' : cells[index] = '0';
  }

  freespace.set_cells(cells);

  std::string output_str;
  freespace.SerializeToString(&output_str);
  // all_time.stop();
  // std::cout << "freespace_all_time: " << all_time.get_time() << " ms" << std::endl;  
  // std::cout << "freespace_detection_time: " << freespace_time.get_time() << " ms" << std::endl;  
  // std::cout << "processing_data_time: " << all_time.get_time() - freespace_time.get_time() << " ms" << std::endl;  
  return py::bytes(output_str);
}

PYBIND11_MODULE(freespace_detection_wrapper, m) {
    m.doc() = "freespace detection python interface";
    m.def("set_grid_detection_params", &set_grid_detection_params, 
        "set_grid_detection_params", py::arg("x_min"), py::arg("x_max"), 
        py::arg("y_min"), py::arg("y_max"), 
        py::arg("z_min"), py::arg("z_max"),
        py::arg("grid_resolution"));
    m.def("detection", &detection_new, "detection",
        py::arg("input"), py::arg("num"));
}
