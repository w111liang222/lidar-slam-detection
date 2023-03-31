#ifndef __PCD_WRITER_H
#define __PCD_WRITER_H

#include <pcl/io/pcd_io.h>
#include "mapping_types.h"

class PCDWriter {
 public:
  static void write(const std::string &file_name, const pcl::PointCloud<Point> &cloud);
  static void write(const std::string &file_name, const pcl::PointCloud<PointRGB> &cloud);
  PCDWriter() {};
  virtual ~PCDWriter() {}
};

template <typename PointT>
void savePCDFile(const std::string &file_name, const pcl::PointCloud<PointT> &cloud) {
  if (!cloud.empty()) {
    pcl::io::savePCDFileBinary(file_name, cloud);
  } else {
    PCDWriter::write(file_name, cloud);
  }
}

#endif // __PCD_WRITER_H
