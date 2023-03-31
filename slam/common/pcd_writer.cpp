#include "pcd_writer.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "Logger.h"

std::string generateHeaderXYZI() {
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS x y z intensity"
         "\nSIZE 4 4 4 4"
         "\nTYPE F F F F"
         "\nCOUNT 1 1 1 1"
         "\nWIDTH 0"
         "\nHEIGHT 1"
         "\nVIEWPOINT 0 0 0 1 0 0 0"
         "\nPOINTS 0"
         "\nDATA ascii"
         "\n";
  return (oss.str ());
}

std::string generateHeaderXYZRGB() {
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS x y z rgb"
         "\nSIZE 4 4 4 4"
         "\nTYPE F F F U"
         "\nCOUNT 1 1 1 1"
         "\nWIDTH 0"
         "\nHEIGHT 1"
         "\nVIEWPOINT 0 0 0 1 0 0 0"
         "\nPOINTS 0"
         "\nDATA ascii"
         "\n";
  return (oss.str ());
}

void PCDWriter::write(const std::string &file_name, const pcl::PointCloud<Point> &cloud) {
  std::ofstream os;
  os.precision (6);
  os.imbue (std::locale::classic ());
  os.open (file_name.c_str ());
  if (!os.is_open()) {
    LOG_ERROR("Can not open {}", file_name);
    return;
  }

  os << generateHeaderXYZI();
  os.close();
}

void PCDWriter::write(const std::string &file_name, const pcl::PointCloud<PointRGB> &cloud) {
  std::ofstream os;
  os.precision (6);
  os.imbue (std::locale::classic ());
  os.open (file_name.c_str ());
  if (!os.is_open()) {
    LOG_ERROR("Can not open {}", file_name);
    return;
  }

  os << generateHeaderXYZRGB();
  os.close();
}
