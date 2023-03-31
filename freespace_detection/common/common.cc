#include "common.h"


int NUM_ZEROS = 5;

double VEGETATION_THR = - SENSOR_HEIGHT * 3 / 4;

std::vector<int> outlier_classes = {UNLABELED, OUTLIER};
std::vector<int> ground_classes = {ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING, VEGETATION, TERRAIN};
std::vector<int> ground_classes_except_terrain = {ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING};

void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr dst){
  dst->points.clear();
  for (const auto &pt: src.points){
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x = pt.x;
    pt_xyzi.y = pt.y;
    pt_xyzi.z = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }
}

int CountNumGround(const pcl::PointCloud<PointXYZILID>& pc){
  int num_ground = 0;

  std::vector<int>::iterator iter;

  for (auto const& pt: pc.points){
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label == VEGETATION){
        if (pt.z < VEGETATION_THR){
           num_ground++;
        }
      }else num_ground++;
    }
  }
  return num_ground;
}

std::map<int, int> SetInitialGtCounts(std::vector<int>& gt_classes){
  std::map<int, int> gt_counts;
  for (int i = 0; i< gt_classes.size(); ++i){
    gt_counts.insert(std::pair<int,int>(gt_classes.at(i), 0));
  }
  return gt_counts;
}

std::map<int, int> CountNumEachClass(const pcl::PointCloud<PointXYZILID>& pc){
  int num_ground = 0;
  auto gt_counts = SetInitialGtCounts(ground_classes);
  std::vector<int>::iterator iter;

  for (auto const& pt: pc.points){
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label == VEGETATION){
        if (pt.z < VEGETATION_THR){
           gt_counts.find(pt.label)->second++;
        }
      }else gt_counts.find(pt.label)->second++;
    }
  }
  return gt_counts;
}

int CountNumOutliers(const pcl::PointCloud<PointXYZILID>& pc){
  int num_outliers = 0;

  std::vector<int>::iterator iter;
  for (auto const& pt: pc.points){
    iter = std::find(outlier_classes.begin(), outlier_classes.end(), pt.label);
    if (iter != outlier_classes.end()){ // corresponding class is in ground classes
      num_outliers ++;
    }
  }
  return num_outliers;
}

void DiscernGround(const pcl::PointCloud<PointXYZILID>& src, 
                   pcl::PointCloud<PointXYZILID>& ground, 
                   pcl::PointCloud<PointXYZILID>& non_ground){
  ground.clear();
  non_ground.clear();
  std::vector<int>::iterator iter;
  for (auto const& pt: src.points){
    if (pt.label == UNLABELED || pt.label == OUTLIER) continue;
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()){ // corresponding class is in ground classes
      if (pt.label == VEGETATION){
        if (pt.z < VEGETATION_THR){
          ground.push_back(pt);
        }else non_ground.push_back(pt);
      }else  ground.push_back(pt);
    }else{
      non_ground.push_back(pt);
    }
  }
}

void CalculatePrecisionRecall(const pcl::PointCloud<PointXYZILID>& pc_curr,
                                pcl::PointCloud<PointXYZILID>& ground_estimated,
                                double & precision,
                                double& recall,
                                bool consider_outliers=true) {

  int num_ground_est = ground_estimated.points.size();
  int num_ground_gt = CountNumGround(pc_curr);
  int num_TP = CountNumGround(ground_estimated);
  if (consider_outliers){
    int num_outliers_est = CountNumOutliers(ground_estimated);
    precision = (double)(num_TP)/(num_ground_est - num_outliers_est) * 100;
    recall = (double)(num_TP)/num_ground_gt * 100;
  }else{
    precision = (double)(num_TP)/num_ground_est * 100;
    recall = (double)(num_TP)/num_ground_gt * 100;
  }
}

void SaveAllLabels(const pcl::PointCloud<PointXYZILID>& pc, 
                  std::string ABS_DIR, std::string seq, int count){

  std::string count_str = std::to_string(count);
  std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
  std::string output_filename = ABS_DIR + "/" + seq + "/" + count_str_padded + ".csv";
  std::ofstream sc_output(output_filename);

  std::vector<int> labels(NUM_ALL_CLASSES, 0);
  for (auto const& pt: pc.points){
    if (pt.label == 0) labels[0]++;
    else if (pt.label == 1) labels[1]++;
    else if (pt.label == 10) labels[2]++;
    else if (pt.label == 11) labels[3]++;
    else if (pt.label == 13) labels[4]++;
    else if (pt.label == 15) labels[5]++;
    else if (pt.label == 16) labels[6]++;
    else if (pt.label == 18) labels[7]++;
    else if (pt.label == 20) labels[8]++;
    else if (pt.label == 30) labels[9]++;
    else if (pt.label == 31) labels[10]++;
    else if (pt.label == 32) labels[11]++;
    else if (pt.label == 40) labels[12]++;
    else if (pt.label == 44) labels[13]++;
    else if (pt.label == 48) labels[14]++;
    else if (pt.label == 49) labels[15]++;
    else if (pt.label == 50) labels[16]++;
    else if (pt.label == 51) labels[17]++;
    else if (pt.label == 52) labels[18]++;
    else if (pt.label == 60) labels[19]++;
    else if (pt.label == 70) labels[20]++;
    else if (pt.label == 71) labels[21]++;
    else if (pt.label == 72) labels[22]++;
    else if (pt.label == 80) labels[23]++;
    else if (pt.label == 81) labels[24]++;
    else if (pt.label == 99) labels[25]++;
    else if (pt.label == 252) labels[26]++;
    else if (pt.label == 253) labels[27]++;
    else if (pt.label == 254) labels[28]++;
    else if (pt.label == 255) labels[29]++;
    else if (pt.label == 256) labels[30]++;
    else if (pt.label == 257) labels[31]++;
    else if (pt.label == 258) labels[32]++;
    else if (pt.label == 259) labels[33]++;
  }

  for (uint8_t i=0; i < NUM_ALL_CLASSES;++i){
    if (i!=33){
      sc_output<<labels[i]<<",";
    }else{
      sc_output<<labels[i]<<std::endl;
    }
  }
  sc_output.close();
}

void SaveAllAccuracy(const pcl::PointCloud<PointXYZILID>& pc_curr,
                      pcl::PointCloud<PointXYZILID>& ground_estimated, 
                      std::string acc_filename, double& accuracy, 
                      std::map<int, int>&pc_curr_gt_counts, 
                      std::map<int, int>&g_est_gt_counts) {

//  std::cout<<"debug: "<<acc_filename<<std::endl;
  std::ofstream sc_output2(acc_filename, std::ios::app);

  int num_True = CountNumGround(pc_curr);
  int num_outliers_gt = CountNumOutliers(pc_curr);
  int num_outliers_est = CountNumOutliers(ground_estimated);

  int num_total_est = ground_estimated.points.size() - num_outliers_est;
  int num_total_gt = pc_curr.points.size() - num_outliers_gt;

  int num_False = num_total_gt - num_True;
  int num_TP = CountNumGround(ground_estimated);
  int num_FP = num_total_est - num_TP;
  accuracy = static_cast<double>(num_TP + (num_False - num_FP)) / num_total_gt * 100.0;

  pc_curr_gt_counts = CountNumEachClass(pc_curr);
  g_est_gt_counts = CountNumEachClass(ground_estimated);

  // save output
  for (auto const& class_id: ground_classes){
    sc_output2 << g_est_gt_counts.find(class_id)->second << "," 
    << pc_curr_gt_counts.find(class_id)->second << ",";
  }
  sc_output2 << accuracy << std::endl;

  sc_output2.close();
}

void pc2pcdfile(const pcl::PointCloud<PointXYZILID>& TP, const pcl::PointCloud<PointXYZILID>& FP,
                const pcl::PointCloud<PointXYZILID>& FN, const pcl::PointCloud<PointXYZILID>& TN,
                std::string pcd_filename){
  pcl::PointCloud<pcl::PointXYZI> pc_out;

  for (auto const pt: TP.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = TRUEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt: FP.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = FALSEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt: FN.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = FALSENEGATIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt: TN.points){
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x; pt_est.y = pt.y; pt_est.z = pt.z;
    pt_est.intensity = TRUENEGATIVE;
    pc_out.points.push_back(pt_est);
  }
  pc_out.width = pc_out.points.size();
  pc_out.height = 1;
  pcl::io::savePCDFileASCII(pcd_filename, pc_out);

}