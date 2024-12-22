#include "lidar_driver.h"

#include <sys/prctl.h>

#include "SystemUtils.h"
#include "UDPServer.h"

namespace LIDAR {

#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

static const double Ang2Rad = 0.01745329251994;
static const uint64_t kPointNumMax = 500000;

const uint8_t kLineNumberMid360 = 4;

void LidarDriver::veloRun() {
  std::string name = std::to_string(AffinityCpu) + "-" + getLidarNameByType(LidarType);
  prctl(PR_SET_NAME, name.c_str(), 0, 0, 0);
  if (AffinityCpu != -1) {
    // setSelfThreadAffinity(AffinityCpu + 1);
  }
  LOG_INFO("start lidar: {}, receveing, port {}", getLidarNameByType(LidarType), port);

  std::unique_ptr<UDPServer> veloUDPServer(new UDPServer(port));
  static std::unique_ptr<UDPServer> veloUDPSender(new UDPServer(0));

  std::unique_ptr<char> buf(new char[65536]);
  
  while (!threadStopFlag) {
    auto clock = std::chrono::steady_clock::now();
    receveSize = veloUDPServer->UDPServerReceive(buf.get(), 65536);
    auto elapseMs = since(clock).count();
    if (elapseMs >= 20) {
      LOG_WARN("lidar {}, receive buffer wait for {} ms", name, elapseMs);
    }

    if (startTransfer) {
      int sendSize = veloUDPSender->UDPSendtoBuf(destinationIp, port, buf.get(), receveSize);
      if (sendSize != receveSize) {
        LOG_ERROR("package transfer send error, {} != {}", sendSize, receveSize);
      }
    }

    clock = std::chrono::steady_clock::now();
    parseFun(buf.get());
    elapseMs = since(clock).count();
    if (elapseMs >= 10) {
      LOG_WARN("lidar {} parse message costs {} ms", name, elapseMs);
    }
  }
}

void LidarDriver::setParseFun() {
  switch (LidarType) {
    case lidarType::VLP_16: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_VLP_16, this,
                           std::placeholders::_1);
      packageLenth = 1206;
      break;
    }
    case lidarType::LS_C_16: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_LS_C_16, this,
                           std::placeholders::_1);
      packageLenth = 1206;
      lastAzimuth = 0.0;
      isFirstSweep = true;
      packetStartTime = 0.0;
      break;
    }
    case lidarType::RS_LiDAR_16: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_RS_LiDAR_16, this,
                           std::placeholders::_1);
      packageLenth = 1248;
      azimuthForRS16 = -36001;
      break;
    }
    case lidarType::RS_LiDAR_32: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_RS_LiDAR_32, this,
                           std::placeholders::_1);
      packageLenth =  1248;
      azimuthForRS32 = -36001;
      break;
    }
    case lidarType::RS_Ruby_Lite: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_RS_Ruby_Lite, this,
                           std::placeholders::_1);
      packageLenth =  1248;
      azimuthForRS80 = -36001;
      break;
    }
    case lidarType::RS_Helios_16P: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_RS_Helios_16P, this,
                           std::placeholders::_1);
      packageLenth =  1248;
      azimuthForRSHelios16P = -36001;
      rs_decode_difop_ = std::make_shared<RsDecodeDifop>(
                         getLidarNameByType(LidarType), packageLenth);
      break;
    }
    case lidarType::RS_Helios: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_RS_Helios, this,
                           std::placeholders::_1);
      packageLenth =  1248;
      azimuthForRSHelios = -36001;
      rs_decode_difop_ = std::make_shared<RsDecodeDifop>(
                         getLidarNameByType(LidarType), packageLenth);
      break;
    }
    case lidarType::RS_LiDAR_M1: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_RS_LiDAR_M1, this,
                           std::placeholders::_1);
      packageLenth =  1210;
      rs_decode_difop_ = std::make_shared<RsDecodeDifop>(
                         getLidarNameByType(LidarType), packageLenth);
      break;
    }
    case lidarType::Ouster_OS1: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_Ouster_OS, this,
                           std::placeholders::_1);
      setOusterCfgMap();                           
      break;
    }  
    case lidarType::Ouster_OS2: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_Ouster_OS, this,
                           std::placeholders::_1);
      setOusterCfgMap(); 
      break;
    }        
    case lidarType::Livox_Mid_360: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_Livox_Mid_360, this,
                           std::placeholders::_1);
      livox_offset_ = std::make_unique<LivoxLidarPacketOffsetInfo>();
      break;
    }       
    case lidarType::Custom: {
      parseFun = std::bind(&LIDAR::LidarDriver::packagePrase_Custom, this,
                           std::placeholders::_1);
      packageLenth = 1206;
      break;
    }
    default: {
      LOG_ERROR("No support lidar type {}", LidarType);
      abort();
      break;
    }
  }
}

void LidarDriver::setOusterCfgMap() {
  OusterCfg cfg;
  // 6400 for ouster 32 v3
  cfg.n = LidarType == lidarType::Ouster_OS1? 15.806 : 13.762;
  cfg.rings = 32;
  cfg.z_offset = LidarType == lidarType::Ouster_OS1? 0.038195 : 0.074296;
  cfg.beamAzimuthAngles = std::vector<double>(beamAzimuthAngles32_v3, 
                          beamAzimuthAngles32_v3 + 32);
  cfg.beamAltitudeAngles = std::vector<double>(beamAltitudeAngles32_v3, 
                            beamAltitudeAngles32_v3 + 32);
  os_cfg_maps_.insert({6400, cfg});
  os_parsefun_maps_.insert(
    {6400, std::bind(&LIDAR::LidarDriver::packagePrase_Ouster_V3, this,
                      std::placeholders::_1, std::placeholders::_2)});

  // 6464 for ouster 32
  cfg.n = LidarType == lidarType::Ouster_OS1? 15.806 : 13.762;
  cfg.rings = 32;
  cfg.z_offset = LidarType == lidarType::Ouster_OS1? 0.038195 : 0.074296;
  cfg.beamAzimuthAngles = std::vector<double>(beamAzimuthAngles32, 
                          beamAzimuthAngles32 + 32);
  cfg.beamAltitudeAngles = std::vector<double>(beamAltitudeAngles32, 
                            beamAltitudeAngles32 + 32);
  os_cfg_maps_.insert({6464, cfg});
  os_parsefun_maps_.insert(
    {6464, std::bind(&LIDAR::LidarDriver::packagePrase_Ouster, this,
                      std::placeholders::_1, std::placeholders::_2)});  

  // 12544 for ouster 64 v3
  cfg.n = LidarType == lidarType::Ouster_OS1? 15.806 : 13.762;
  cfg.rings = 64;
  cfg.z_offset = LidarType == lidarType::Ouster_OS1? 0.038195 : 0.074296;
  cfg.beamAzimuthAngles = std::vector<double>(beamAzimuthAngles64_v3, 
                          beamAzimuthAngles64_v3 + 64);
  cfg.beamAltitudeAngles = std::vector<double>(beamAltitudeAngles64_v3, 
                            beamAltitudeAngles64_v3 + 64);
  os_cfg_maps_.insert({12544, cfg});
  os_parsefun_maps_.insert(
    {12544, std::bind(&LIDAR::LidarDriver::packagePrase_Ouster_V3, this,
                      std::placeholders::_1, std::placeholders::_2)});  
  
  // 12608 for ouster 64
  cfg.n = LidarType == lidarType::Ouster_OS1? 15.806 : 13.762;
  cfg.rings = 64;
  cfg.z_offset = LidarType == lidarType::Ouster_OS1? 0.038195 : 0.074296;
  cfg.beamAzimuthAngles = std::vector<double>(beamAzimuthAngles64, 
                          beamAzimuthAngles64 + 64);
  cfg.beamAltitudeAngles = std::vector<double>(beamAltitudeAngles64, 
                            beamAltitudeAngles64 + 64);
  os_cfg_maps_.insert({12608, cfg});
  os_parsefun_maps_.insert(
    {12608, std::bind(&LIDAR::LidarDriver::packagePrase_Ouster, this,
                      std::placeholders::_1, std::placeholders::_2)});  
  // 24832 for ouster 128 v3
  cfg.n = LidarType == lidarType::Ouster_OS1? 15.806 : 13.762;
  cfg.rings = 128;
  cfg.z_offset = LidarType == lidarType::Ouster_OS1? 0.038195 : 0.074296;
  cfg.beamAzimuthAngles = LidarType == lidarType::Ouster_OS1?
      std::vector<double>(beamAzimuthAngles_v3, beamAzimuthAngles_v3 + 128) : 
      std::vector<double>(beamAzimuthAnglesForOS2_v3, beamAzimuthAnglesForOS2_v3 + 128);
  cfg.beamAltitudeAngles = LidarType == lidarType::Ouster_OS1?
      std::vector<double>(beamAltitudeAngles_v3, beamAltitudeAngles_v3 + 128) : 
      std::vector<double>(beamAltitudeAnglesForOS2_v3, beamAltitudeAnglesForOS2_v3 + 128);
  os_cfg_maps_.insert({24832, cfg});
  os_parsefun_maps_.insert(
    {24832, std::bind(&LIDAR::LidarDriver::packagePrase_Ouster_V3, this,
                      std::placeholders::_1, std::placeholders::_2)});    

  // 24896 for ouster 128
  cfg.n = LidarType == lidarType::Ouster_OS1? 15.806 : 13.762;
  cfg.rings = 128;
  cfg.z_offset = LidarType == lidarType::Ouster_OS1? 0.038195 : 0.074296;
  cfg.beamAzimuthAngles = LidarType == lidarType::Ouster_OS1?
      std::vector<double>(beamAzimuthAngles, beamAzimuthAngles + 128) : 
      std::vector<double>(beamAzimuthAnglesForOS2, beamAzimuthAnglesForOS2 + 128);
  cfg.beamAltitudeAngles = LidarType == lidarType::Ouster_OS1?
      std::vector<double>(beamAltitudeAngles, beamAltitudeAngles + 128) : 
      std::vector<double>(beamAltitudeAnglesForOS2, beamAltitudeAnglesForOS2 + 128);
  os_cfg_maps_.insert({24896, cfg});
  os_parsefun_maps_.insert(
    {24896, std::bind(&LIDAR::LidarDriver::packagePrase_Ouster, this,
                      std::placeholders::_1, std::placeholders::_2)});
}

LidarDriver::lidarType LidarDriver::getLidarTypeByName(std::string name) {
  if (name.compare("VLP-16") == 0) {
    return lidarType::VLP_16;
  } else if (name.compare("LS-C-16") == 0) {
    return lidarType::LS_C_16;
  } else if (name.compare("Ouster-OS1") == 0) {
    return lidarType::Ouster_OS1;
  } else if (name.compare("Ouster-OS2") == 0) {
    return lidarType::Ouster_OS2;    
  } else if (name.compare("RS-LiDAR-16") == 0) {
    return lidarType::RS_LiDAR_16;
  } else if (name.compare("RS-LiDAR-32") == 0) {
    return lidarType::RS_LiDAR_32;
  } else if (name.compare("RS-Ruby-Lite") == 0) {
    return lidarType::RS_Ruby_Lite;
  } else if (name.compare("RS-Helios-16P") == 0) {
    return lidarType::RS_Helios_16P;
  } else if (name.compare("RS-Helios") == 0) {
    return lidarType::RS_Helios;
  } else if (name.compare("RS-LiDAR-M1") == 0) {
    return lidarType::RS_LiDAR_M1;
  } else if (name.compare("Livox-Mid-360") == 0) {
    return lidarType::Livox_Mid_360;    
  } else if (name.compare("Custom") == 0) {
    return lidarType::Custom;
  } else {
    return lidarType::None;
  }
}

std::string LidarDriver::getLidarNameByType(lidarType type) {
  switch (type) {
    case lidarType::VLP_16: {
      return "VLP-16";
    }
    case lidarType::LS_C_16: {
      return "LS-C-16";
    }
    case lidarType::Ouster_OS1: {
      return "Ouster-OS1";
    }  
    case lidarType::Ouster_OS2: {
      return "Ouster-OS2";
    }
    case lidarType::RS_LiDAR_16: {
      return "RS-LiDAR-16";
    }
    case lidarType::RS_LiDAR_32: {
      return "RS-LiDAR-32";
    }
    case lidarType::RS_Ruby_Lite: {
      return "RS-Ruby-Lite";
    }
    case lidarType::RS_Helios_16P: {
      return "RS-Helios-16P";
    }
    case lidarType::RS_Helios: {
      return "RS-Helios";
    }
    case lidarType::RS_LiDAR_M1: {
      return "RS-LiDAR-M1";
    }
    case lidarType::Livox_Mid_360: {
      return "Livox-Mid-360";
    }    
    case lidarType::Custom: {
      return "Custom";
    }
    default: {
      return "";
    }
  }
}

LidarDriver::LidarDriver(modeType modeIn, lidarType lidarIn, int affinityCpu) {
  veloMode = modeIn;
  LidarType = lidarIn;
  AffinityCpu = affinityCpu;

  staticTransform = Transform();
  startTransfer = false;
  xmlCorrection();
  setParseFun();
  resetRuntimeVariables();
}

LidarDriver::~LidarDriver() { stopRun(); }

void LidarDriver::resetPoints() {
  pointCloud = std::make_shared<std::vector<float>>();
  pointCloudAttr = std::make_shared<std::vector<float>>();
  pointCloud->reserve(4 * kPointNumMax);
  pointCloudAttr->reserve(2 * kPointNumMax);
}

void LidarDriver::resetRuntimeVariables() {
  resetPoints();
  while (scanQueue.pop()) {
    ;
  }
  rotAngle = 0;
  rotAngleOld = 359.0;
  scanStartTime = getCurrentTime();
  scanMonotonicTime = getMonotonicTime();
  prevFrameId = 0;
}

void LidarDriver::startRun(int portIn) {
  if (veloRunThread != nullptr) {
    return;
  }

  port = portIn;
  resetRuntimeVariables();
  threadStopFlag = false;
  if (online == veloMode) {
      veloRunThread.reset(new std::thread(&LidarDriver::veloRun, this));
      setThreadPriority(veloRunThread.get(), 99);
  }
}

void LidarDriver::stopRun() {
  if (veloRunThread == nullptr) {
    return;
  }
  threadStopFlag = true;
  // for notify the udp receiving thread
  std::unique_ptr<UDPServer> veloUDPServer(new UDPServer(0));
  veloUDPServer->UDPSendto("127.0.0.1", port, "stop", 4);
  veloRunThread->join();
  veloRunThread.reset(nullptr);
}

void LidarDriver::startPackageTransfer(std::string dest) {
  destinationIp = dest;
  startTransfer = true;
}

void LidarDriver::stopPackageTransfer() {
  startTransfer = false;
}

void LidarDriver::setExternalParameter(Transform &externalPara) {
  staticTransform = externalPara;
}

bool LidarDriver::pointsInROI(float &x, float &y, float &z) {
  pointTransform(x, y, z, staticTransform);
  if (x < filter.xmin || x > filter.xmax ||
      y < filter.ymin || y > filter.ymax ||
      z < filter.zmin || z > filter.zmax) {
    return false;
  }
  if (x > exclude.xmin && x < exclude.xmax &&
      y > exclude.ymin && y < exclude.ymax &&
      z > exclude.zmin && z < exclude.zmax) {
    return false;
  }
  return true;
}

Transform LidarDriver::getExternalParameter() {
  return staticTransform;
}

void LidarDriver::setRangeFilter(RangeFilter rangeFilter) {
  filter = rangeFilter;
}

RangeFilter LidarDriver::getRangeFilter() {
  return filter;
}

void LidarDriver::setExcludeFilter(RangeFilter excludeFilter) {
  exclude = excludeFilter;
}

void LidarDriver::packagePrase_VLP_16(char buf[1206]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:VLP-16 receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }  
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  int i = 0, j = 0;
  int index1 = 0, index2 = 0;

  float verticleAngle = 0;
  float distance = 0;
  float reflect = 0;
  unsigned char highBit = 0, lowBit = 0;

  for (i = 0; i < 12; ++i) {
    index1 = 100 * i;
    highBit = buf[index1 + 2];
    lowBit = buf[index1 + 3];
    rotAngle = (unsigned short int)((lowBit << 8) + highBit) / 100.0f;
    if (rotAngle < 0)
      rotAngle = rotAngle + 360;
    else if (rotAngle >= 360)
      rotAngle = rotAngle - 360;
    if ((rotAngleOld - rotAngle) > 0.1) {
      // new frame received
      LidarScan *scan = new LidarScan("VLP-16", scanStartTime, 2, pointCloud, pointCloudAttr);
      scanQueue.enqueue(scan);
      resetPoints();
      scanStartTime = getCurrentTime();
      scanMonotonicTime = getMonotonicTime();
    }
    rotAngleOld = rotAngle;
    //////////////////////////
    for (j = 0; j < 16; j++) {
      index2 = 3 * j + index1 + 4;
      highBit = buf[index2];
      lowBit = buf[index2 + 1];  // distance
      distance = (unsigned short int)((lowBit << 8) + highBit) * 0.2f / 100.0f;
      reflect = (unsigned char)buf[index2 + 2];
      verticleAngle = xmlData[j];

      if (distance > 0.1) {
        float x, y, z;
        x = float(distance * cos(verticleAngle * Ang2Rad) *
                  sin(rotAngle * Ang2Rad));
        y = float(distance * cos(verticleAngle * Ang2Rad) *
                  cos(rotAngle * Ang2Rad));
        z = float(distance * sin(verticleAngle * Ang2Rad));
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(reflect / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(j));
        }
      }
    }
  }
}

void LidarDriver::packagePrase_LS_C_16(char buf[1206]) {
    if (receveSize != packageLenth) {
        LOG_ERROR("{}:LS-C-16 receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
        return;
    }    
    float pointTime = float(getMonotonicTime() - scanMonotonicTime);
    const raw_packet_t* packet = (const raw_packet_t*)&buf[0];
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        if (packet->blocks[blk_idx].header != UPPER_BANK) {
            return;
        }
    }
    // Compute the azimuth angle for each firing.
    for (size_t fir_idx = 0; fir_idx < FIRINGS_PER_PACKET; fir_idx+=2) {
        size_t blk_idx = fir_idx / 2;
        firings[fir_idx].firing_azimuth =
            static_cast<double>(packet->blocks[blk_idx].rotation) / 100.0 * 0.017453292;
    }

    // Interpolate the azimuth values
    for (size_t fir_idx = 1; fir_idx < FIRINGS_PER_PACKET; fir_idx+=2) {
        size_t lfir_idx = fir_idx - 1;
        size_t rfir_idx = fir_idx + 1;

        double azimuth_diff;
        if (fir_idx == FIRINGS_PER_PACKET - 1) {
            lfir_idx = fir_idx - 3;
            rfir_idx = fir_idx - 1;
        }

        azimuth_diff = firings[rfir_idx].firing_azimuth -
                firings[lfir_idx].firing_azimuth;
        azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 2*M_PI : azimuth_diff;

        firings[fir_idx].firing_azimuth =
                firings[fir_idx-1].firing_azimuth + azimuth_diff/2.0;


        firings[fir_idx].firing_azimuth  =
                firings[fir_idx].firing_azimuth > 2*M_PI ?
                    firings[fir_idx].firing_azimuth-2*M_PI : firings[fir_idx].firing_azimuth;
    }

    // Fill in the distance and intensity for each firing.
    for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
        const raw_block_t& raw_block = packet->blocks[blk_idx];

        for (size_t blk_fir_idx = 0; blk_fir_idx < LSC16_FIRINGS_PER_BLOCK; ++blk_fir_idx){
            size_t fir_idx = blk_idx*LSC16_FIRINGS_PER_BLOCK + blk_fir_idx;

            double azimuth_diff = 0.0;
            if (fir_idx < FIRINGS_PER_PACKET - 1)
                azimuth_diff = firings[fir_idx+1].firing_azimuth -
                        firings[fir_idx].firing_azimuth;
            else
                azimuth_diff = firings[fir_idx].firing_azimuth -
                        firings[fir_idx-1].firing_azimuth;

            for (size_t scan_fir_idx = 0; scan_fir_idx < LSC16_SCANS_PER_FIRING; ++scan_fir_idx){
                size_t byte_idx = RAW_SCAN_SIZE * (
                            LSC16_SCANS_PER_FIRING*blk_fir_idx + scan_fir_idx);

                // Azimuth
                firings[fir_idx].azimuth[scan_fir_idx] = firings[fir_idx].firing_azimuth +
                        (scan_fir_idx*DSR_TOFFSET/FIRING_TOFFSET) * azimuth_diff;

                // Distance
                union two_bytes raw_distance;
                raw_distance.bytes[0] = raw_block.data[byte_idx];
                raw_distance.bytes[1] = raw_block.data[byte_idx+1];
                firings[fir_idx].distance[scan_fir_idx] = static_cast<double>(
                            raw_distance.distance) * DISTANCE_RESOLUTION;

                // Intensity
                firings[fir_idx].intensity[scan_fir_idx] = static_cast<double>(
                            raw_block.data[byte_idx+2]);
            }
        }
    }

    size_t new_sweep_start = 0;
    do {
        if (fabs(firings[new_sweep_start].firing_azimuth - lastAzimuth) > M_PI) break;
        else {
            lastAzimuth = firings[new_sweep_start].firing_azimuth;
            ++new_sweep_start;
        }
    } while (new_sweep_start < FIRINGS_PER_PACKET);

    size_t start_fir_idx = 0;
    size_t end_fir_idx = new_sweep_start;
    if (isFirstSweep &&
            new_sweep_start == FIRINGS_PER_PACKET) {
        // The first sweep has not ended yet.
        return;
    } else {
        if (isFirstSweep) {
            isFirstSweep = false;
            start_fir_idx = new_sweep_start;
            end_fir_idx = FIRINGS_PER_PACKET;
            // sweep_start_time = msg->stamp.toSec() +
            //         FIRING_TOFFSET * (end_fir_idx-start_fir_idx) * 1e-6;
        }
    }

    for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
        for (size_t scan_idx = 0; scan_idx < LSC16_SCANS_PER_FIRING; ++scan_idx) {
            // Check if the point is valid.
            if (firings[fir_idx].distance[scan_idx] < 0.2f ||
                firings[fir_idx].distance[scan_idx] > 150.0) continue;

            // Convert the point to xyz coordinate
            size_t table_idx = floor(firings[fir_idx].azimuth[scan_idx]*1000.0+0.5);
            //cout << table_idx << endl;
            double cos_azimuth = cosTableForLSC16[table_idx];
            double sin_azimuth = sinTableForLSC16[table_idx];

            //double x = firings[fir_idx].distance[scan_idx] *
            //  cos_scan_altitude[scan_idx] * sin(firings[fir_idx].azimuth[scan_idx]);
            //double y = firings[fir_idx].distance[scan_idx] *
            //  cos_scan_altitude[scan_idx] * cos(firings[fir_idx].azimuth[scan_idx]);
            //double z = firings[fir_idx].distance[scan_idx] *
            //  sin_scan_altitude[scan_idx];

            double x = firings[fir_idx].distance[scan_idx] *
                    cos_scan_altitude[scan_idx] * sin_azimuth;
            double y = firings[fir_idx].distance[scan_idx] *
                    cos_scan_altitude[scan_idx] * cos_azimuth;
            double z = firings[fir_idx].distance[scan_idx] *
                    sin_scan_altitude[scan_idx];
            double intensity = firings[fir_idx].intensity[scan_idx];

            float x_coord = y;
            float y_coord = -x;
            float z_coord = z;
            if (pointsInROI(x_coord, y_coord, z_coord)) {
                pointCloud->emplace_back(x_coord);
                pointCloud->emplace_back(y_coord);
                pointCloud->emplace_back(z_coord);
                pointCloud->emplace_back(intensity / 255.0f);
                pointCloudAttr->emplace_back(pointTime);
                pointCloudAttr->emplace_back(scan_idx);
            }

            // Compute the time of the point
            double time = packetStartTime +
                    FIRING_TOFFSET*fir_idx + DSR_TOFFSET*scan_idx;
        }
    }
    packetStartTime += FIRING_TOFFSET * (end_fir_idx-start_fir_idx);

     // A new sweep begins
    if (end_fir_idx != FIRINGS_PER_PACKET) {
        LidarScan *scan =
            new LidarScan("LS-C-16", scanStartTime, 2, pointCloud, pointCloudAttr);
        scanQueue.enqueue(scan);
        resetPoints();
        scanStartTime = getCurrentTime();
        scanMonotonicTime = getMonotonicTime();

        packetStartTime = 0.0;
        lastAzimuth = firings[FIRINGS_PER_PACKET-1].firing_azimuth;

        start_fir_idx = end_fir_idx;
        end_fir_idx = FIRINGS_PER_PACKET;

        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            for (size_t scan_idx = 0; scan_idx < LSC16_SCANS_PER_FIRING; ++scan_idx) {
                // Check if the point is valid.
                if (firings[fir_idx].distance[scan_idx] < 0.2f ||
                    firings[fir_idx].distance[scan_idx] > 150.0) continue;

                // Convert the point to xyz coordinate
                size_t table_idx = floor(firings[fir_idx].azimuth[scan_idx]*1000.0+0.5);
                //cout << table_idx << endl;
                double cos_azimuth = cosTableForLSC16[table_idx];
                double sin_azimuth = sinTableForLSC16[table_idx];

                double x = firings[fir_idx].distance[scan_idx] *
                        cos_scan_altitude[scan_idx] * sin_azimuth;
                double y = firings[fir_idx].distance[scan_idx] *
                        cos_scan_altitude[scan_idx] * cos_azimuth;
                double z = firings[fir_idx].distance[scan_idx] *
                        sin_scan_altitude[scan_idx];
                double intensity = firings[fir_idx].intensity[scan_idx];

                float x_coord = y;
                float y_coord = -x;
                float z_coord = z;

                // Compute the time of the point
                double time = packetStartTime +
                        FIRING_TOFFSET*(fir_idx-start_fir_idx) + DSR_TOFFSET*scan_idx;

                if (pointsInROI(x_coord, y_coord, z_coord)) {
                    pointCloud->emplace_back(x_coord);
                    pointCloud->emplace_back(y_coord);
                    pointCloud->emplace_back(z_coord);
                    pointCloud->emplace_back(intensity / 255.0f);
                    pointCloudAttr->emplace_back(0);
                    pointCloudAttr->emplace_back(scan_idx);
                }
            }
        }

        packetStartTime += FIRING_TOFFSET * (end_fir_idx-start_fir_idx);
    }
}

void LidarDriver::packagePrase_Ouster_OS(char buf[]) {
  if (os_cfg_maps_.find(receveSize) != os_cfg_maps_.end()) {
    os_parsefun_maps_[receveSize](buf, os_cfg_maps_[receveSize]);
  } else {
    LOG_ERROR("{}:{} receive wrong size package len {}", 
              AffinityCpu, getLidarNameByType(LidarType), receveSize);
    return;
  }
}

void LidarDriver::packagePrase_Ouster(char buf[], const OusterCfg& cfg) {
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  const int column_bytes = 16 + (cfg.rings * 12) + 4;
  const double n = cfg.n;  // lidar_origin_to_beam_origin_mm
  int h = cfg.rings;
  int w = 1024;
  int next_m_id{w};
  int cur_m_id = 0;
  int32_t cur_f_id{-1};

  for (int i = 0; i < 16; i++) {
    uint8_t *col_buf = (uint8_t *)(buf + i * column_bytes);
    uint64_t time_stamp;
    std::memcpy(&time_stamp, col_buf, sizeof(uint64_t));

    uint16_t m_id;
    std::memcpy(&m_id, col_buf + 8, sizeof(uint16_t));
    cur_m_id = m_id;

    uint16_t f_id;
    memcpy(&f_id, col_buf + 10, sizeof(uint16_t));

    uint32_t is_valid;
    memcpy(&is_valid, col_buf + column_bytes - 4, sizeof(uint32_t));
    const bool valid = is_valid == 0xffffffff;

    uint32_t encoder_count;
    std::memcpy(&encoder_count, col_buf + 12, sizeof(uint32_t));

    if (!valid || m_id >= w || f_id + 1 == cur_f_id) continue;

    if (f_id != cur_f_id) {
      next_m_id = 0;
      cur_f_id = f_id;
    }

    for (uint8_t ipx = 0; ipx < h; ipx++) {
      const uint8_t *px_buf = col_buf + 16 + ipx * 12;
      uint32_t range;
      std::memcpy(&range, px_buf, sizeof(uint32_t));
      range &= 0x000fffff;

      uint16_t intensity;
      std::memcpy(&intensity, px_buf + 6, sizeof(uint16_t));

      uint16_t reflectivity;
      std::memcpy(&reflectivity, px_buf + 4, sizeof(uint16_t));

      uint16_t ambient;
      std::memcpy(&ambient, px_buf + 8, sizeof(uint16_t));

      uint8_t ring = ipx;
      const double r = static_cast<double>(range);

      float px, py, pz;
      std::ptrdiff_t u = ipx;
      std::ptrdiff_t v = m_id;
      double encoder = -1.0 * 2 * M_PI * encoder_count / 90112;
      double azimuth = -1.0 * cfg.beamAzimuthAngles[u] * M_PI / 180.0;
      double altitude = cfg.beamAltitudeAngles[u] * M_PI / 180.0;
      double x = (r - n) * cos(azimuth + encoder) * cos(altitude) + n * cos(encoder);
      px = static_cast<float>(x / 1000.0);
      double y = (r - n) * sin(azimuth + encoder) * cos(altitude) + n * sin(encoder);
      py = static_cast<float>(y / 1000.0);
      double z = (r - n) * sin(altitude);
      pz = static_cast<float>(z / 1000.0);

      px = -px;
      py = -py;
      pz = pz + cfg.z_offset;

      if (pointsInROI(px, py, pz)) {
        pointCloud->emplace_back(px);
        pointCloud->emplace_back(py);
        pointCloud->emplace_back(pz);
        pointCloud->emplace_back(std::min(1.0f, intensity / 2048.0f));
        pointCloudAttr->emplace_back(pointTime);
        pointCloudAttr->emplace_back(float(ring));
      }
    }
  }
  if (cur_m_id == 1023) {  // 1023 or 2047
    std::string lidar_name = getLidarNameByType(LidarType) + "-" + std::to_string(cfg.rings);
    LidarScan *scan = new LidarScan(lidar_name, scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
  }
}

void LidarDriver::packagePrase_Ouster_V3(char buf[], const OusterCfg& cfg) {
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  const int column_bytes = 12 + (cfg.rings * 12);
  const double n = cfg.n;  // lidar_origin_to_beam_origin_mm
  int h = cfg.rings;
  int w = 1024;
  int next_m_id{w};
  int cur_m_id = 0;
  int32_t cur_f_id{-1};
  uint16_t packet_type, f_id;
  uint32_t init_id;
  std::memcpy(&packet_type, buf, sizeof(uint16_t));
  std::memcpy(&cur_f_id, buf + 2, sizeof(uint16_t));
  // std::memcpy(&init_id, buf + 4, sizeof(uint32_t));
  // uint32_t init_id_mask = (1 << 24) - 1;
  // init_id &= init_id_mask;

  uint8_t *col_buf = nullptr;
  uint64_t time_stamp;
  uint16_t m_id, status, reflectivity, signal, NIR;
  uint32_t range;
  uint8_t ring;
  float px, py, pz;
  std::ptrdiff_t u, v;
  double encoder, azimuth, altitude, x, y, z;  

  for (int i = 0; i < 16; i++) {
    col_buf = (uint8_t *)(buf + 32 + i * column_bytes);
    std::memcpy(&time_stamp, col_buf, sizeof(uint64_t));
    std::memcpy(&m_id, col_buf + 8, sizeof(uint16_t));
    cur_m_id = m_id;

    memcpy(&status, col_buf + 10, sizeof(uint16_t));

    if (m_id >= w || f_id + 1 == cur_f_id) continue;

    if (f_id != cur_f_id) {
      next_m_id = 0;
      cur_f_id = f_id;
    }

    for (uint8_t ipx = 0; ipx < h; ipx++) {
      const uint8_t *px_buf = col_buf + 12 + ipx * 12;
      
      std::memcpy(&range, px_buf, sizeof(uint32_t));
      std::memcpy(&reflectivity, px_buf + 4, sizeof(uint16_t));
      std::memcpy(&signal, px_buf + 6, sizeof(uint16_t));
      std::memcpy(&NIR, px_buf + 8, sizeof(uint16_t));

      ring = ipx;
      const double r = static_cast<double>(range);
      
      u = ipx;
      v = m_id;
      encoder = (1 - m_id / 1024.00) * 2 * M_PI;    
      azimuth = -1.0 * cfg.beamAzimuthAngles[u] * M_PI / 180.0;
      altitude = cfg.beamAltitudeAngles[u] * M_PI / 180.0;
      x = (r - n) * cos(azimuth + encoder) * cos(altitude) + n * cos(encoder);
      px = static_cast<float>(x / 1000.0);
      y = (r - n) * sin(azimuth + encoder) * cos(altitude) + n * sin(encoder);
      py = static_cast<float>(y / 1000.0);
      z = (r - n) * sin(altitude);
      pz = static_cast<float>(z / 1000.0);

      px = -px;
      py = -py;
      pz = pz + cfg.z_offset;

      if (pointsInROI(px, py, pz)) {
        pointCloud->emplace_back(px);
        pointCloud->emplace_back(py);
        pointCloud->emplace_back(pz);
        pointCloud->emplace_back(std::min(1.0f, signal / 2048.0f));
        pointCloudAttr->emplace_back(pointTime);
        pointCloudAttr->emplace_back(float(ring));
      }
    }
  }
  if (cur_m_id == 1023) {  // 1023 or 2047
    std::string lidar_name = getLidarNameByType(LidarType) + "-" + std::to_string(cfg.rings);
    LidarScan *scan = new LidarScan(lidar_name, scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
  }
}

void LidarDriver::packagePrase_RS_LiDAR_16(char buf[1248]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:RS-LiDAR-16 receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }   
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  const RS16MsopPkt* mpkt_ptr = reinterpret_cast<const RS16MsopPkt*>(buf);
  int start_angle = 0;
  int end_angle = 36000;
  float max_distance = 200.0f;
  float min_distance = 0.2f;
  uint64_t msop_id = 0xA050A55A0A05AA55;
  uint16_t block_id = 0xEEFF;
  constexpr int RS_ONE_ROUND = 36000;
  if (mpkt_ptr->header.id != msop_id) {
    return;
  }
  float azi_diff = 0;
  int azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  for (size_t blk_idx = 0; blk_idx < 12; blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != block_id) {
      break;
    }

    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (blk_idx == 0) {
      azi_diff = static_cast<float>((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) % RS_ONE_ROUND);
    } else {
      azi_diff = static_cast<float>((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) % RS_ONE_ROUND);
    }

    for (size_t channel_idx = 0; channel_idx < 32; channel_idx++) {
      float azi_channel_ori = cur_azi + azi_diff * ((2.8 * 0.009 * static_cast<float>(channel_idx % 16)) +
                                  static_cast<float>(channel_idx / 16) * 0.5f);

      int azi_channel_final = (static_cast<int>(azi_channel_ori) + horiAangleList16[channel_idx % 16] + RS_ONE_ROUND) % RS_ONE_ROUND;
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * 0.005;
      int angle_horiz_ori = static_cast<int>(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = ((vertAangleList16[channel_idx % 16]) + RS_ONE_ROUND) % RS_ONE_ROUND;

      if ((distance <= max_distance && distance >= min_distance) &&
          (azi_channel_final >= start_angle && azi_channel_final <= end_angle)) {

        float x = distance * cosTableForRS16[angle_vert + RS_ONE_ROUND] * cosTableForRS16[azi_channel_final + RS_ONE_ROUND] +
                  0.03825 * cosTableForRS16[angle_horiz_ori + RS_ONE_ROUND];

        float y = -distance *  cosTableForRS16[angle_vert + RS_ONE_ROUND] * sinTableForRS16[azi_channel_final + RS_ONE_ROUND] -
                  0.03825 * sinTableForRS16[angle_horiz_ori + RS_ONE_ROUND];

        float z = distance * sinTableForRS16[angle_vert + RS_ONE_ROUND];

        uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(intensity / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(channel_idx % 16));
        }
      }
    }
  }
  if (azimuth < azimuthForRS16) {
    azimuthForRS16 -= RS_ONE_ROUND;
  }
  if (azimuthForRS16 != -36001 && azimuthForRS16 < 0 && azimuth >= 0) {
    azimuthForRS16 = azimuth;
    LidarScan *scan = new LidarScan("RS-LiDAR-16", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
    return;
  }
  azimuthForRS16 = azimuth;
}

void LidarDriver::packagePrase_RS_LiDAR_32(char buf[1248]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:RS-LiDAR-32 receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }  
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  const RS32MsopPkt* mpkt_ptr = reinterpret_cast<const RS32MsopPkt*>(buf);
  int start_angle = 0;
  int end_angle = 36000;
  float max_distance = 200.0f;
  float min_distance = 0.2f;
  uint64_t msop_id = 0xA050A55A0A05AA55;
  uint16_t block_id = 0xEEFF;
  constexpr int RS_ONE_ROUND = 36000;
  if (mpkt_ptr->header.id != msop_id) {
    return;
  }
  float azi_diff = 0;
  int azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  for (size_t blk_idx = 0; blk_idx < 12; blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != block_id) {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (blk_idx == 0) {
      azi_diff = static_cast<float>((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) % RS_ONE_ROUND);
    } else {
      azi_diff = static_cast<float>((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) % RS_ONE_ROUND);
    }
    for (size_t channel_idx = 0; channel_idx < 32; channel_idx++) {
      float azi_channel_ori = cur_azi + azi_diff * 0.018 * 1.44 *
                              static_cast<float>(2 * (channel_idx % 16) + (channel_idx / 16));

      int azi_channel_final = (static_cast<int>(azi_channel_ori) + horiAangleList32[channel_idx] + RS_ONE_ROUND) % RS_ONE_ROUND;
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * 0.005;
      int angle_horiz_ori = static_cast<int>(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = ((vertAangleList32[channel_idx]) + RS_ONE_ROUND) % RS_ONE_ROUND;

      if ((distance <= max_distance && distance >= min_distance) &&
          (azi_channel_final >= start_angle && azi_channel_final <= end_angle)) {
        float x = distance * cosTableForRS16[angle_vert + RS_ONE_ROUND] * cosTableForRS16[azi_channel_final + RS_ONE_ROUND] +
                  0.03997 * cosTableForRS16[angle_horiz_ori + RS_ONE_ROUND];
        float y = -distance *  cosTableForRS16[angle_vert + RS_ONE_ROUND] * sinTableForRS16[azi_channel_final + RS_ONE_ROUND] -
                  0.03997 * sinTableForRS16[angle_horiz_ori + RS_ONE_ROUND];
        float z = distance * sinTableForRS16[angle_vert + RS_ONE_ROUND];
        uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(intensity / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(channel_idx));
        }
      }
    }
  }
  if (azimuth < azimuthForRS32) {
    azimuthForRS32 -= RS_ONE_ROUND;
  }
  if (azimuthForRS32 != -36001 && azimuthForRS32 < 0 && azimuth >= 0) {
    azimuthForRS32 = azimuth;
    LidarScan *scan = new LidarScan("RS-LiDAR-32", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
    return;
  }
  azimuthForRS32 = azimuth;
}

void LidarDriver::packagePrase_RS_Ruby_Lite(char buf[1248]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:RS-Ruby-Lite receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }     
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  const RS80MsopPkt* mpkt_ptr = reinterpret_cast<const RS80MsopPkt*>(buf);
  int start_angle = 0;
  int end_angle = 36000;
  float max_distance = 200.0f;
  float min_distance = 0.2f;
  float azi_diff_between_block_theoretical = 20;
  uint64_t msop_id = 0x5A05AA55;
  uint16_t block_id = 0xFE;
  constexpr int RS_ONE_ROUND = 36000;
  if (mpkt_ptr->header.id != msop_id) {
    return;
  }
  float azi_diff = 0;
  int azimuth = RS_SWAP_SHORT(mpkt_ptr->blocks[0].azimuth);
  for (size_t blk_idx = 0; blk_idx < 4; blk_idx++) {
    if (mpkt_ptr->blocks[blk_idx].id != block_id) {
      break;
    }
    int cur_azi = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth);
    if (blk_idx == 0) {
      azi_diff = static_cast<float>((RS_ONE_ROUND + RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) % RS_ONE_ROUND);
    } else {
      azi_diff = static_cast<float>((RS_ONE_ROUND + cur_azi - RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx - 1].azimuth)) % RS_ONE_ROUND);
    }
    azi_diff = (azi_diff > 100) ? azi_diff_between_block_theoretical : azi_diff;
    for (size_t channel_idx = 0; channel_idx < 80; channel_idx++) {

      int dsr_temp = (channel_idx / 4) % 16;
      float azi_channel_ori = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].azimuth) +
                              (azi_diff * static_cast<float>(dsr_temp) * 3.236 * 0.018);
      int azi_channel_final = (static_cast<int>(azi_channel_ori) + horiAangleList80[channel_idx] + RS_ONE_ROUND) % RS_ONE_ROUND;
      float distance = RS_SWAP_SHORT(mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * 0.005;
      int angle_horiz = static_cast<int>(azi_channel_ori + RS_ONE_ROUND) % RS_ONE_ROUND;
      int angle_vert = ((vertAangleList80[channel_idx]) + RS_ONE_ROUND) % RS_ONE_ROUND;

      if ((distance <= max_distance && distance >= min_distance) &&
          (azi_channel_final >= start_angle && azi_channel_final <= end_angle)) {
        float x = distance * cosTableForRS16[angle_vert + RS_ONE_ROUND] * cosTableForRS16[azi_channel_final + RS_ONE_ROUND] +
                  0.03615 * cosTableForRS16[angle_horiz + RS_ONE_ROUND];
        float y = -distance *  cosTableForRS16[angle_vert + RS_ONE_ROUND] * sinTableForRS16[azi_channel_final + RS_ONE_ROUND] -
                  0.03615 * sinTableForRS16[angle_horiz + RS_ONE_ROUND];
        float z = distance * sinTableForRS16[angle_vert + RS_ONE_ROUND];
        uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(intensity / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(channel_idx));
        }
      }
    }
  }
  if (azimuth < azimuthForRS80) {
    azimuthForRS80 -= RS_ONE_ROUND;
  }
  if (azimuthForRS80 != -36001 && azimuthForRS80 < 0 && azimuth >= 0) {
    azimuthForRS80 = azimuth;
    LidarScan *scan = new LidarScan("RS-Ruby-Lite", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
    return;
  }
  azimuthForRS80 = azimuth;
}

void LidarDriver::packagePrase_RS_Helios_16P(char buf[1248]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:RS-Helios-16P receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }    
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  // Get rs_helios device info
  if (!is_received_difop_) {
    is_received_difop_ = rs_decode_difop_->ReceiveDifop(port + 1);
    if (is_received_difop_) {
      rs_decode_difop_->Decode();
      chan_angles_ = rs_decode_difop_->GetChanAngles();
      mech_const_param_ = rs_decode_difop_->GetMechConstParam();
    }
  }
  const RSHELIOSMsopPkt* pkt = reinterpret_cast<const RSHELIOSMsopPkt*>(buf);
  constexpr int RS_ONE_ROUND = 36000;

  int azimuth = RS_SWAP_SHORT(pkt->blocks[0].azimuth);
  for (uint16_t blk = 0; blk < mech_const_param_.base.BLOCKS_PER_PKT; blk++) {
    const RSHELIOSMsopBlock& block = pkt->blocks[blk];

    if (memcmp(mech_const_param_.base.BLOCK_ID, block.id, 2) != 0) {
      // this->cb_excep_(Error(ERRCODE_WRONGPKTHEADER));
      break;
    }

    int32_t block_az = ntohs(block.azimuth);
    int32_t block_az_diff;
    rs_decode_difop_->GetAzDiff(pkt, blk, block_az_diff);

    for (uint16_t chan = 0; chan < mech_const_param_.base.CHANNELS_PER_BLOCK; chan++) {
      const RSChannel& channel = block.channels[chan];
      int32_t angle_horiz = block_az +
        (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan]);

      uint16_t laser = chan % 16;
      int32_t angle_vert = this->chan_angles_.vertAdjust(laser);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(laser, angle_horiz);
      float distance = ntohs(channel.distance) * mech_const_param_.base.DISTANCE_RES;

      if (rs_decode_difop_->IsInByDist(distance) &&
          rs_decode_difop_->IsInByAzim(angle_horiz_final)) {
        float x =  distance * COS(angle_vert) * COS(angle_horiz_final) + this->mech_const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->mech_const_param_.RX * SIN(angle_horiz);
        float z =  distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        // this->transformPoint(x, y, z);
        uint8_t intensity = channel.intensity;
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(intensity / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(laser));
        }
      }
    }
  }
  if (azimuth < azimuthForRSHelios16P) {
    azimuthForRSHelios16P -= RS_ONE_ROUND;
  }
  if (azimuthForRSHelios16P != -36001 && azimuthForRSHelios16P < 0 && azimuth >= 0) {
    azimuthForRSHelios16P = azimuth;
    LidarScan *scan = new LidarScan("RS-Helios-16P", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
    return;
  }
  azimuthForRSHelios16P = azimuth;
}

void LidarDriver::packagePrase_RS_Helios(char buf[1248]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:RS-Helios receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }    
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);
  // Get rs_helios device info
  if (!is_received_difop_) {
    is_received_difop_ = rs_decode_difop_->ReceiveDifop(port + 1);
    if (is_received_difop_) {
      rs_decode_difop_->Decode();
      chan_angles_ = rs_decode_difop_->GetChanAngles();
      mech_const_param_ = rs_decode_difop_->GetMechConstParam();
    }
  }

  const RSHELIOSMsopPkt* pkt = reinterpret_cast<const RSHELIOSMsopPkt*>(buf);
  constexpr int RS_ONE_ROUND = 36000;

  int azimuth = RS_SWAP_SHORT(pkt->blocks[0].azimuth);
  for (uint16_t blk = 0; blk < mech_const_param_.base.BLOCKS_PER_PKT; blk++) {
    const RSHELIOSMsopBlock& block = pkt->blocks[blk];

    if (memcmp(mech_const_param_.base.BLOCK_ID, block.id, 2) != 0) {
      // this->cb_excep_(Error(ERRCODE_WRONGPKTHEADER));
      break;
    }

    int32_t block_az = ntohs(block.azimuth);
    int32_t block_az_diff;
    rs_decode_difop_->GetAzDiff(pkt, blk, block_az_diff);

    for (uint16_t chan = 0; chan < mech_const_param_.base.CHANNELS_PER_BLOCK; chan++) {
      const RSChannel& channel = block.channels[chan];
      int32_t angle_horiz = block_az +
        (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan]);

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, angle_horiz);
      float distance = ntohs(channel.distance) * mech_const_param_.base.DISTANCE_RES;

      if (rs_decode_difop_->IsInByDist(distance) &&
          rs_decode_difop_->IsInByAzim(angle_horiz_final)) {
        float x =  distance * COS(angle_vert) * COS(angle_horiz_final) + this->mech_const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->mech_const_param_.RX * SIN(angle_horiz);
        float z =  distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        // this->transformPoint(x, y, z);
        uint8_t intensity = channel.intensity;
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(intensity / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(chan));
        }
      }
    }
  }
  if (azimuth < azimuthForRSHelios) {
    azimuthForRSHelios -= RS_ONE_ROUND;
  }
  if (azimuthForRSHelios != -36001 && azimuthForRSHelios < 0 && azimuth >= 0) {
    azimuthForRSHelios = azimuth;
    LidarScan *scan = new LidarScan("RS-Helios", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
    return;
  }
  azimuthForRSHelios = azimuth;
}

void LidarDriver::packagePrase_RS_LiDAR_M1(char buf[1210]) {
  if (receveSize != packageLenth) {
    LOG_ERROR("{}:RS-LiDAR-M1 receive wrong size package len {} != {}", AffinityCpu, receveSize, packageLenth);
    return;
  }    
  float pointTime = float(getMonotonicTime() - scanMonotonicTime);

  int azimuth = 0;
  int height = rs_decode_difop_->rs_lidar_const_param_.LASER_NUM;
  const RSM1MsopPkt* mpkt_ptr = (RSM1MsopPkt*)buf;

  for (size_t blk_idx = 0; blk_idx < rs_decode_difop_->rs_lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    const RSM1Block& blk = mpkt_ptr->blocks[blk_idx];
    for (size_t channel_idx = 0; channel_idx < rs_decode_difop_->rs_lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
    {
      const RSM1Channel& channel = blk.channel[channel_idx];

      bool pointValid = false;
      float distance = RS_SWAP_SHORT(channel.distance) * rs_decode_difop_->rs_lidar_const_param_.DIS_RESOLUTION;
      if (distance <= 200.0 && distance >= 0.2)
      {
        int pitch = RS_SWAP_SHORT(channel.pitch) - rs_decode_difop_->ANGLE_OFFSET;
        int yaw = RS_SWAP_SHORT(channel.yaw) - rs_decode_difop_->ANGLE_OFFSET;
        float x = distance * rs_decode_difop_->checkCosTable(pitch) * rs_decode_difop_->checkCosTable(yaw);
        float y = distance * rs_decode_difop_->checkCosTable(pitch) * rs_decode_difop_->checkSinTable(yaw);
        float z = distance * rs_decode_difop_->checkSinTable(pitch);

        uint8_t intensity = channel.intensity;
        if (pointsInROI(x, y, z)) {
          pointCloud->emplace_back(x);
          pointCloud->emplace_back(y);
          pointCloud->emplace_back(z);
          pointCloud->emplace_back(intensity / 255.0f);
          pointCloudAttr->emplace_back(pointTime);
          pointCloudAttr->emplace_back(float(channel_idx));
        }
      }
    }
  }

  unsigned int pkt_cnt = RS_SWAP_SHORT(mpkt_ptr->header.pkt_cnt);

  // TODO whatif packet loss or seq unorder
  if (pkt_cnt == rs_decode_difop_->max_pkt_num_ || pkt_cnt < rs_decode_difop_->last_pkt_cnt_)
  {
    rs_decode_difop_->last_pkt_cnt_ = 1;
    LidarScan *scan = new LidarScan("RS-LiDAR-M1", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = getCurrentTime();
    scanMonotonicTime = getMonotonicTime();
    return;
  }
  rs_decode_difop_->last_pkt_cnt_ = pkt_cnt;
}

void LidarDriver::CopyCartesianHighRawPoint(char buf[], RawPacket& raw_packet,
                               unsigned int& start_pos, unsigned int& offset) {
  LivoxLidarCartesianHighRawPoint raw_point;
  for (size_t i = livox_offset_->data_offset; i < raw_packet.length; i += livox_offset_->cartesian_high_size) {
    memcpy(&raw_point, buf + i, livox_offset_->cartesian_high_size);
    float x = raw_point.x / 1000.0;
    float y = raw_point.y / 1000.0;
    float z = raw_point.z / 1000.0;
    float intensity = raw_point.reflectivity / 255.0f;
    float line = static_cast<float>(i / livox_offset_->cartesian_high_size % raw_packet.line_num);
    float offset_time = raw_packet.time_stamp - scanStartTime + i / livox_offset_->cartesian_high_size * raw_packet.point_interval;
    if (pointsInROI(x, y, z)) {
      pointCloud->emplace_back(x);
      pointCloud->emplace_back(y);
      pointCloud->emplace_back(z);
      pointCloud->emplace_back(intensity);
      pointCloudAttr->emplace_back(offset_time);
      pointCloudAttr->emplace_back(line);
    }
  }
}

void LidarDriver::CopyCartesianlowRawPoint(char buf[], RawPacket& raw_packet,
                              unsigned int& start_pos, unsigned int& offset) {
  LivoxLidarCartesianLowRawPoint raw_point;
  for (size_t i = livox_offset_->data_offset; i < raw_packet.length; i += livox_offset_->cartesian_low_size) {
    memcpy(&raw_point, buf + i, livox_offset_->cartesian_low_size);
    float x = raw_point.x / 100.0;
    float y = raw_point.y / 100.0;
    float z = raw_point.z / 100.0;
    float intensity = raw_point.reflectivity / 255.0f;
    float line = static_cast<float>(i / livox_offset_->cartesian_low_size % raw_packet.line_num);
    float offset_time = raw_packet.time_stamp - scanStartTime + i / livox_offset_->cartesian_low_size * raw_packet.point_interval;
    
    if (pointsInROI(x, y, z)) {
      pointCloud->emplace_back(x);
      pointCloud->emplace_back(y);
      pointCloud->emplace_back(z);
      pointCloud->emplace_back(intensity);
      pointCloudAttr->emplace_back(offset_time);
      pointCloudAttr->emplace_back(line);
    }
  }
}

void LidarDriver::packagePrase_Livox_Mid_360(char buf[]) {
  auto now_time = getCurrentTime();
  //First Set
  if (livox_first || now_time - last_frame_time_ > 1000000) {
    last_frame_time_ = now_time;
    livox_first = false;
    // return;
  }
  LivoxLidarEthernetPacket* packet = reinterpret_cast<LivoxLidarEthernetPacket*>(buf);
  RawPacket raw_packet = {};
  // packet.handle = handle;
  raw_packet.length = packet->length;
  raw_packet.lidar_type = LidarProtoType::kLivoxLidarType;
  raw_packet.extrinsic_enable = false; 
  raw_packet.line_num = kLineNumberMid360;

  raw_packet.data_type = packet->data_type;
  raw_packet.point_num = packet->dot_num;
  raw_packet.point_interval = packet->time_interval * 100 / packet->dot_num / 1000;  // us
  raw_packet.time_stamp = now_time; // us

  if (livox_frame_start) {
    scanStartTime = raw_packet.time_stamp;
    livox_frame_start  = false;
  }

  switch (packet->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      CopyCartesianHighRawPoint(buf, raw_packet, 
                                livox_offset_->data_offset,
                                livox_offset_->cartesian_high_size);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      CopyCartesianlowRawPoint(buf, raw_packet, 
                               livox_offset_->data_offset,
                               livox_offset_->cartesian_low_size);
      break;                                
    case kLivoxLidarSphericalCoordinateData:
      LOG_WARN("not support data type: {} !!", static_cast<int>(packet->data_type));
      return;
    default:
      LOG_WARN("unknown data type: {} !!", static_cast<int>(packet->data_type));
      return;
  }

  if (now_time - last_frame_time_ < livox_offset_->frame_interval_ / 1000) {
    return;
  }

  last_frame_time_ += livox_offset_->frame_interval_ / 1000;
  livox_frame_start = true;

  LidarScan *scan = new LidarScan("Livox-Mid-360", scanStartTime, 2, pointCloud, pointCloudAttr);
  scanQueue.enqueue(scan);
  resetPoints();
}

void LidarDriver::packagePrase_Custom(char buf[1206]) {
  CustomLidarPackage lidarPackage;
  memcpy(&lidarPackage, buf, sizeof(lidarPackage));

  if (prevFrameId != lidarPackage.frame_id) {
    prevFrameId = lidarPackage.frame_id;
    LidarScan *scan = new LidarScan("Custom", scanStartTime, 2, pointCloud, pointCloudAttr);
    scanQueue.enqueue(scan);
    resetPoints();
    scanStartTime = lidarPackage.timestamp;
    scanMonotonicTime = getMonotonicTime();
  }

  for (int i = 0; i < (4 * lidarPackage.point_num); i = i + 4) {
    float x, y, z;
    x = lidarPackage.points_buf[i + 0];
    y = lidarPackage.points_buf[i + 1];
    z = lidarPackage.points_buf[i + 2];
    float intensity = lidarPackage.points_buf[i + 3];
    if (pointsInROI(x, y, z)) {
      pointCloud->emplace_back(x);
      pointCloud->emplace_back(y);
      pointCloud->emplace_back(z);
      pointCloud->emplace_back(intensity);
      pointCloudAttr->emplace_back(0);
      pointCloudAttr->emplace_back(0);
    }
  }
}

void LidarDriver::xmlCorrection(void) {
  xmlData[0] = -15.0f;
  xmlData[1] = 1.0f;
  xmlData[2] = -13.0f;
  xmlData[3] = 3.0f;
  xmlData[4] = -11.0f;
  xmlData[5] = 5.0f;
  xmlData[6] = -9.0f;
  xmlData[7] = 7.0f;
  xmlData[8] = -7.0f;
  xmlData[9] = 9.0f;
  xmlData[10] = -5.0f;
  xmlData[11] = 11.0f;
  xmlData[12] = -3.0f;
  xmlData[13] = 13.0f;
  xmlData[14] = -1.0f;
  xmlData[15] = 15.0f;

  // os1 ouster 128
  double tmp1_128[128] = {
      21.57, 21.27, 20.96, 20.64, 20.34, 20.04, 19.71, 19.4, 19.12, 18.79,
      18.47, 18.15, 17.84, 17.53, 17.17, 16.88, 16.56, 16.22, 15.9, 15.57,
      15.26, 14.92, 14.59, 14.25, 13.92, 13.59, 13.26, 12.9, 12.59, 12.25,
      11.9, 11.57, 11.25, 10.89, 10.56, 10.22, 9.88, 9.53, 9.2, 8.84, 8.52,
      8.16, 7.8, 7.48, 7.12, 6.77, 6.42, 6.1, 5.74, 5.39, 5.05, 4.72, 4.36,
      4.01, 3.64, 3.3, 2.95, 2.59, 2.25, 1.91, 1.55, 1.21, 0.85, 0.52, 0.13,
      -0.21, -0.56, -0.92, -1.28, -1.61, -1.96, -2.31, -2.67, -3, -3.34, -3.69,
      -4.08, -4.41, -4.75, -5.1, -5.47, -5.8, -6.14, -6.48, -6.87, -7.21, -7.55,
      -7.88, -8.23, -8.57, -8.9, -9.23, -9.62, -9.95, -10.29, -10.62, -10.98,
      -11.3, -11.64, -11.98, -12.34, -12.66, -12.99, -13.32, -13.68, -13.99,
      -14.31, -14.65, -15.01, -15.32, -15.64, -15.97, -16.34, -16.64, -16.95,
      -17.26, -17.62, -17.94, -18.24, -18.53, -18.91, -19.22, -19.51, -19.8,
      -20.15, -20.46, -20.76, -21.04};

  double tmp2_128[128] = {
      4.22, 1.43, -1.38, -4.2, 4.22, 1.43, -1.4, -4.19, 4.23, 1.43, -1.38, -4.19,
      4.22, 1.43, -1.4, -4.19, 4.23, 1.42, -1.39, -4.19, 4.23, 1.43, -1.39, -4.2,
      4.22, 1.41, -1.39, -4.22, 4.24, 1.41, -1.39, -4.22, 4.23, 1.4, -1.4, -4.21,
      4.22, 1.4, -1.39, -4.22, 4.23, 1.41, -1.43, -4.22, 4.22, 1.4, -1.43, -4.21,
      4.22, 1.4, -1.42, -4.21, 4.23, 1.41, -1.42, -4.22, 4.22, 1.4, -1.42, -4.21,
      4.22, 1.4, -1.41, -4.2, 4.2, 1.4, -1.42, -4.24, 4.2, 1.4, -1.42, -4.23, 4.2,
      1.41, -1.41, -4.22, 4.19, 1.4, -1.41, -4.23, 4.2, 1.4, -1.42, -4.21, 4.19,
      1.38, -1.43, -4.23, 4.21, 1.4, -1.42, -4.22, 4.21, 1.38, -1.44, -4.23, 4.21,
      1.4, -1.42, -4.23, 4.22, 1.4, -1.43, -4.24, 4.21, 1.41, -1.41, -4.23, 4.22,
      1.41, -1.42, -4.24, 4.2, 1.4, -1.41, -4.23, 4.21, 1.39, -1.43, -4.23, 4.2,
      1.39, -1.44, -4.23, 4.21, 1.39, -1.44, -4.25};

  memcpy(beamAltitudeAngles, tmp1_128, sizeof(beamAltitudeAngles));
  memcpy(beamAzimuthAngles, tmp2_128, sizeof(beamAzimuthAngles));

  // os1 ouster 128 v2.5.2-v3
  double tmp1_128_v3[128] = {
      20.95, 20.68, 20.37, 20, 19.7, 19.43, 19.12, 18.77, 
      18.46, 18.17, 17.86, 17.49, 17.18, 16.9, 16.56, 16.19, 
      15.88, 15.58, 15.24, 14.88, 14.57, 14.26, 13.93, 13.56, 
      13.25, 12.93, 12.59, 12.23, 11.9, 11.58, 11.24, 10.87, 
      10.54, 10.21, 9.869999999999999, 9.51, 9.16, 8.84, 8.5, 8.140000000000001, 
      7.78, 7.46, 7.11, 6.74, 6.41, 6.06, 5.71, 5.35, 
      5.01, 4.66, 4.32, 3.95, 3.6, 3.27, 2.91, 2.55, 
      2.2, 1.85, 1.5, 1.15, 0.8, 0.44, 0.1, -0.26, 
      -0.61, -0.96, -1.32, -1.67, -2.02, -2.38, -2.72, -3.08, 
      -3.41, -3.78, -4.14, -4.48, -4.82, -5.18, -5.53, -5.88, 
      -6.21, -6.57, -6.92, -7.27, -7.61, -7.96, -8.32, -8.630000000000001, 
      -8.970000000000001, -9.34, -9.68, -10.01, -10.35, -10.7, -11.04, -11.37, 
      -11.69, -12.06, -12.4, -12.71, -13.05, -13.41, -13.74, -14.04, 
      -14.37, -14.73, -15.06, -15.36, -15.67, -16.03, -16.36, -16.66, 
      -16.96, -17.32, -17.64, -17.92, -18.24, -18.58, -18.91, -19.19, 
      -19.49, -19.83, -20.14, -20.41, -20.7, -21.06, -21.38, -21.63};

  double tmp2_128_v3[128] = {
      4.21, 1.38, -1.44, -4.27, 4.2, 1.38, -1.45, -4.27, 
      4.22, 1.38, -1.43, -4.26, 4.21, 1.4, -1.43, -4.25, 
      4.21, 1.38, -1.44, -4.26, 4.21, 1.38, -1.44, -4.25, 
      4.21, 1.38, -1.42, -4.26, 4.21, 1.39, -1.42, -4.25, 
      4.22, 1.41, -1.43, -4.25, 4.22, 1.41, -1.42, -4.24, 
      4.22, 1.4, -1.4, -4.24, 4.23, 1.4, -1.43, -4.24, 
      4.23, 1.4, -1.4, -4.23, 4.22, 1.42, -1.41, -4.22, 
      4.23, 1.42, -1.4, -4.22, 4.23, 1.42, -1.4, -4.22, 
      4.23, 1.42, -1.41, -4.22, 4.24, 1.42, -1.39, -4.23, 
      4.24, 1.43, -1.4, -4.23, 4.23, 1.43, -1.4, -4.22, 
      4.24, 1.42, -1.4, -4.22, 4.24, 1.42, -1.4, -4.21, 
      4.24, 1.43, -1.39, -4.21, 4.24, 1.43, -1.4, -4.21, 
      4.24, 1.42, -1.4, -4.21, 4.23, 1.43, -1.4, -4.21, 
      4.23, 1.42, -1.39, -4.21, 4.24, 1.42, -1.4, -4.21, 
      4.24, 1.44, -1.4, -4.2, 4.25, 1.43, -1.39, -4.22, 
      4.25, 1.43, -1.39, -4.21, 4.26, 1.44, -1.39, -4.22};

  memcpy(beamAltitudeAngles_v3, tmp1_128_v3, sizeof(beamAltitudeAngles_v3));
  memcpy(beamAzimuthAngles_v3, tmp2_128_v3, sizeof(beamAzimuthAngles_v3));

  // os2 ouster 128
  double tmp3_128[128] = {
      10.82, 10.64, 10.47, 10.29, 10.13, 9.960000000000001, 9.779999999999999,
      9.619999999999999, 9.460000000000001, 9.27, 9.130000000000001, 8.960000000000001,
      8.77, 8.609999999999999, 8.449999999999999, 8.279999999999999, 8.07, 7.93, 7.76,
      7.59, 7.41, 7.23, 7.06, 6.89, 6.72, 6.55, 6.38, 6.21, 6.03, 5.87, 5.68, 5.51,
      5.32, 5.16, 4.99, 4.81, 4.64, 4.46, 4.28, 4.12, 3.92, 3.78, 3.6, 3.43, 3.27,
      3.07, 2.91, 2.72, 2.56, 2.39, 2.2, 2.04, 1.86, 1.67, 1.51, 1.33, 1.15, 0.97,
      0.8100000000000001, 0.64, 0.43, 0.28, 0.12, -0.06, -0.23, -0.41, -0.58, -0.76,
      -0.95, -1.11, -1.29, -1.46, -1.62, -1.81, -1.98, -2.15, -2.34, -2.51, -2.69,
      -2.86, -3.05, -3.19, -3.38, -3.55, -3.74, -3.9, -4.07, -4.25, -4.42, -4.6,
      -4.78, -4.95, -5.11, -5.3, -5.46, -5.65, -5.84, -5.98, -6.16, -6.32, -6.49,
      -6.68, -6.86, -7.02, -7.2, -7.36, -7.55, -7.73, -7.87, -8.06, -8.23, -8.4,
      -8.58, -8.73, -8.9, -9.09, -9.23, -9.43, -9.58, -9.76, -9.960000000000001,
      -10.1, -10.28, -10.45, -10.59, -10.77, -10.93, -11.09};

  double tmp4_128[128] = {
      2.07, 0.6899999999999999, -0.68, -2.08, 2.07, 0.6899999999999999, -0.6899999999999999,
      -2.08, 2.09, 0.6899999999999999, -0.7, -2.08, 2.07, 0.71, -0.6899999999999999, -2.08,
      2.07, 0.71, -0.6899999999999999, -2.07, 2.07, 0.71, -0.6899999999999999, -2.08, 2.09,
      0.7, -0.7, -2.07, 2.09, 0.71, -0.6899999999999999, -2.08, 2.07, 0.7, -0.7, -2.08, 2.08,
      0.7, -0.6899999999999999, -2.07, 2.07, 0.7, -0.7, -2.08, 2.08, 0.68, -0.6899999999999999,
      -2.08, 2.09, 0.7, -0.7, -2.07, 2.09, 0.68, -0.68, -2.08, 2.08, 0.6899999999999999, -0.7,
      -2.07, 2.07, 0.6899999999999999, -0.7, -2.08, 2.09, 0.6899999999999999, -0.6899999999999999,
      -2.08, 2.07, 0.6899999999999999, -0.7, -2.08, 2.08, 0.6899999999999999, -0.6899999999999999,
      -2.07, 2.07, 0.6899999999999999, -0.7, -2.08, 2.07, 0.6899999999999999, -0.7, -2.07, 2.06,
      0.6899999999999999, -0.7, -2.07, 2.07, 0.6899999999999999, -0.7, -2.07, 2.09, 0.68,
      -0.6899999999999999, -2.08, 2.07, 0.7, -0.7, -2.07, 2.08, 0.6899999999999999, -0.7, -2.07,
      2.08, 0.68, -0.71, -2.09, 2.08, 0.68, -0.7, -2.09, 2.07, 0.7, -0.7, -2.09, 2.08, 0.68, -0.7,
      -2.08, 2.07, 0.68, -0.71, -2.08, 2.07, 0.68, -0.6899999999999999, -2.09};

  memcpy(beamAltitudeAnglesForOS2, tmp3_128, sizeof(beamAltitudeAnglesForOS2));
  memcpy(beamAzimuthAnglesForOS2, tmp4_128, sizeof(beamAzimuthAnglesForOS2));

  // os2 ouster 128((v2.5.2-v3)):Execute <get_beam_intrinsics> command obtains the lidar information
  double tmp3_128_v3[128] = {
      10.82, 10.64, 10.47, 10.29, 10.13, 9.960000000000001, 9.779999999999999,
      9.619999999999999, 9.460000000000001, 9.27, 9.130000000000001, 8.960000000000001,
      8.77, 8.609999999999999, 8.449999999999999, 8.279999999999999, 8.07, 7.93, 7.76,
      7.59, 7.41, 7.23, 7.06, 6.89, 6.72, 6.55, 6.38, 6.21, 6.03, 5.87, 5.68, 5.51,
      5.32, 5.16, 4.99, 4.81, 4.64, 4.46, 4.28, 4.12, 3.92, 3.78, 3.6, 3.43, 3.27,
      3.07, 2.91, 2.72, 2.56, 2.39, 2.2, 2.04, 1.86, 1.67, 1.51, 1.33, 1.15, 0.97,
      0.8100000000000001, 0.64, 0.43, 0.28, 0.12, -0.06, -0.23, -0.41, -0.58, -0.76,
      -0.95, -1.11, -1.29, -1.46, -1.62, -1.81, -1.98, -2.15, -2.34, -2.51, -2.69,
      -2.86, -3.05, -3.19, -3.38, -3.55, -3.74, -3.9, -4.07, -4.25, -4.42, -4.6,
      -4.78, -4.95, -5.11, -5.3, -5.46, -5.65, -5.84, -5.98, -6.16, -6.32, -6.49,
      -6.68, -6.86, -7.02, -7.2, -7.36, -7.55, -7.73, -7.87, -8.06, -8.23, -8.4,
      -8.58, -8.73, -8.9, -9.09, -9.23, -9.43, -9.58, -9.76, -9.960000000000001,
      -10.1, -10.28, -10.45, -10.59, -10.77, -10.93, -11.09};

  double tmp4_128_v3[128] = {
      2.07, 0.6899999999999999, -0.68, -2.08, 2.07, 0.6899999999999999, -0.6899999999999999,
      -2.08, 2.09, 0.6899999999999999, -0.7, -2.08, 2.07, 0.71, -0.6899999999999999, -2.08,
      2.07, 0.71, -0.6899999999999999, -2.07, 2.07, 0.71, -0.6899999999999999, -2.08, 2.09,
      0.7, -0.7, -2.07, 2.09, 0.71, -0.6899999999999999, -2.08, 2.07, 0.7, -0.7, -2.08, 2.08,
      0.7, -0.6899999999999999, -2.07, 2.07, 0.7, -0.7, -2.08, 2.08, 0.68, -0.6899999999999999,
      -2.08, 2.09, 0.7, -0.7, -2.07, 2.09, 0.68, -0.68, -2.08, 2.08, 0.6899999999999999, -0.7,
      -2.07, 2.07, 0.6899999999999999, -0.7, -2.08, 2.09, 0.6899999999999999, -0.6899999999999999,
      -2.08, 2.07, 0.6899999999999999, -0.7, -2.08, 2.08, 0.6899999999999999, -0.6899999999999999,
      -2.07, 2.07, 0.6899999999999999, -0.7, -2.08, 2.07, 0.6899999999999999, -0.7, -2.07, 2.06,
      0.6899999999999999, -0.7, -2.07, 2.07, 0.6899999999999999, -0.7, -2.07, 2.09, 0.68,
      -0.6899999999999999, -2.08, 2.07, 0.7, -0.7, -2.07, 2.08, 0.6899999999999999, -0.7, -2.07,
      2.08, 0.68, -0.71, -2.09, 2.08, 0.68, -0.7, -2.09, 2.07, 0.7, -0.7, -2.09, 2.08, 0.68, -0.7,
      -2.08, 2.07, 0.68, -0.71, -2.08, 2.07, 0.68, -0.6899999999999999, -2.09};

  memcpy(beamAltitudeAnglesForOS2_v3, tmp3_128_v3, sizeof(beamAltitudeAnglesForOS2_v3));
  memcpy(beamAzimuthAnglesForOS2_v3, tmp4_128_v3, sizeof(beamAzimuthAnglesForOS2_v3));  

  double tmp1_64[64] = {
    20.68, 20, 19.42, 18.73, 18.14, 17.46, 16.86, 16.14,
    15.54, 14.83, 14.2, 13.48, 12.87, 12.14, 11.51, 10.77,
    10.14, 9.41, 8.76, 8.02, 7.39, 6.64, 5.98, 5.23,
    4.58, 3.83, 3.18, 2.43, 1.77, 1.02, 0.36, -0.4,
    -1.07, -1.8, -2.45, -3.21, -3.88, -4.6, -5.28, -6.02,
    -6.66, -7.41, -8.06, -8.779999999999999, -9.44, -10.16, -10.8, -11.53,
    -12.16, -12.88, -13.52, -14.22, -14.87, -15.54, -16.19, -16.86,
    -17.49, -18.15, -18.77, -19.43, -20.04, -20.67, -21.27, -21.9
  };

  double tmp2_64[64] = {
    1.24, -4.4, 1.25, -4.37, 1.27, -4.37, 1.29, -4.36,
    1.3, -4.34, 1.29, -4.33, 1.32, -4.32, 1.33, -4.3,
    1.33, -4.3, 1.34, -4.29, 1.36, -4.27, 1.37, -4.26,
    1.37, -4.25, 1.38, -4.24, 1.39, -4.23, 1.4, -4.22,
    1.39, -4.22, 1.42, -4.19, 1.41, -4.18, 1.43, -4.18,
    1.45, -4.17, 1.46, -4.15, 1.47, -4.14, 1.5, -4.13,
    1.5, -4.12, 1.51, -4.11, 1.53, -4.09, 1.53, -4.09,
    1.53, -4.07, 1.54, -4.07, 1.56, -4.06, 1.58, -4.05
  };

  memcpy(beamAltitudeAngles64, tmp1_64, sizeof(beamAltitudeAngles64));
  memcpy(beamAzimuthAngles64, tmp2_64, sizeof(beamAzimuthAngles64));

  // os1 ouster 64((v2.5.2-v3)):Execute <get_beam_intrinsics> command obtains the lidar information
  double tmp1_64_v3[64] = {
    20.68, 20, 19.42, 18.73, 18.14, 17.46, 16.86, 16.14,
    15.54, 14.83, 14.2, 13.48, 12.87, 12.14, 11.51, 10.77,
    10.14, 9.41, 8.76, 8.02, 7.39, 6.64, 5.98, 5.23,
    4.58, 3.83, 3.18, 2.43, 1.77, 1.02, 0.36, -0.4,
    -1.07, -1.8, -2.45, -3.21, -3.88, -4.6, -5.28, -6.02,
    -6.66, -7.41, -8.06, -8.779999999999999, -9.44, -10.16, -10.8, -11.53,
    -12.16, -12.88, -13.52, -14.22, -14.87, -15.54, -16.19, -16.86,
    -17.49, -18.15, -18.77, -19.43, -20.04, -20.67, -21.27, -21.9
  };

  double tmp2_64_v3[64] = {
    1.24, -4.4, 1.25, -4.37, 1.27, -4.37, 1.29, -4.36,
    1.3, -4.34, 1.29, -4.33, 1.32, -4.32, 1.33, -4.3,
    1.33, -4.3, 1.34, -4.29, 1.36, -4.27, 1.37, -4.26,
    1.37, -4.25, 1.38, -4.24, 1.39, -4.23, 1.4, -4.22,
    1.39, -4.22, 1.42, -4.19, 1.41, -4.18, 1.43, -4.18,
    1.45, -4.17, 1.46, -4.15, 1.47, -4.14, 1.5, -4.13,
    1.5, -4.12, 1.51, -4.11, 1.53, -4.09, 1.53, -4.09,
    1.53, -4.07, 1.54, -4.07, 1.56, -4.06, 1.58, -4.05
  };

  memcpy(beamAltitudeAngles64_v3, tmp1_64_v3, sizeof(beamAltitudeAngles64_v3));
  memcpy(beamAzimuthAngles64_v3, tmp2_64_v3, sizeof(beamAzimuthAngles64_v3));


  double tmp1_32[32] = {
    21.62, 20.36, 19.11, 17.85, 16.56, 15.25, 13.92,
    12.57, 11.2, 9.84, 8.470000000000001, 7.07, 5.68,
    4.26, 2.87, 1.47, 0.06, -1.34, -2.77, -4.17, -5.57,
    -6.96, -8.359999999999999, -9.73, -11.09, -12.45,
    -13.78, -15.12, -16.41, -17.69, -18.97, -20.19  };
  double tmp2_32[32] = {
    4.28, 4.27, 4.26, 4.28, 4.27, 4.27, 4.27, 4.26,
    4.24, 4.25, 4.26, 4.23, 4.24, 4.22, 4.22, 4.24,
    4.24, 4.23, 4.2, 4.22, 4.21, 4.22, 4.2, 4.19,
    4.19, 4.19, 4.2, 4.17, 4.17, 4.17, 4.15, 4.17  };
  memcpy(beamAltitudeAngles32, tmp1_32, sizeof(beamAltitudeAngles32));
  memcpy(beamAzimuthAngles32, tmp2_32, sizeof(beamAzimuthAngles32));

  // os1 ouster 32((v2.5.2-v3)):Execute <get_beam_intrinsics> command obtains the lidar information
  double tmp1_32_v3[32] = {
    21.62, 20.36, 19.11, 17.85, 16.56, 15.25, 13.92,
    12.57, 11.2, 9.84, 8.470000000000001, 7.07, 5.68,
    4.26, 2.87, 1.47, 0.06, -1.34, -2.77, -4.17, -5.57,
    -6.96, -8.359999999999999, -9.73, -11.09, -12.45,
    -13.78, -15.12, -16.41, -17.69, -18.97, -20.19  };
  double tmp2_32_v3[32] = {
    4.28, 4.27, 4.26, 4.28, 4.27, 4.27, 4.27, 4.26,
    4.24, 4.25, 4.26, 4.23, 4.24, 4.22, 4.22, 4.24,
    4.24, 4.23, 4.2, 4.22, 4.21, 4.22, 4.2, 4.19,
    4.19, 4.19, 4.2, 4.17, 4.17, 4.17, 4.15, 4.17  };
  memcpy(beamAltitudeAngles32_v3, tmp1_32_v3, sizeof(beamAltitudeAngles32_v3));
  memcpy(beamAzimuthAngles32_v3, tmp2_32_v3, sizeof(beamAzimuthAngles32_v3));  

  // init LSC16 parameters
  for (size_t i = 0; i < 6300; ++i) {
    double angle = static_cast<double>(i) / 1000.0;
    cosTableForLSC16[i] = cos(angle);
    sinTableForLSC16[i] = sin(angle);
  }

  // init RS16 parameters
  for (int i = 0; i < 72000; i++) {
    double rad = static_cast<double>(i - 36000) * 0.01 * M_PI / 180;
    cosTableForRS16[i] = std::cos(rad);
    sinTableForRS16[i] = std::sin(rad);
  }
  int tmp9[16] = {-1499, -1299, -1100, -900, -700, -500, -299, -100,
                  1499, 1299, 1099, 899, 700, 500, 299, 99};
  memcpy(vertAangleList16, tmp9, sizeof(vertAangleList16));
  int tmp10[16] = {0};
  memcpy(horiAangleList16, tmp10, sizeof(horiAangleList16));

  // init RS32 parameters
  int tmp11[32] = {-1031,-642,229,329,463,700,1033,1506,
                   29,0,-36,-70,163,126,96,63,
                   -2500,-1460,-791,-540,-370,-400,-436,-470,
                   -236,-266,-300,-333,-100,-133,-170,-200};
  memcpy(vertAangleList32, tmp11, sizeof(vertAangleList32));
  int tmp12[32] = {813,818,840,-672,826,-720,826,-726,
                  -766,-241,279,818,-778,-250,284,813,
                  -782,-772,-748,-741,-760,-241,296,826,
                  -764,-240,292,823,-757,-231,301,828};
  memcpy(horiAangleList32, tmp12, sizeof(horiAangleList32));

  // init RS-Ruby-Lite parameters
  int tmp13[80] = {-1356, -109, -439, -29, -359, -579, 51, -279, 351, -498,
                                -199, 506, -419, -1958, -129, -339, -715, -49, -259, -599,
                                31, -179, -519, -99, -2500, -19, -765, 61, -269, 141, -189,
                                -1604, -119, -685, -39, 41, -289, 656, 121, -208, -835, -69,
                                -399, -619, 11, -319, -539, 91, -239, -459, -159, -379, 251,
                                -1034, -89, -299, -9, -219, -559, 71, -139, 1150, -479, -58,
                                -1174, 21, -650, 101, -229, 181, -149, 900, -924, -79, 1,
                                81, -249, 1500, 161, -169};
  memcpy(vertAangleList80, tmp13, sizeof(vertAangleList80));
  int tmp14[80] = {595, 425, 255, 425, 255, 595, 425, 255, 85, 595, 255,
                                85, 595, 255, 85, 595, 255, 85, 595, 255, 85, 595, 255,
                                595, 85, 595, 85, 595, 425, 595, 425, 425, 255, 425, 255,
                                255, 85, 595, 255, 85, -85, -255, -425, -85, -255, -425,
                                -85, -255, -425, -85, -425, -85, -255, -425, -595, -85,
                                -595, -85, -425, -595, -85, -255, -425, -85, -595, -85,
                                -595, -85, -255, -85, -255, -425, -255, -425, -425, -425,
                                 -595, -85, -425, -595};
  memcpy(horiAangleList80, tmp14, sizeof(horiAangleList80));
}

}  // namespace LIDAR
