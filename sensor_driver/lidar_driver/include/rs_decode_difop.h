#ifndef _RS_DECODE_DIFOP_H_
#define _RS_DECODE_DIFOP_H_

#include <string>
#include <string.h>
#include <memory>
#include <vector>
#include <arpa/inet.h>

#include "readerwriterqueue.h"
#include "LidarScan.h"
#include "DecoderBase.h"
#include "UDPServer.h"
#include "SystemUtils.h"
#include "Logger.h"

class RsDecodeDifop {
 public:
  constexpr static int32_t RS_ONE_ROUND = 36000;
  const size_t MEMS_MSOP_LEN = 1210;
  const size_t MEMS_DIFOP_LEN = 256;

  const uint32_t SINGLE_PKT_NUM = 630;
  const uint32_t DUAL_PKT_NUM = 1260;
  const int ANGLE_OFFSET = 32768;

  enum LidarType  ///< LiDAR type
  {
    RS16 = 1,
    RS32,
    RSBP,
    RS128,
    RS80,
    RSP128,
    RSP80,
    RSP48,
    RSHELIOS,
    RSHELIOS_16P,
    RSROCK,
    RSM1 = 0x20,
    RSM2,
    RSEOS
  };

  inline std::string lidarTypeToStr(const LidarType& type)
  {
    std::string str = "";
    switch (type)
    {
      case LidarType::RS16:
      str = "RS16";
      break;
      case LidarType::RS32:
      str = "RS32";
      break;
      case LidarType::RSBP:
      str = "RSBP";
      break;
      case LidarType::RSHELIOS:
      str = "RS-Helios";
      break;
      case LidarType::RSHELIOS_16P:
      str = "RS-Helios-16P";
      break;
      case LidarType::RS128:
      str = "RS128";
      break;
      case LidarType::RS80:
      str = "RS80";
      break;
      case LidarType::RSP128:
      str = "RSP128";
      break;
      case LidarType::RSP80:
      str = "RSP80";
      break;
      case LidarType::RSP48:
      str = "RSP48";
      break;
      case LidarType::RSM1:
      str = "RS-LiDAR-M1";
      break;
      case LidarType::RSM2:
      str = "RSM2";
      break;
      case LidarType::RSEOS:
      str = "RSEOS";
      break;
      default:
      str = "ERROR";
    }
  return str;
  }

  inline LidarType strToLidarType(const std::string& type)
  {
    if (type == "RS16")
    {
        return LidarType::RS16;
    }
    else if (type == "RS32")
    {
        return LidarType::RS32;
    }
    else if (type == "RSBP")
    {
        return LidarType::RSBP;
    }
    else if (type == "RS-Helios")
    {
        return LidarType::RSHELIOS;
    }
    else if (type == "RS-Helios-16P")
    {
        return LidarType::RSHELIOS_16P;
    }
    else if (type == "RS128")
    {
        return LidarType::RS128;
    }
    else if (type == "RS80")
    {
        return LidarType::RS80;
    }
    else if (type == "RSP128")
    {
        return LidarType::RSP128;
    }
    else if (type == "RSP80")
    {
        return LidarType::RSP80;
    }
    else if (type == "RSP48")
    {
        return LidarType::RSP48;
    }
    else if (type == "RS-LiDAR-M1")
    {
        return LidarType::RSM1;
    }
    else if (type == "RSM2")
    {
        return LidarType::RSM2;
    }
    else if (type == "RSEOS")
    {
        return LidarType::RSEOS;
    }
    else
    {
        exit(-1);
    }
  }  
 public:
  RsDecodeDifop(const std::string& name, const int& packageLenth);
  ~RsDecodeDifop() {}
  bool ReceiveDifop(const int& port);
  void Decode();
  void DecodeDifop(char buf[]);

  inline std::vector<float> initTrigonometricLookupTable(const std::function<double(const double)>& func)
  {
    std::vector<float> temp_table = std::vector<float>(2 * RS_ONE_ROUND, 0.0);

    for (int i = 0; i < 2 * RS_ONE_ROUND; i++)
    {
      const double rad = RS_TO_RADS(static_cast<double>(i - RS_ONE_ROUND) * RS_ANGLE_RESOLUTION);
      temp_table[i] = (float)func(rad);
    }
    return temp_table;
  }

  inline float checkCosTable(const int& angle)
  {
    return cos_lookup_table_[angle + RS_ONE_ROUND];
  }
  inline float checkSinTable(const int& angle)
  {
    return sin_lookup_table_[angle + RS_ONE_ROUND];
  }

  ChanAngles GetChanAngles() {return chan_angles_;}
  RSDecoderMechConstParam GetMechConstParam() {return mech_const_param_;}
  inline void GetAzDiff(const RSHELIOSMsopPkt* pkt, const uint16_t& blk,
                           int32_t& block_az_diff) {
    if (blk < mech_const_param_.base.BLOCKS_PER_PKT - 1) {
      block_az_diff = ntohs(pkt->blocks[blk+1].azimuth) - ntohs(pkt->blocks[blk].azimuth);
      if (block_az_diff < 0) { block_az_diff += 36000; }

      // Skip FOV blind zone 
      if (block_az_diff > 100)
      {
        block_az_diff = this->block_az_diff_;
      }    
    } else {
      block_az_diff = this->block_az_diff_;
    }
  }  

  inline bool IsInByDist(float distance) {
    return distance_section_.in(distance);
  }

  inline bool IsInByAzim(int32_t angle) {
    return scan_section_.in(angle);
  }


 private:
  inline RSDecoderMechConstParam& getConstParamHelios16P() {
    static RSDecoderMechConstParam param = 
    {
        1248 // msop len
        , 1248 // difop len
        , 4 // msop id len
        , 8 // difop id len
        , {0x55, 0xAA, 0x05, 0x5A} // msop id
        , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
        , {0xFF, 0xEE} // block id
        , 16 // laser number
        , 12 // blocks per packet
        , 32 // channels per block
        , 0.4f // distance min
        , 200.0f // distance max
        , 0.0025f // distance resolution
        , 0.0625f // temperature resolution

        // lens center
        , 0.03498f // RX
        , -0.015f // RY
        , 0.0f // RZ
    };

    float blk_ts = 55.56f;
    param.BLOCK_DURATION = blk_ts / 1000000;

    return param;
  } 

  inline RSDecoderMechConstParam& getConstParamHelios() {
    static RSDecoderMechConstParam param = 
    {
      1248 // msop len
        , 1248 // difop len
        , 4 // msop id len
        , 8 // difop id len
        , {0x55, 0xAA, 0x05, 0x5A} // msop id
      , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
      , {0xFF, 0xEE} // block id
      , 32 // laser number
      , 12 // blocks per packet
        , 32 // channels per block
        , 0.4f // distance min
        , 200.0f // distance max
        , 0.0025f // distance resolution
        , 0.0625f // temperature resolution

        // lens center
        , 0.03498f // RX
        , -0.015f // RY
        , 0.0f // RZ
    };

    // INIT_ONLY_ONCE();

    float blk_ts = 55.56f;
    float firing_tss[] = 
    {
      0.00f,  1.57f,  3.15f,  4.72f,  6.30f,  7.87f,  9.45f, 11.36f, 
      13.26f, 15.17f, 17.08f, 18.99f, 20.56f, 22.14f, 23.71f, 25.29f,
      26.53f, 29.01f, 27.77f, 30.25f, 31.49f, 33.98f, 32.73f, 35.22f, 
      36.46f, 37.70f, 38.94f, 40.18f, 41.42f, 42.67f, 43.91f, 45.15f
    };

    param.BLOCK_DURATION = blk_ts / 1000000;
    for (uint16_t i = 0; i < sizeof(firing_tss)/sizeof(firing_tss[0]); i++)
    {
      param.CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
      param.CHAN_AZIS[i] = firing_tss[i] / blk_ts;
    }

    return param;
  }  
 
  inline const LidarConstantParameter getRSM1ConstantParam()
  {
    LidarConstantParameter ret_param;
    ret_param.MSOP_ID = 0xA55AAA55;
    ret_param.DIFOP_ID = 0x555511115A00FFA5;
    ret_param.BLOCKS_PER_PKT = 25;
    ret_param.CHANNELS_PER_BLOCK = 5;
    ret_param.LASER_NUM = 5;
    ret_param.DIS_RESOLUTION = 0.005;
    return ret_param;
  }
 
  template <typename T_Difop>
    inline void decodeDifopCommon(const T_Difop& pkt) {
    // rounds per second
    this->rps_ = ntohs(pkt.rpm) / 60;
    if (this->rps_ == 0) {
      // RS_WARNING << "LiDAR RPM is 0. Use default value 600." << RS_REND;
      this->rps_ = 10;
    }

    // blocks per frame
    this->blks_per_frame_ = (uint16_t)(1 / 
          (this->rps_ * this->mech_const_param_.BLOCK_DURATION));

    // block diff of azimuth
    this->block_az_diff_ = 
      (uint16_t)std::round(RS_ONE_ROUND * this->rps_ * this->mech_const_param_.BLOCK_DURATION);

    // fov related
    uint16_t fov_start_angle = ntohs(pkt.fov.start_angle);
    uint16_t fov_end_angle = ntohs(pkt.fov.end_angle);
    uint16_t fov_range = (fov_start_angle < fov_end_angle) ? 
      (fov_end_angle - fov_start_angle) : (fov_end_angle + RS_ONE_ROUND - fov_start_angle);
    uint16_t fov_blind_range = RS_ONE_ROUND - fov_range;

    // fov blind diff of timestamp
    this->fov_blind_ts_diff_ = 
      (double)fov_blind_range / ((double)RS_ONE_ROUND * (double)this->rps_);

    // load angles
    if (!this->param_.config_from_file && !this->angles_ready_)
    {
      int ret = this->chan_angles_.loadFromDifop(pkt.vert_angle_cali, pkt.horiz_angle_cali);
      this->angles_ready_ = (ret == 0);
    }
  }

  inline RSEchoMode getEchoMode(uint8_t mode) {
    switch (mode)
    {
      case 0x00: // dual return
        return RSEchoMode::ECHO_DUAL;
      case 0x04: // strongest return
      case 0x05: // last return
      case 0x06: // nearest return
      default:
        return RSEchoMode::ECHO_SINGLE;
    }
  }

  inline void calcParamHelios16P() {
    float blk_ts = 55.56f;
    float firing_tss[] = 
    {
      0.00f,  3.15f,  6.30f,  9.45f, 13.26f, 17.08f, 20.56f, 23.71f,
      26.53f, 27.77f, 31.49f, 32.73f, 36.46f, 38.94f, 41.42f, 43.91f,
      55.56f, 58.70f, 61.85f, 65.00f, 68.82f, 72.64f, 76.12f, 79.27f, 
      82.08f, 83.32f, 87.05f, 88.29f, 92.01f, 94.50f, 96.98f, 99.46f
    };

    if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE) {
      SinglecalcChannel(blk_ts, firing_tss, 
      this->mech_const_param_.CHAN_AZIS, this->mech_const_param_.CHAN_TSS);
    }
    else {
      DualcalcChannel(blk_ts, firing_tss, 
      this->mech_const_param_.CHAN_AZIS, this->mech_const_param_.CHAN_TSS);
    }
  }

  static void SinglecalcChannel(const float blk_ts, const float firing_tss[], 
                                float az_percents[], double ts_diffs[]) {
    for (uint16_t chan = 0; chan < 32; chan++) {
      az_percents[chan] = firing_tss[chan] / (blk_ts * 2);
      ts_diffs[chan] = (double)firing_tss[chan] / 1000000;
    }
  }

  static void DualcalcChannel(const float blk_ts, const float firing_tss[], 
                              float az_percents[], double ts_diffs[]) {
    for (uint16_t chan = 0; chan < 32; chan++) {
      az_percents[chan] = firing_tss[chan%16] / blk_ts;
      ts_diffs[chan] = (double)firing_tss[chan%16] / 1000000;
    }
  }  

 private:
  LidarType lidar_type_;
  double packet_duration_;
  const int packageLenth_;
  std::unique_ptr<char> buf_difop_;
  RSDecoderMechConstParam mech_const_param_; // const param 
  RSDecoderParam param_; // user param
  ChanAngles chan_angles_; // vert_angles/horiz_angles adjustment
  uint16_t rps_; // rounds per second
  uint16_t blks_per_frame_; // blocks per frame/round
  uint16_t split_blks_per_frame_; // blocks in msop pkt per frame/round. 
  uint16_t block_az_diff_ = 20; // azimuth difference between adjacent blocks.
  double fov_blind_ts_diff_; // timestamp difference across blind section(defined by fov)

  AzimuthSection scan_section_; // valid azimuth section
  DistanceSection distance_section_; // invalid section of distance

  RSEchoMode echo_mode_ = RSEchoMode::ECHO_SINGLE; // echo mode (defined by return mode)
  float temperature_; // lidar temperature

  bool angles_ready_ = false; // is vert_angles/horiz_angles ready from csv file/difop packet?

  std::vector<float> cos_lookup_table_;
  std::vector<float> sin_lookup_table_;
public:
  LidarConstantParameter rs_lidar_const_param_;
  uint32_t msop_pkt_len_;
  uint32_t difop_pkt_len_;
  uint32_t max_pkt_num_;
  uint32_t last_pkt_cnt_;
  double last_pkt_time_;
};

#endif // _RS_DECODE_DIFOP_H_