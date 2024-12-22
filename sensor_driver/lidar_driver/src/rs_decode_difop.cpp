#include "rs_decode_difop.h"

RsDecodeDifop::RsDecodeDifop(const std::string& name, const int& packageLenth) : 
  lidar_type_(strToLidarType(name)), 
  packageLenth_(packageLenth),
  distance_section_(0.4f, 200.0f, 0.2f, 200.0f),
  scan_section_(0.0f, 360.0f) {
  buf_difop_.reset(new char[packageLenth_]);
  switch (lidar_type_) {
  case LidarType::RSHELIOS_16P:
    mech_const_param_ = getConstParamHelios16P();
    packet_duration_ = mech_const_param_.BLOCK_DURATION * 
                       mech_const_param_.base.BLOCKS_PER_PKT * 2;
    calcParamHelios16P();
    break;
  case LidarType::RSHELIOS:
    mech_const_param_ = getConstParamHelios();
    break;    
  case LidarType::RSM1:
    rs_lidar_const_param_ = getRSM1ConstantParam();
    cos_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::cos(rad); });
    sin_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::sin(rad); });
    max_pkt_num_ = SINGLE_PKT_NUM;
    last_pkt_cnt_ = 1;
    last_pkt_time_ = 0;
    msop_pkt_len_ = MEMS_MSOP_LEN;
    difop_pkt_len_ = MEMS_DIFOP_LEN;
  default:
    break;
  }
  blks_per_frame_ = (uint16_t)(1 / (10 * mech_const_param_.BLOCK_DURATION));
  split_blks_per_frame_ = blks_per_frame_;
  chan_angles_ = ChanAngles(mech_const_param_.base.LASER_NUM);
  distance_section_ = DistanceSection(mech_const_param_.base.DISTANCE_MIN, 
                      mech_const_param_.base.DISTANCE_MAX, 
                      param_.min_distance, param_.max_distance);
  scan_section_ = AzimuthSection((int32_t)(param_.start_angle * 100), 
                                 (int32_t)(param_.end_angle * 100));
}


bool RsDecodeDifop::ReceiveDifop(const int& port) {
  std::unique_ptr<UDPServer> veloUDPServer_difop(new UDPServer(port));
  int receve_size_difop = 0;
  bool is_verification = false;
  uint8_t id = 0;
  while (receve_size_difop != packageLenth_) {
    receve_size_difop = veloUDPServer_difop->UDPServerReceive(buf_difop_.get(),
                                                              packageLenth_);                                                        
    if (receve_size_difop != packageLenth_) {
      LOG_ERROR("Rs_helios_difop receive wrong size package len {} != {}", 
                receve_size_difop, packageLenth_);
      continue;
    } else {
      memcpy(&id, buf_difop_.get(), sizeof(uint8_t));
      id == 0xA5 ? is_verification = true : is_verification = false;
      if (!is_verification) {
        LOG_ERROR("Rs_helios_lidar port {}, id of difop verification failed!", 
                  port - 1);  
        break;      
      }
    }
  } 
  return is_verification;
}

void RsDecodeDifop::Decode() {
  const RSHELIOSDifopPkt& pkt = *reinterpret_cast<const RSHELIOSDifopPkt*>(buf_difop_.get());
  decodeDifopCommon(pkt);
  RSEchoMode echo_mode = getEchoMode(pkt.return_mode);
  if(lidar_type_ == LidarType::RSHELIOS_16P) {
    if (this->echo_mode_ != echo_mode)
    {
      this->echo_mode_ = echo_mode;
      this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
        this->blks_per_frame_ : (this->blks_per_frame_ >> 1);
      calcParamHelios16P();
    }
  } else if (lidar_type_ == LidarType::RSHELIOS) {
    this->echo_mode_ = echo_mode;
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
      (this->blks_per_frame_ << 1) : this->blks_per_frame_;    
  }
}

void RsDecodeDifop::DecodeDifop(char buf[]) {
  const RSHELIOSDifopPkt& pkt = *reinterpret_cast<const RSHELIOSDifopPkt*>(buf);
  decodeDifopCommon(pkt);
  RSEchoMode echo_mode = getEchoMode(pkt.return_mode);
  if(lidar_type_ == LidarType::RSHELIOS_16P) {
    if (this->echo_mode_ != echo_mode)
    {
      this->echo_mode_ = echo_mode;
      this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
        this->blks_per_frame_ : (this->blks_per_frame_ >> 1);
      calcParamHelios16P();
    }
  } else if (lidar_type_ == LidarType::RSHELIOS) {
    this->echo_mode_ = echo_mode;
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
      (this->blks_per_frame_ << 1) : this->blks_per_frame_;    
  }  
}  
