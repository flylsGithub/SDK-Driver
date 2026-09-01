/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Wanji Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_base.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/cmd_repository_719e.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_field_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_firmware_version_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_monitoring_case_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_protocol_version_set.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_verify_results_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {

#pragma pack(push, 1)

typedef struct _Vanjee719EDataUnit {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t Confidence;
} Vanjee719EDataUnit;

typedef struct _Vanjee719EDataBlock {
  uint8_t start_flag;
  int16_t azimuth;
  uint8_t remain3[2];
  Vanjee719EDataUnit channel[4];
} Vanjee719EDataBlock;

typedef struct _VanjeeFieldData719E {
  uint32_t field_data_id;
  uint8_t line_id;
  float range_max[1381];
  float range_min[1381];
  float start_angle;
  float angle_increment;
  float angular_resolution;
  uint32_t field_definition;
} VanjeeFieldData719E;

typedef struct _VanjeeMonitoringCase719E {
  int32_t monitoring_case_type;
  uint32_t monitoring_case_list_id;
  uint32_t monitoring_case_id;
  std::vector<int32_t> input_io;
  std::vector<int32_t> input_in;
  std::vector<int32_t> range;
  std::vector<std::vector<int32_t>> output_io;
  std::vector<uint32_t> case_info;
} VanjeeMonitoringCase719E;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee719E : public Decoder<T_PointCloud> {
 private:
  std::vector<double> all_points_luminous_moment_719e;         // Cache 4 channels with a circular point cloud time difference
  const double luminous_period_of_ld_ = 0.000027777;           // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 0.000006944;  // Time interval between adjacent vertical angles within the group

  int32_t azimuth_cur_ = -1.0;
  int32_t pre_frame_id_ = -1;
  uint8_t pre_bank_id_ = 0;
  uint32_t point_id_offset_ = 0;
  uint32_t pre_point_id_offset_ = 0xffffffff;

  uint8_t get_zone_alarm_data_flag_ = 0;
  std::string zone_alarm_state_ = "";
  std::string io_info_str_ = "";
  std::string intrusion_info_str_1_ = "";
  std::string intrusion_info_str_2_ = "";

  std::vector<int32_t> lidar_ver_angle_ = std::vector<int32_t>{355500, 0, 4500, 9000};
  std::map<uint16_t, std::string> get_lidar_param_;
  std::string lidar_mac_ = "";
  std::string firmware_version_ = "";

  uint32_t recv_field_data_total_pkt_num_ = 0;
  std::vector<VanjeeFieldData719E> field_data_;
  bool get_field_data_flag_ = false;
  std::vector<uint32_t> per_field_data_criterion_ = std::vector<uint32_t>(128, 0);
  std::vector<uint32_t> per_field_data_flag_ = std::vector<uint32_t>(128, 0);
  std::vector<std::vector<float>> range_min_ = std::vector<std::vector<float>>(128, std::vector<float>(1381, 500.0f));
  std::vector<std::vector<float>> range_max_ = std::vector<std::vector<float>>(128, std::vector<float>(1381, 500.0f));

  uint32_t recv_monitoring_case_total_pkt_num_ = 0;
  bool get_monitoring_case_flag_ = false;
  std::vector<VanjeeMonitoringCase719E> monitoring_case_;

  bool exit_detect_flag_ = false;
  std::thread detect_difop_ts_thread_;
  double start_recv_difop_field_ts_ = 0.0;
  double start_recv_difop_monitoring_ts_ = 0.0;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderConstParam &getConstParam();

  uint16_t pre_operate_frequency_ = 0;
  void initLdLuminousMoment(uint16_t operate_frequency, uint32_t point_offset);

  DecoderPacketBase<T_PointCloud, Vanjee719EDataBlock, Vanjee719EDataUnit> *decoder_packet_base_ = nullptr;
  void updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info, Vanjee719EDataBlock *data_block);
  void decoderDataUnitCallback(Vanjee719EDataUnit *data_unit, PointInfo &point_info);
  void zoneAlarmUpload(int32_t hor_angle, uint8_t *buf, size_t size);
  std::string getZoneAlarmIOInfo(uint8_t *buf);
  std::string getZoneAlarmCaseInfo(std::string key_info, std::string key_speed, uint8_t *buf);
  void detectDifopTsProcess();

 public:
  constexpr static double FRAME_DURATION = 0.05;
  constexpr static uint32_t SINGLE_PKT_NUM = 28;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee719E() {  // = default;
    delete decoder_packet_base_;
    exit_detect_flag_ = true;
    if (detect_difop_ts_thread_.joinable()) {
      detect_difop_ts_thread_.join();
    }
  }
  explicit DecoderVanjee719E(const WJDecoderParam &param);
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  bool decodeMsopPktBaseProtocol(const uint8_t *pkt, size_t size);
};

template <typename T_PointCloud>
inline WJDecoderConstParam &DecoderVanjee719E<T_PointCloud>::getConstParam() {
  static WJDecoderConstParam param = {4, 0.001f};
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee719E<T_PointCloud>::DecoderVanjee719E(const WJDecoderParam &param) : Decoder<T_PointCloud>(getConstParam(), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  this->point_cloud_ready_ = true;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  decoder_packet_base_ = new DecoderPacketBase<T_PointCloud, Vanjee719EDataBlock, Vanjee719EDataUnit>(this);
  detect_difop_ts_thread_ = std::thread(std::bind(&DecoderVanjee719E<T_PointCloud>::detectDifopTsProcess, this));
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::initLdLuminousMoment(uint16_t operate_frequency, uint32_t point_offset) {
  if (operate_frequency != pre_operate_frequency_ || point_offset != pre_point_id_offset_) {
    uint16_t col_num = 1800 - (point_offset / 2);
    uint32_t point_num = col_num * 4;
    double offset = 0;
    all_points_luminous_moment_719e.resize(point_num);  // 7200

    for (uint16_t col = 0; col < col_num; col++) {  // 1800
      for (uint8_t row = 0; row < 4; row++) {
        offset = row * luminous_period_of_adjacent_ld_;
        all_points_luminous_moment_719e[col * 4 + row] = col * luminous_period_of_ld_ + offset;
      }
    }
    decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_719e;
    pre_operate_frequency_ = operate_frequency;
    pre_point_id_offset_ = point_offset;
  }

  if (this->param_.laser_scan_enable) {
    double angle_offset = point_offset / 4 * 0.2;
    this->scan_data_->angle_min = 180 - angle_offset;
    this->scan_data_->angle_max = -180 + angle_offset;
    this->scan_data_->angle_increment = -360.0 / 1800;
    this->scan_data_->scan_time = 1.0 / operate_frequency;
    if (this->param_.ts_first_point)
      this->scan_data_->time_increment = this->scan_data_->scan_time / 1800;
    else
      this->scan_data_->time_increment = -this->scan_data_->scan_time / 1800;
    this->scan_data_->range_min = this->param_.min_distance;
    this->scan_data_->range_max = this->param_.max_distance;
  }
}

template <typename T_PointCloud>
inline bool DecoderVanjee719E<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  uint16_t l_pktheader = pkt[0] << 8 | pkt[1];
  switch (l_pktheader) {
    case 0xFFCC: {
      if (size == 1400) {
        ret = decodeMsopPktBaseProtocol(pkt, size);
      }
    } break;

    default:
      break;
  }

  return ret;
}

template <typename T_PointCloud>
std::string DecoderVanjee719E<T_PointCloud>::getZoneAlarmIOInfo(uint8_t *buf) {
  std::string ret_str = "";
  std::string ossd_a_1_str = "";
  std::string ossd_b_1_str = "";
  std::string input_a_str = "";
  std::string input_b_str = "";
  std::string in_a_str = "";
  std::string in_b_str = "";
  std::string output_str = "";
  std::vector<int16_t> input_a_vec;
  std::vector<int16_t> input_b_vec;
  std::vector<int16_t> in_a_vec;
  std::vector<int16_t> in_b_vec;
  std::vector<int16_t> output_vec;

  uint16_t seq = buf[8] & 0x03;
  int16_t ossd_a_1 = (buf[8] & 0x04) == 0x04 ? ((buf[8] & 0x08) >> 3) : -1;
  int16_t ossd_b_1 = (buf[8] & 0x10) == 0x10 ? ((buf[8] & 0x20) >> 5) : -1;

  input_a_vec.push_back((buf[9] & 0x03) == 0x01 ? ((buf[9] & 0x04) >> 2) : -1);
  input_b_vec.push_back((buf[9] & 0x03) == 0x01 ? ((buf[9] & 0x08) >> 3) : -1);
  input_a_vec.push_back((buf[9] & 0x30) == 0x10 ? ((buf[9] & 0x40) >> 6) : -1);
  input_b_vec.push_back((buf[9] & 0x30) == 0x10 ? ((buf[9] & 0x80) >> 7) : -1);
  input_a_vec.push_back((buf[10] & 0x03) == 0x01 ? ((buf[10] & 0x04) >> 2) : -1);
  input_b_vec.push_back((buf[10] & 0x03) == 0x01 ? ((buf[10] & 0x08) >> 3) : -1);
  input_a_vec.push_back((buf[10] & 0x30) == 0x10 ? ((buf[10] & 0x40) >> 6) : -1);
  input_b_vec.push_back((buf[10] & 0x30) == 0x10 ? ((buf[10] & 0x80) >> 7) : -1);
  input_a_vec.push_back((buf[11] & 0x03) == 0x01 ? ((buf[11] & 0x04) >> 2) : -1);
  input_b_vec.push_back((buf[11] & 0x03) == 0x01 ? ((buf[11] & 0x08) >> 3) : -1);
  input_a_vec.push_back((buf[11] & 0x30) == 0x10 ? ((buf[11] & 0x40) >> 6) : -1);
  input_b_vec.push_back((buf[11] & 0x30) == 0x10 ? ((buf[11] & 0x80) >> 7) : -1);
  input_a_vec.push_back((buf[12] & 0x03) == 0x01 ? ((buf[12] & 0x04) >> 2) : -1);
  input_b_vec.push_back((buf[12] & 0x03) == 0x01 ? ((buf[12] & 0x08) >> 3) : -1);
  input_a_vec.push_back((buf[12] & 0x30) == 0x10 ? ((buf[12] & 0x40) >> 6) : -1);
  input_b_vec.push_back((buf[12] & 0x30) == 0x10 ? ((buf[12] & 0x80) >> 7) : -1);

  in_a_vec.push_back((buf[13] & 0x03) == 0x01 ? ((buf[13] & 0x04) >> 2) : -1);
  in_b_vec.push_back((buf[13] & 0x03) == 0x01 ? ((buf[13] & 0x08) >> 3) : -1);
  in_a_vec.push_back((buf[13] & 0x30) == 0x10 ? ((buf[13] & 0x40) >> 6) : -1);
  in_b_vec.push_back((buf[13] & 0x30) == 0x10 ? ((buf[13] & 0x80) >> 7) : -1);
  in_a_vec.push_back((buf[14] & 0x03) == 0x01 ? ((buf[14] & 0x04) >> 2) : -1);
  in_b_vec.push_back((buf[14] & 0x03) == 0x01 ? ((buf[14] & 0x08) >> 3) : -1);
  in_a_vec.push_back((buf[14] & 0x30) == 0x10 ? ((buf[14] & 0x40) >> 6) : -1);
  in_b_vec.push_back((buf[14] & 0x30) == 0x10 ? ((buf[14] & 0x80) >> 7) : -1);

  output_vec.push_back((buf[13] & 0x03) == 0x02 ? ((buf[13] & 0x04) >> 2) : -1);
  output_vec.push_back((buf[13] & 0x30) == 0x20 ? ((buf[13] & 0x40) >> 6) : -1);
  output_vec.push_back((buf[14] & 0x03) == 0x02 ? ((buf[14] & 0x04) >> 2) : -1);
  output_vec.push_back((buf[14] & 0x30) == 0x20 ? ((buf[14] & 0x40) >> 6) : -1);

  ossd_a_1_str += "\"ossd_a_1\": " + std::to_string(ossd_a_1) + "";
  ossd_b_1_str += "\"ossd_b_1\": " + std::to_string(ossd_b_1) + "";

  input_a_str += "\"input_a\": [";
  for (int i = 0; i < input_a_vec.size(); i++) {
    input_a_str += std::to_string(input_a_vec[i]);
    if (i < input_a_vec.size() - 1) {
      input_a_str += ", ";
    }
  }
  input_a_str += "]";

  input_b_str += "\"input_b\": [";
  for (int i = 0; i < input_b_vec.size(); i++) {
    input_b_str += std::to_string(input_b_vec[i]);
    if (i < input_b_vec.size() - 1) {
      input_b_str += ", ";
    }
  }
  input_b_str += "]";

  in_a_str += "\"in_io_a\": [";
  for (int i = 0; i < in_a_vec.size(); i++) {
    in_a_str += std::to_string(in_a_vec[i]);
    if (i < in_a_vec.size() - 1) {
      in_a_str += ", ";
    }
  }
  in_a_str += "]";

  in_b_str += "\"in_io_b\": [";
  for (int i = 0; i < in_b_vec.size(); i++) {
    in_b_str += std::to_string(in_b_vec[i]);
    if (i < in_b_vec.size() - 1) {
      in_b_str += ", ";
    }
  }
  in_b_str += "]";

  output_str += "\"out_io\": [";
  for (int i = 0; i < output_vec.size(); i++) {
    output_str += std::to_string(output_vec[i]);
    if (i < output_vec.size() - 1) {
      output_str += ", ";
    }
  }
  output_str += "]";

  ret_str = ossd_a_1_str + R"(,
    )" + ossd_b_1_str +
            R"(,
    )" + input_a_str +
            R"(,
    )" + input_b_str +
            R"(,
    )" + in_a_str +
            R"(,
    )" + in_b_str +
            R"(,
    )" + output_str;
  return ret_str;
}

template <typename T_PointCloud>
std::string DecoderVanjee719E<T_PointCloud>::getZoneAlarmCaseInfo(std::string key_info, std::string key_speed, uint8_t *buf) {
  std::string ret_str = "";
  std::string intrusion_info_str = "";
  std::string input_speed_str = "";
  std::vector<uint16_t> intrusion_info_id;
  uint16_t seq = buf[8] & 0x03;
  uint16_t case_no = buf[9];
  uint16_t intrusion_info = buf[10];
  int32_t input_speed_1 = buf[11] | (buf[12] << 8);
  int32_t input_speed_2 = buf[13] | (buf[14] << 8);

  input_speed_str += "\"" + key_speed + "\": [" + std::to_string(input_speed_1) + ", " + std::to_string(input_speed_2) + "]";

  if (case_no != 0) {
    for (int i = 0; i < 8; i++) {
      if ((intrusion_info & (0x01 << i)) == (0x01 << i)) {
        intrusion_info_id.push_back(i + 1);
      }
    }
    if (intrusion_info_id.size() > 0) {
      intrusion_info_str += "\"" + key_info + "\": [";

      for (uint16_t i = 0; i < intrusion_info_id.size(); i++) {
        intrusion_info_str += std::to_string(intrusion_info_id[i]);
        if (i < intrusion_info_id.size() - 1) {
          intrusion_info_str += ", ";
        }
      }
      intrusion_info_str += "]";
    } else {
      intrusion_info_str += "\"" + key_info + "\": []";
    }
  } else {
    intrusion_info_str += "\"" + key_info + "\": []";
  }
  ret_str = intrusion_info_str + R"(,
    )" + input_speed_str;
  return ret_str;
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::zoneAlarmUpload(int32_t hor_angle, uint8_t *buf, size_t size) {
  if (this->param_.send_lidar_param_enable && size == 16) {
    if ((get_zone_alarm_data_flag_ & 0x0E) != 0x0E) {
      std::vector<uint16_t> intrusion_info_id;
      uint16_t seq = buf[8] & 0x03;
      if (seq == 0 && (get_zone_alarm_data_flag_ & 0x02) == 0) {
        io_info_str_ = getZoneAlarmIOInfo(buf);
        get_zone_alarm_data_flag_ |= 0x02;
      } else if (seq == 1 && (get_zone_alarm_data_flag_ & 0x04) == 0) {
        intrusion_info_str_1_ = getZoneAlarmCaseInfo("intrusion_info_arm1", "input_speed_arm1", buf);
        get_zone_alarm_data_flag_ |= 0x04;
      } else if (seq == 2 && (get_zone_alarm_data_flag_ & 0x08) == 0) {
        intrusion_info_str_2_ = getZoneAlarmCaseInfo("intrusion_info_arm2", "input_speed_arm2", buf);
        get_zone_alarm_data_flag_ |= 0x08;
      }
    }

    if (hor_angle == 180000) {
      std::vector<uint16_t> vec_id;
      std::string zone_alarm_ossd_str = "";
      std::string zone_alarm_io_str = "";
      uint8_t lidar_state = buf[0];
      uint8_t ossd_state = buf[1];
      uint8_t io_state = buf[2];
      uint8_t warning_chan = buf[3];

      if ((ossd_state & 0xff) == 0) {
        zone_alarm_ossd_str += "\"ossd\": [1, 2]";
      } else if ((ossd_state & 0x0f) == 0) {
        zone_alarm_ossd_str += "\"ossd\": [1]";
      } else if ((ossd_state & 0xf0) == 0) {
        zone_alarm_ossd_str += "\"ossd\": [2]";
      } else {
        zone_alarm_ossd_str += "\"ossd\": []";
      }

      for (uint16_t i = 0; i < 4; i++) {
        if ((io_state & (0x3 << (i * 2))) == 0) {
          vec_id.push_back(i + 1);
        }
      }

      if (vec_id.size() > 0) {
        zone_alarm_io_str += "\"io\": [";
        for (uint16_t i = 0; i < vec_id.size(); i++) {
          zone_alarm_io_str += std::to_string(vec_id[i]);
          if (i < vec_id.size() - 1) {
            zone_alarm_io_str += ", ";
          }
        }
        zone_alarm_io_str += "]";
      } else {
        zone_alarm_io_str += "\"io\": []";
      }

      if (zone_alarm_ossd_str != "" && zone_alarm_io_str != "") {
        zone_alarm_state_ = zone_alarm_ossd_str + R"(,
    )" + zone_alarm_io_str;
      } else {
        if (zone_alarm_ossd_str != "") {
          zone_alarm_state_ = zone_alarm_ossd_str;
        } else {
          zone_alarm_state_ = zone_alarm_io_str;
        }
      }

      get_zone_alarm_data_flag_ |= 0x01;
    }

    if ((get_zone_alarm_data_flag_ & 0x0f) == 0x0f) {
      std::string zone_alarm_str;
      if (zone_alarm_state_ != "" && intrusion_info_str_1_ != "" && intrusion_info_str_2_ != "") {
        zone_alarm_str += R"(
  {
    )" + zone_alarm_state_ +
                          R"(,
    )" + io_info_str_ + R"(,
    )" + intrusion_info_str_1_ +
                          R"(,
    )" + intrusion_info_str_2_ +
                          R"(
  })";

        LidarParameterInterface lidar_param;
        lidar_param.cmd_id = (uint16_t)LidarParam::zone_alarm;
        lidar_param.cmd_type = 0;
        lidar_param.repeat_interval = 0;

        if (get_lidar_param_.count((uint16_t)LidarParam::zone_alarm) > 0) {
          get_lidar_param_[(uint16_t)LidarParam::zone_alarm] = zone_alarm_str;
        } else {
          get_lidar_param_.emplace((uint16_t)LidarParam::zone_alarm, zone_alarm_str);
        }
        this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
        this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
      }
      get_zone_alarm_data_flag_ = 0;
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info,
                                                                          Vanjee719EDataBlock *data_block) {
  int32_t hor_angle = ((data_block->azimuth * 10) + 360000) % 360000;
  uint16_t hor_resolution =
      (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[0] + (data_block_angle_and_timestamp_info.reserved_field_1_[1] << 8));
  int32_t col_index = hor_angle / hor_resolution;

  if (point_id_offset_ != pre_point_id_offset_) {
    uint32_t point_num =
        ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ * 4) +
        (data_block_angle_and_timestamp_info.data_block_index_in_packet_ * 4);
    uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_;
    point_id_offset_ = point_id - point_num;
  }

  initLdLuminousMoment((uint16_t)(data_block_angle_and_timestamp_info.operate_frequency_ * 0.01), point_id_offset_);

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    // uint32_t point_id = (col_index + 1) * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ - 1 -
    // point_id_offset_;
    if (!this->param_.use_lidar_clock) {
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          decoder_packet_base_->decoder_packet_general_version_base_ptr_
              ->all_points_luminous_moment_[decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];
    } else {
      data_block_angle_and_timestamp_info.first_point_ts_ -= (25567LL * 24 * 3600);
      if (data_block_angle_and_timestamp_info.first_point_ts_ < 0) {
        data_block_angle_and_timestamp_info.first_point_ts_ = 0;
      }
    }
  }

  // zoneAlarmUpload(hor_angle, data_block_angle_and_timestamp_info.reserved_field_2_, sizeof(data_block_angle_and_timestamp_info.reserved_field_2_));

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t row_index = chan_id;
      uint32_t point_id =
          col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index - point_id_offset_;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point)
        timestamp_point = decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      else
        timestamp_point =
            decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];
      PointInfo point_info;
      // point_info.distance_ = 0;
      point_info.azimuth_ = ((540000 - hor_angle) % 360000) * 1e-3;
      point_info.elevation_ = lidar_ver_angle_[row_index] * 1e-3;
      // point_info.reflectivity_ = 0;
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.tag_ = 0;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::decoderDataUnitCallback(Vanjee719EDataUnit *data_unit, PointInfo &point_info) {
  point_info.distance_ = data_unit->distance;
  // point_info.azimuth_ = point_info.azimuth_;
  // point_info.elevation_ = point_info.elevation_;
  point_info.reflectivity_ = data_unit->reflectivity;
  // point_info.ring_ = point_info.ring_;
  // point_info.timestamp_ = point_info.timestamp_;
  // point_info.tag_ = point_info.tag_;
  // point_info.id_ = point_info.id_;
}

template <typename T_PointCloud>
inline bool DecoderVanjee719E<T_PointCloud>::decodeMsopPktBaseProtocol(const uint8_t *pkt, size_t size) {
  return decoder_packet_base_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee719E::updateAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee719E::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T_PointCloud>
inline void DecoderVanjee719E<T_PointCloud>::detectDifopTsProcess() {
  while (!exit_detect_flag_) {
    double cur_ts = getTimeHost() * 1e-6;
    if (get_field_data_flag_ && cur_ts - start_recv_difop_field_ts_ > 1.0 && start_recv_difop_field_ts_ > 0.0) {
      recv_field_data_total_pkt_num_ = 0;
      field_data_.clear();
      per_field_data_criterion_ = std::vector<uint32_t>(128, 0);
      per_field_data_flag_ = std::vector<uint32_t>(128, 0);
      range_min_ = std::vector<std::vector<float>>(128, std::vector<float>(1381, 500.0f));
      range_max_ = std::vector<std::vector<float>>(128, std::vector<float>(1381, 500.0f));
      get_field_data_flag_ = false;

      (*(this->get_difo_ctrl_map_ptr_))[CmdRepository719E::CreateInstance()->sp_field_data_get_->GetCmdKey()].setStopFlag(false);
      WJ_INFOL << "Retrieve data again - field data" << WJ_REND;
    }

    if (get_monitoring_case_flag_ && cur_ts - start_recv_difop_monitoring_ts_ > 1.0 && start_recv_difop_monitoring_ts_ > 0.0) {
      recv_monitoring_case_total_pkt_num_ = 0;
      monitoring_case_.clear();
      get_monitoring_case_flag_ = false;

      (*(this->get_difo_ctrl_map_ptr_))[CmdRepository719E::CreateInstance()->sp_monitoring_case_get_->GetCmdKey()].setStopFlag(false);
      WJ_INFOL << "Retrieve data again - monitoring case" << WJ_REND;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract719E> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository719E::CreateInstance()->sp_scan_data_get_)) {
    p = std::make_shared<Protocol_ScanDataGet719E>();
  } else if (*sp_cmd == *(CmdRepository719E::CreateInstance()->sp_firmware_version_get_)) {
    p = std::make_shared<Protocol_FirmwareVersionGet719E>();
  } else if (*sp_cmd == *(CmdRepository719E::CreateInstance()->sp_field_data_get_)) {
    p = std::make_shared<Protocol_FieldDataGet719E>();
  } else if (*sp_cmd == *(CmdRepository719E::CreateInstance()->sp_monitoring_case_get_)) {
    p = std::make_shared<Protocol_MonitoringCaseGet719E>();
  } else if (*sp_cmd == *(CmdRepository719E::CreateInstance()->sp_verify_results_get_)) {
    p = std::make_shared<Protocol_VerifyResultsGet719E>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_ScanData719E)) {
    if (lidar_mac_ == "") {
      std::shared_ptr<Params_ScanData719E> param = std::dynamic_pointer_cast<Params_ScanData719E>(params);
      std::stringstream ss;
      ss << std::hex << std::setw(2) << std::setfill('0') << ((uint16_t)param->mac_[0] & 0xff) << "-" << ((uint16_t)param->mac_[1] & 0xff) << "-"
         << ((uint16_t)param->mac_[2] & 0xff) << "-" << ((uint16_t)param->mac_[3] & 0xff) << "-" << ((uint16_t)param->mac_[4] & 0xff) << "-"
         << ((uint16_t)param->mac_[5] & 0xff);
      lidar_mac_ = ss.str();
    }

    if (this->param_.send_lidar_param_enable) {
      if (firmware_version_ != "" && lidar_mac_ != "") {
        auto it = this->getLidarParam((uint16_t)LidarParam::basic_param);
        if (it != this->lidar_param_vec_.end()) {
          WJ_INFOL << "Get lidar mac succ ( " << lidar_mac_ << " )" << WJ_REND;

          std::string basic_param_str = R"(
  {
    "lidar_type": "WLR-719E",
    "mac": ")" + lidar_mac_ + R"(",
    "firmware_version": ")" + firmware_version_ +
                                        R"("
  })";

          if (get_lidar_param_.count((uint16_t)LidarParam::basic_param > 0)) {
            get_lidar_param_[(uint16_t)LidarParam::basic_param] = basic_param_str;
          } else {
            get_lidar_param_.emplace((uint16_t)LidarParam::basic_param, basic_param_str);
          }

          LidarParameterInterface lidar_param;
          lidar_param.cmd_id = (uint16_t)LidarParam::basic_param;
          lidar_param.cmd_type = 0;
          lidar_param.repeat_interval = 0;
          this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
          this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
          firmware_version_ = "";
          lidar_mac_ = "";

          this->delLidarParam(it);
        }
      }
    }

  } else if (typeid(*params) == typeid(Params_FirmwareVersion719E)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_FirmwareVersion719E> param = std::dynamic_pointer_cast<Params_FirmwareVersion719E>(params);

    if (get_lidar_param_.count((uint16_t)LidarParam::firmware_version) > 0) {
      get_lidar_param_[(uint16_t)LidarParam::firmware_version] = param->firmware_version_;
    } else {
      get_lidar_param_.emplace((uint16_t)LidarParam::firmware_version, param->firmware_version_);
    }

    firmware_version_ = param->firmware_version_;
    if (this->param_.send_lidar_param_enable) {
      auto it = this->getLidarParam((uint16_t)LidarParam::firmware_version);
      if (it != this->lidar_param_vec_.end()) {
        LidarParameterInterface lidar_param;
        lidar_param.cmd_id = (uint16_t)LidarParam::firmware_version;
        lidar_param.cmd_type = 0;
        lidar_param.repeat_interval = 0;
        this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
        this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);

        this->delLidarParam(it);
      }

      if (firmware_version_ != "" && lidar_mac_ != "") {
        auto it = this->getLidarParam((uint16_t)LidarParam::basic_param);
        if (it != this->lidar_param_vec_.end()) {
          std::string basic_param_str = R"(
  {
    "lidar_type": "WLR-719E",
    "mac": ")" + lidar_mac_ + R"(",
    "firmware_version": ")" + firmware_version_ +
                                        R"("
  })";

          if (get_lidar_param_.count((uint16_t)LidarParam::basic_param > 0)) {
            get_lidar_param_[(uint16_t)LidarParam::basic_param] = basic_param_str;
          } else {
            get_lidar_param_.emplace((uint16_t)LidarParam::basic_param, basic_param_str);
          }

          LidarParameterInterface lidar_param;
          lidar_param.cmd_id = (uint16_t)LidarParam::basic_param;
          lidar_param.cmd_type = 0;
          lidar_param.repeat_interval = 0;
          this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
          this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
          firmware_version_ = "";
          lidar_mac_ = "";

          this->delLidarParam(it);
        }
      }
    }

    WJ_INFOL << "Get lidar firmware version succ ( " << param->firmware_version_ << " )" << WJ_REND;

  } else if (typeid(*params) == typeid(Params_FieldData719E)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_FieldData719E> param = std::dynamic_pointer_cast<Params_FieldData719E>(params);
    start_recv_difop_field_ts_ = getTimeHost() * 1e-6;
    if (!param->valid_data_flag_) {
      return;
    }
    get_field_data_flag_ = true;
    if (!param->last_pkt_flag_) {
      if (per_field_data_criterion_[param->field_id_] == 0) {
        for (uint32_t i = 0; i < param->total_pkt_num_; i++) {
          per_field_data_criterion_[param->field_id_] |= (0x01 << i);
        }
      }
      for (uint32_t i = 0; i < 1381; i++) {
        if (param->range_min_[i] >= 0 && param->range_min_[i] < 500.0) {
          if (range_min_[param->field_id_][i] >= 0 && range_min_[param->field_id_][i] < 500.0) {
            range_min_[param->field_id_][i] = std::min(range_min_[param->field_id_][i], param->range_min_[i]);
          } else {
            range_min_[param->field_id_][i] = param->range_min_[i];
          }
        }
        if (param->range_max_[i] > 0 && param->range_max_[i] < 500.0) {
          if (range_max_[param->field_id_][i] > 0 && range_max_[param->field_id_][i] < 500.0) {
            range_max_[param->field_id_][i] = std::max(range_max_[param->field_id_][i], param->range_max_[i]);
          } else {
            range_max_[param->field_id_][i] = param->range_max_[i];
          }
        }
      }
      per_field_data_flag_[param->field_id_] |= (0x01 << (param->pkt_id_ - 1));
      if (per_field_data_flag_[param->field_id_] == per_field_data_criterion_[param->field_id_]) {
        per_field_data_flag_[param->field_id_] = 0;

        VanjeeFieldData719E field_data;
        field_data.field_data_id = param->field_id_;
        field_data.line_id = param->line_id_;
        field_data.start_angle = 138.0 / 180.0 * M_PI;
        field_data.angle_increment = -0.2 / 180.0 * M_PI;
        field_data.angular_resolution = 0.2 / 180.0 * M_PI;
        field_data.field_definition = param->field_definition_;
        for (uint32_t i = 0; i < 1381; i++) {
          field_data.range_min[i] = range_min_[param->field_id_][i];
          field_data.range_max[i] = range_max_[param->field_id_][i];
        }
        field_data_.emplace_back(field_data);
      }
      recv_field_data_total_pkt_num_++;
    } else {
      get_field_data_flag_ = false;
      if (this->param_.send_lidar_param_enable && recv_field_data_total_pkt_num_ == param->send_total_pkt_num_) {
        if (field_data_.size() > 0) {
          auto it = this->getLidarParam((uint16_t)LidarParam::safety_field_data);
          WJ_INFOL << "Get field data succ ( field num: " << field_data_.size() << " )" << WJ_REND;

          std::string field_data_str = R"(
  {
    "fields":[
)";

          for (uint32_t i = 0; i < field_data_.size(); i++) {
            std::string min_str = "[";
            std::string max_str = "[";

            for (uint32_t index = 0; index < range_min_[field_data_[i].field_data_id].size(); index++) {
              if (index < range_min_[field_data_[i].field_data_id].size() - 1) {
                min_str += std::to_string(range_min_[field_data_[i].field_data_id][index]) + ", ";
                max_str += std::to_string(range_max_[field_data_[i].field_data_id][index]) + ", ";
              } else {
                min_str += std::to_string(range_min_[field_data_[i].field_data_id][index]) + "]";
                max_str += std::to_string(range_max_[field_data_[i].field_data_id][index]) + "]";
              }
            }
            std::string per_field_data_str = R"(
    {
      "field_data_id": )" + std::to_string(field_data_[i].field_data_id) +
                                             R"(,
      "line_id": )" + std::to_string((uint16_t)(field_data_[i].line_id)) +
                                             R"(,
      "range_max": )" + max_str + R"(,
      "range_min": )" + min_str + R"(,
      "start_angle": )" + std::to_string(field_data_[i].start_angle) +
                                             R"(,
      "angle_increment": )" + std::to_string(field_data_[i].angle_increment) +
                                             R"(,
      "angular_resolution": )" + std::to_string(field_data_[i].angular_resolution) +
                                             R"(,
      "field_definition": )" + std::to_string(field_data_[i].field_definition) +
                                             R"(
    })";
            field_data_str += per_field_data_str;
            if (i < field_data_.size() - 1) {
              field_data_str += ",";
            }
          }
          field_data_str += R"(
    ]
  })";

          if (it != this->lidar_param_vec_.end()) {
            LidarParameterInterface lidar_param;
            lidar_param.cmd_id = (uint16_t)LidarParam::safety_field_data;
            lidar_param.cmd_type = 0;
            lidar_param.repeat_interval = 0;
            lidar_param.data = field_data_str;
            this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);

            this->delLidarParam(it);
          }
        }
      }

      recv_field_data_total_pkt_num_ = 0;
      field_data_.clear();
      per_field_data_criterion_ = std::vector<uint32_t>(128, 0);
      per_field_data_flag_ = std::vector<uint32_t>(128, 0);
      range_min_ = std::vector<std::vector<float>>(128, std::vector<float>(1381, 500.0f));
      range_max_ = std::vector<std::vector<float>>(128, std::vector<float>(1381, 500.0f));

      uint16_t cmd = CmdRepository719E::CreateInstance()->sp_monitoring_case_get_->GetCmdKey();
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[cmd].setStopFlag(false);
    }

  } else if (typeid(*params) == typeid(Params_MonitoringCase719E)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_MonitoringCase719E> param = std::dynamic_pointer_cast<Params_MonitoringCase719E>(params);
    start_recv_difop_monitoring_ts_ = getTimeHost() * 1e-6;
    get_monitoring_case_flag_ = true;
    if (!param->last_pkt_flag_) {
      for (int case_index = 0; case_index < param->data_.size(); case_index++) {
        VanjeeMonitoringCase719E monitoring_case;
        uint16_t static_input = (param->data_[case_index].static_input & 0x03);
        uint16_t dynamic_input = (param->data_[case_index].dynamic_input[0] & 0x03);
        if (static_input != 0) {
          if (dynamic_input == 0) {
            monitoring_case.monitoring_case_type = static_input == 1 ? 1 : 0;
          } else {
            if (static_input == 2 && dynamic_input == 1) {
              monitoring_case.monitoring_case_type = 4;
            } else if (static_input == 2 && dynamic_input == 2) {
              monitoring_case.monitoring_case_type = 5;
            } else if (static_input == 1 && dynamic_input == 1) {
              monitoring_case.monitoring_case_type = 6;
            } else if (static_input == 1 && dynamic_input == 2) {
              monitoring_case.monitoring_case_type = 7;
            } else {
              monitoring_case.monitoring_case_type = -1;
            }
          }
        } else {
          if (dynamic_input == 0) {
            monitoring_case.monitoring_case_type = -1;
          } else {
            monitoring_case.monitoring_case_type = dynamic_input == 1 ? 2 : 3;
          }
        }
        monitoring_case.monitoring_case_list_id = param->monitoring_case_list_id_;
        monitoring_case.monitoring_case_id = param->data_[case_index].case_id;

        if (monitoring_case.monitoring_case_type == 0 || monitoring_case.monitoring_case_type == 4 || monitoring_case.monitoring_case_type == 5) {
          monitoring_case.input_io.resize(4);
          monitoring_case.input_in.resize(8);

          for (int i = 0; i < (monitoring_case.input_io.size() / 2); i++) {
            uint8_t flag = (param->data_[case_index].static_input >> (2 * i + 2)) & 0x03;
            if (flag == 0) {
              monitoring_case.input_io[2 * i] = -1;
              monitoring_case.input_io[2 * i + 1] = -1;
            } else {
              monitoring_case.input_io[2 * i] = (flag & 0x01);
              monitoring_case.input_io[2 * i + 1] = ((flag >> 1) & 0x01);
            }
          }
          for (int i = 0; i < (monitoring_case.input_in.size() / 2); i++) {
            uint8_t flag = (param->data_[case_index].static_input >> (6 + (2 * i))) & 0x03;
            if (flag == 0) {
              monitoring_case.input_in[2 * i] = -1;
              monitoring_case.input_in[2 * i + 1] = -1;
            } else {
              monitoring_case.input_in[2 * i] = (flag & 0x01);
              monitoring_case.input_in[2 * i + 1] = ((flag >> 1) & 0x01);
            }
          }
        }

        if (monitoring_case.monitoring_case_type == 1 || monitoring_case.monitoring_case_type == 6 || monitoring_case.monitoring_case_type == 7) {
          monitoring_case.input_io.resize(2);
          monitoring_case.input_in.resize(4);
          for (int i = 0; i < monitoring_case.input_io.size(); i++) {
            uint8_t flag = (param->data_[case_index].static_input >> (2 * i + 2)) & 0x03;
            if (flag == 0) {
              monitoring_case.input_io[i] = -1;
            } else if (flag == 1) {
              monitoring_case.input_io[i] = 1;
            } else if (flag == 2) {
              monitoring_case.input_io[i] = 0;
            } else if (flag == 3) {
              monitoring_case.input_io[i] = 2;
            }
          }
          for (int i = 0; i < monitoring_case.input_in.size(); i++) {
            uint8_t flag = (param->data_[case_index].static_input >> (6 + (2 * i))) & 0x03;
            if (flag == 0) {
              monitoring_case.input_in[i] = -1;
            } else if (flag == 1) {
              monitoring_case.input_in[i] = 1;
            } else if (flag == 2) {
              monitoring_case.input_in[i] = 0;
            } else if (flag == 3) {
              monitoring_case.input_in[i] = 2;
            }
          }
        }

        if (monitoring_case.monitoring_case_type >= 2 && monitoring_case.monitoring_case_type <= 7) {
          monitoring_case.range.resize(2);
          int16_t min_speed = (int16_t)(param->data_[case_index].dynamic_input[1] | (param->data_[case_index].dynamic_input[2] << 8));
          int16_t max_speed = (int16_t)(param->data_[case_index].dynamic_input[3] | (param->data_[case_index].dynamic_input[4] << 8));
          monitoring_case.range[0] = min_speed;
          monitoring_case.range[1] = max_speed;
        }

        for (int i = 0; i < param->data_[case_index].intrusion_num; i++) {
          monitoring_case.case_info.emplace_back(param->data_[case_index].intrusion[i].info[0]);
          std::vector<int32_t> value;
          for (int j = 0; j < 6; j++) {
            value.emplace_back(((int32_t)(param->data_[case_index].intrusion[i].info[1] >> j) & 0x01));
          }
          monitoring_case.output_io.emplace_back(value);
        }
        monitoring_case_.emplace_back(monitoring_case);
      }
      recv_monitoring_case_total_pkt_num_++;
    } else {
      get_monitoring_case_flag_ = false;
      if (this->param_.send_lidar_param_enable && recv_monitoring_case_total_pkt_num_ == param->send_total_pkt_num_) {
        if (monitoring_case_.size() > 0) {
          auto it = this->getLidarParam((uint16_t)LidarParam::safety_monitoring_case);
          WJ_INFOL << "Get monitoring case succ ( case num: " << monitoring_case_.size() << " )" << WJ_REND;

          std::string monitoring_case_str = R"(
  {
    "monitoring_case": [
)";

          for (uint32_t i = 0; i < monitoring_case_.size(); i++) {
            std::string input_io_str = "[";
            std::string input_in_str = "[";
            std::string range_str = "[";
            std::string output_io_str = "[";
            std::string case_info_str = "[";

            if (monitoring_case_[i].input_io.size() > 0) {
              for (uint32_t index = 0; index < monitoring_case_[i].input_io.size(); index++) {
                if (index < monitoring_case_[i].input_io.size() - 1) {
                  input_io_str += std::to_string(monitoring_case_[i].input_io[index]) + ", ";
                } else {
                  input_io_str += std::to_string(monitoring_case_[i].input_io[index]) + "]";
                }
              }
            } else {
              input_io_str += "]";
            }

            if (monitoring_case_[i].input_in.size() > 0) {
              for (uint32_t index = 0; index < monitoring_case_[i].input_in.size(); index++) {
                if (index < monitoring_case_[i].input_in.size() - 1) {
                  input_in_str += std::to_string(monitoring_case_[i].input_in[index]) + ", ";
                } else {
                  input_in_str += std::to_string(monitoring_case_[i].input_in[index]) + "]";
                }
              }
            } else {
              input_in_str += "]";
            }

            if (monitoring_case_[i].range.size() > 0) {
              for (uint32_t index = 0; index < monitoring_case_[i].range.size(); index++) {
                if (index < monitoring_case_[i].range.size() - 1) {
                  range_str += std::to_string(monitoring_case_[i].range[index]) + ", ";
                } else {
                  range_str += std::to_string(monitoring_case_[i].range[index]) + "]";
                }
              }
            } else {
              range_str += "]";
            }

            if (monitoring_case_[i].case_info.size() > 0) {
              for (uint32_t index = 0; index < monitoring_case_[i].case_info.size(); index++) {
                if (index < monitoring_case_[i].case_info.size() - 1) {
                  case_info_str += std::to_string(monitoring_case_[i].case_info[index]) + ", ";
                } else {
                  case_info_str += std::to_string(monitoring_case_[i].case_info[index]) + "]";
                }
              }
            } else {
              case_info_str += "]";
            }

            if (monitoring_case_[i].output_io.size() > 0) {
              for (uint32_t index_intrusion = 0; index_intrusion < monitoring_case_[i].output_io.size(); index_intrusion++) {
                output_io_str += "[";
                if (monitoring_case_[i].output_io[index_intrusion].size() > 0) {
                  for (uint32_t index_io = 0; index_io < monitoring_case_[i].output_io[index_intrusion].size(); index_io++) {
                    if (index_io < monitoring_case_[i].output_io[index_intrusion].size() - 1) {
                      output_io_str += std::to_string(monitoring_case_[i].output_io[index_intrusion][index_io]) + ", ";
                    } else {
                      output_io_str += std::to_string(monitoring_case_[i].output_io[index_intrusion][index_io]) + "]";
                    }
                  }
                  if (index_intrusion != monitoring_case_[i].output_io.size() - 1) {
                    output_io_str += ", ";
                  }
                } else {
                  output_io_str += "]";
                }
                if (index_intrusion == monitoring_case_[i].output_io.size() - 1) {
                  output_io_str += "]";
                }
              }
            } else {
              output_io_str += "]";
            }

            std::string per_monitoring_case_str = R"(
    {
      "monitoring_case_list_id": )" + std::to_string(monitoring_case_[i].monitoring_case_list_id) +
                                                  R"(,
      "monitoring_case_id": )" + std::to_string(monitoring_case_[i].monitoring_case_id) +
                                                  R"(,
      "monitoring_case_type": )" + std::to_string(monitoring_case_[i].monitoring_case_type) +
                                                  R"(,
      "input_io": )" + input_io_str + R"(,
      "input_in": )" + input_in_str + R"(,
      "range": )" + range_str + R"(,
      "output_io": )" + output_io_str + R"(,
      "case_info": )" + case_info_str + R"(
    })";
            monitoring_case_str += per_monitoring_case_str;
            if (i < field_data_.size() - 1) {
              monitoring_case_str += ",";
            }
          }
          monitoring_case_str += R"(
    ]
  })";

          if (it != this->lidar_param_vec_.end()) {
            LidarParameterInterface lidar_param;
            lidar_param.cmd_id = (uint16_t)LidarParam::safety_monitoring_case;
            lidar_param.cmd_type = 0;
            lidar_param.repeat_interval = 0;
            lidar_param.data = monitoring_case_str;
            this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);

            this->delLidarParam(it);
          }
        }
      }

      recv_monitoring_case_total_pkt_num_ = 0;
      monitoring_case_.clear();

      uint16_t cmd = CmdRepository719E::CreateInstance()->sp_verify_results_get_->GetCmdKey();
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[cmd].setStopFlag(false);
    }

  } else if (typeid(*params) == typeid(Param_VerifyResultsGet719E)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Param_VerifyResultsGet719E> param = std::dynamic_pointer_cast<Param_VerifyResultsGet719E>(params);
    WJ_INFOL << "Get verify resultS succ" << field_data_.size() << " )" << WJ_REND;

  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar
}  // namespace vanjee