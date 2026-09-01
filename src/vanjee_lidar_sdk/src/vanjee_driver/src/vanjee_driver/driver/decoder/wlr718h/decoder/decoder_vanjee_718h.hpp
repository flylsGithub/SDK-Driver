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
#include <vanjee_driver/driver/decoder/wlr718h/filter/gs_filter.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/cmd_repository_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_bank_id_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_bank_id_set.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_bank_switch_type_set.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_firmware_version_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_network_param_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_topboard_operate_params_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)

typedef struct _Vanjee718hHeaderPkt {
  uint8_t header[2];
  uint16_t frame_len;
  uint16_t frame_id;
  uint32_t timestamp;
  uint8_t check_type;
  uint8_t frame_type;
  uint16_t device_type;
  uint8_t remain1[6];
  uint16_t protocol_version;
  uint8_t main_cmd;
  uint8_t sub_cmd;
  uint8_t cmd_param1;
  uint8_t cmd_param2;

  void ToLittleEndian() {
    frame_len = ntohs(frame_len);
    frame_id = ntohs(frame_id);
    timestamp = ntohl(timestamp);
    device_type = ntohs(device_type);
    protocol_version = ntohs(protocol_version);
  }
} Vanjee718hHeaderPkt;

typedef struct _Vanjee718hMsopPkt {
  Vanjee718hHeaderPkt header;

  uint16_t device_status;
  uint16_t watchdog_reset_num;
  uint16_t software_reset_num;
  uint16_t loss_elec_reset_num;
  uint16_t serial_port_pulse_width;
  uint16_t bot_board_serial_port_threshold;
  uint16_t offset;
  uint16_t detected_encoder_groove_time_interval;
  uint16_t motor_speed_adjust_compare_value;
  uint16_t motor_speed;
  uint16_t top_board_SPAD_voltage;
  uint16_t top_board_temperature;
  uint16_t bot_board_voltage;
  uint16_t bot_board_voltage_compare_value;
  uint16_t link_disconnect_num;
  uint16_t reconnect_num;
  uint16_t disconnect_num;
  uint16_t clog_num;
  uint16_t resend_failure_num;
  uint16_t heartbeat_disconnect_num;
  uint16_t keepalive_disconnect_num;
  uint8_t input_IO_value;
  uint8_t remain2;
  uint8_t turn_source;
  uint8_t cur_bank_id;
  uint8_t zone_type;
  uint8_t zone_output_value;

  uint32_t circle_id;
  uint8_t intensity_flag;
  uint8_t resolution;
  uint8_t total_pkts_num;
  uint8_t pkt_id;
  uint8_t pkt_type;
  uint16_t points_num;

  void ToLittleEndian() {
    // header.ToLittleEndian();

    device_status = ntohs(device_status);
    watchdog_reset_num = ntohs(watchdog_reset_num);
    software_reset_num = ntohs(software_reset_num);
    loss_elec_reset_num = ntohs(loss_elec_reset_num);
    serial_port_pulse_width = ntohs(serial_port_pulse_width);
    bot_board_serial_port_threshold = ntohs(bot_board_serial_port_threshold);
    offset = ntohs(offset);
    detected_encoder_groove_time_interval = ntohs(detected_encoder_groove_time_interval);
    motor_speed_adjust_compare_value = ntohs(motor_speed_adjust_compare_value);
    motor_speed = ntohs(motor_speed);
    top_board_SPAD_voltage = ntohs(top_board_SPAD_voltage);
    top_board_temperature = ntohs(top_board_temperature);
    bot_board_voltage = ntohs(bot_board_voltage);
    bot_board_voltage_compare_value = ntohs(bot_board_voltage_compare_value);
    link_disconnect_num = ntohs(link_disconnect_num);
    reconnect_num = ntohs(reconnect_num);
    disconnect_num = ntohs(disconnect_num);
    clog_num = ntohs(clog_num);
    resend_failure_num = ntohs(resend_failure_num);
    heartbeat_disconnect_num = ntohs(heartbeat_disconnect_num);
    keepalive_disconnect_num = ntohs(keepalive_disconnect_num);
    circle_id = ntohl(circle_id);
    points_num = ntohs(points_num);
  }
} Vanjee718hMsopPkt;
#pragma pack(pop)

typedef struct _PointDXYZIRTT {
  float the_1st_echo_distance;
  float the_1st_echo_x;
  float the_1st_echo_y;
  float the_1st_echo_z;
  float the_2nd_echo_distance;
  float the_2nd_echo_x;
  float the_2nd_echo_y;
  float the_2nd_echo_z;
  float intensity;
  double timestamp;
  int ring;
  uint8_t tag;
} PointDXYZIRTT;

template <typename T_PointCloud>
class DecoderVanjee718H : public Decoder<T_PointCloud> {
 private:
  static constexpr size_t POINTS_PER_CIRCLE = 1440;
  static constexpr double ANGLE_RESOLUTION = 0.25;
  static constexpr double TIME_FRAC_SCALE = 4294967296.0;  // 2^32
  static constexpr size_t POINTS_DATA_OFFSET = sizeof(Vanjee718hMsopPkt);

  std::vector<std::vector<double>> all_points_luminous_moment_718h_;  // Cache a circle of point cloud time difference
  double luminous_period_of_ld_10hz_ = 0.0000693;                     // Time interval at adjacent horizontal angles(10hz)
  double luminous_period_of_ld_15hz_ = 0.0000462;                     // Time interval at adjacent horizontal angles(15hz)
  double luminous_period_of_ld_14hz_ = 0.0000496;                     // Time interval at adjacent horizontal angles(14hz)

  bool scan_data_recv_flag_ = false;
  double pkt_ts_ = 0;
  uint8_t pkt_id_mask_ = 0;
  uint8_t pkt_id_mask_result_ = 0;
  uint32_t circle_id_of_pre_pkt_ = 0;
  int32_t pre_frame_id_ = -1;

  std::map<uint16, std::string> get_lidar_param_;
  std::string lidar_mac_ = "";
  std::string firmware_version_ = "";

  std::vector<uint8_t> buf_cache_;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderConstParam &getConstParam();

  std::vector<PointDXYZIRTT> point_cloud_value_;
  std::vector<uint16_t> distance_cache_;

  void initLdLuminousMoment(void);
  void setPointsValue(double timestamp, const uint8_t *points_buf, uint8_t pkt_type, uint8_t pkg_no, uint16_t points_num, uint8_t resolution_index);
  void zoneAlarmUpload(uint8_t info);
  void resetCirclePointCloud();
  float applyAngleAndHideFilter(float distance, int32_t angle);
  void publishBasicParamIfReady();

 public:
  constexpr static double FRAME_DURATION = 0.066666667;
  constexpr static uint32_t SINGLE_PKT_NUM = 6;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee718H() = default;
  explicit DecoderVanjee718H(const WJDecoderParam &param);
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);
};

template <typename T_PointCloud>
inline WJDecoderConstParam &DecoderVanjee718H<T_PointCloud>::getConstParam() {
  static WJDecoderConstParam param = {1, 0.001f};
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee718H<T_PointCloud>::DecoderVanjee718H(const WJDecoderParam &param) : Decoder<T_PointCloud>(getConstParam(), param) {
  this->point_cloud_detect_params_.enable = true;
  this->point_cloud_detect_params_.valid_point_num_standard = 1368;
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  if (this->param_.filter_alg_enable) {
    distance_cache_.reserve(POINTS_PER_CIRCLE);
  }
  point_cloud_value_.reserve(POINTS_PER_CIRCLE);

  initLdLuminousMoment();
}

template <typename T_PointCloud>
void DecoderVanjee718H<T_PointCloud>::initLdLuminousMoment() {
  all_points_luminous_moment_718h_.resize(3);
  all_points_luminous_moment_718h_[0].resize(POINTS_PER_CIRCLE);
  all_points_luminous_moment_718h_[1].resize(POINTS_PER_CIRCLE);
  all_points_luminous_moment_718h_[2].resize(POINTS_PER_CIRCLE);
  for (size_t col = 0; col < POINTS_PER_CIRCLE; col++) {
    all_points_luminous_moment_718h_[0][col] = col * luminous_period_of_ld_10hz_;
    all_points_luminous_moment_718h_[1][col] = col * luminous_period_of_ld_15hz_;
    all_points_luminous_moment_718h_[2][col] = col * luminous_period_of_ld_14hz_;
  }
}

template <typename T_PointCloud>
inline void DecoderVanjee718H<T_PointCloud>::resetCirclePointCloud() {
  point_cloud_value_.clear();
  pkt_id_mask_ = 0;
  if (this->param_.filter_alg_enable) {
    distance_cache_.clear();
  }
}

template <typename T_PointCloud>
inline float DecoderVanjee718H<T_PointCloud>::applyAngleAndHideFilter(float distance, int32_t angle) {
  if (this->start_angle_ < this->end_angle_) {
    if (angle < this->start_angle_ || angle > this->end_angle_) {
      return 0.0f;
    }
  } else {
    if (angle > this->end_angle_ && angle < this->start_angle_) {
      return 0.0f;
    }
  }

  if (!this->hide_range_params_.empty() && distance != 0 &&
      this->isValueInRange(this->first_line_id_, angle / 1000.0, distance, this->hide_range_params_)) {
    return 0.0f;
  }
  return distance;
}

template <typename T_PointCloud>
inline bool DecoderVanjee718H<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  const uint8_t *data_ptr = pkt;
  size_t data_size = size;
  std::vector<uint8_t> data;

  if (!buf_cache_.empty()) {
    data.reserve(buf_cache_.size() + size);
    data.insert(data.end(), buf_cache_.begin(), buf_cache_.end());
    data.insert(data.end(), pkt, pkt + size);
    buf_cache_.clear();
    data_ptr = data.data();
    data_size = data.size();
  }

  uint32 index_last = 0;
  for (size_t i = 0; i < data_size; i++) {
    if (data_size - i < 4) {
      index_last = i;
      break;
    }

    if (!(data_ptr[i] == 0xff && data_ptr[i + 1] == 0xaa)) {
      index_last = i + 1;
      continue;
    }

    uint16_t frame_len = ((data_ptr[i + 2] << 8) | data_ptr[i + 3]) + 4;

    if (i + frame_len > data_size) {
      if (frame_len > 1500) {
        index_last = i + 1;
        continue;
      } else {
        index_last = i;
        break;
      }
    }

    if (!(data_ptr[i + frame_len - 2] == 0xee && data_ptr[i + frame_len - 1] == 0xee)) {
      index_last = i + 1;
      continue;
    }
    if (decodeMsopPkt_1(&data_ptr[i], frame_len)) {
      ret = true;
      index_last = i + frame_len;
      break;
    }

    i += frame_len - 1;
    index_last = i + 1;
  }

  if (index_last < data_size) {
    buf_cache_.assign(data_ptr + index_last, data_ptr + data_size);
  }

  return ret;
}

template <typename T_PointCloud>
inline void DecoderVanjee718H<T_PointCloud>::zoneAlarmUpload(uint8_t info) {
  if (!this->param_.send_lidar_param_enable) {
    return;
  }

  std::vector<uint16_t> vec_id;
  vec_id.reserve(3);
  for (uint16_t i = 0; i < 3; i++) {
    if (info & (0x01 << i)) {
      vec_id.push_back(i + 1);
    }
  }
  if (vec_id.empty()) {
    return;
  }

  std::string zone_alarm_str = R"(
  {
    io: [)";
  for (size_t i = 0; i < vec_id.size(); i++) {
    zone_alarm_str += std::to_string(vec_id[i]);
    if (i + 1 < vec_id.size()) {
      zone_alarm_str += ", ";
    }
  }
  zone_alarm_str += R"(]
  })";

  LidarParameterInterface lidar_param;
  lidar_param.cmd_id = (uint16_t)LidarParam::zone_alarm;
  lidar_param.cmd_type = 0;
  lidar_param.repeat_interval = 0;
  get_lidar_param_[(uint16_t)LidarParam::zone_alarm] = zone_alarm_str;
  this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
  this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
}

template <typename T_PointCloud>
inline void DecoderVanjee718H<T_PointCloud>::publishBasicParamIfReady() {
  if (firmware_version_.empty() || lidar_mac_.empty() || !this->param_.send_lidar_param_enable) {
    return;
  }

  auto it = this->getLidarParam((uint16_t)LidarParam::basic_param);
  if (it == this->lidar_param_vec_.end()) {
    return;
  }

  std::string basic_param_str = R"(
  {
    "lidar_type": "WLR-718H",
    "mac": ")" + lidar_mac_ + R"(",
    "firmware_version": ")" + firmware_version_ +
                                R"("
  })";

  get_lidar_param_[(uint16_t)LidarParam::basic_param] = basic_param_str;

  LidarParameterInterface lidar_param;
  lidar_param.cmd_id = (uint16_t)LidarParam::basic_param;
  lidar_param.cmd_type = 0;
  lidar_param.repeat_interval = 0;
  this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
  this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);
  firmware_version_.clear();
  lidar_mac_.clear();
  this->delLidarParam(it);
}

template <typename T_PointCloud>
inline void DecoderVanjee718H<T_PointCloud>::setPointsValue(double timestamp, const uint8_t *points_buf, uint8_t pkt_type, uint8_t pkg_no,
                                                            uint16_t points_num, uint8_t resolution_index) {
  if (point_cloud_value_.empty()) {
    point_cloud_value_.resize(POINTS_PER_CIRCLE);
    pkt_ts_ = timestamp;
  }

  const auto &luminous_moments = all_points_luminous_moment_718h_[resolution_index];
  const double last_moment = luminous_moments.back();
  const bool ts_first_point = this->param_.ts_first_point;
  const bool filter_alg_enable = this->param_.filter_alg_enable;

  if (pkt_type == 0) {
    const uint32_t point_id_base = (pkg_no - 1) * 500;
    for (size_t i = 0; i < points_num; i++) {
      const uint32_t point_id = point_id_base + i;
      const double timestamp_point = ts_first_point ? luminous_moments[point_id] : (luminous_moments[point_id] - last_moment);

      const int32_t angle = (int32_t)((180 + point_id * ANGLE_RESOLUTION) * 1000) % 360000;
      float distance = ((points_buf[2 * i] << 8) + points_buf[2 * i + 1]) * this->const_param_.distance_res;
      if (filter_alg_enable) {
        distance_cache_.emplace_back((uint16_t)(distance * 1000));
      } else {
        if (distance > 0.007f) {
          this->point_cloud_detect_params_.valid_point_num++;
        }
        distance = applyAngleAndHideFilter(distance, angle);
        point_cloud_value_[point_id].the_1st_echo_distance = distance;
        point_cloud_value_[point_id].the_1st_echo_x = distance * COS(angle);
        point_cloud_value_[point_id].the_1st_echo_y = distance * SIN(angle);
      }
      point_cloud_value_[point_id].the_1st_echo_z = 0.0;
      point_cloud_value_[point_id].timestamp = timestamp_point;
      point_cloud_value_[point_id].ring = this->first_line_id_;
      point_cloud_value_[point_id].tag = 0;
    }
  } else if (pkt_type == 1) {
    const uint32_t point_id_base = (pkg_no - 4) * 500;
    for (size_t i = 0; i < points_num; i++) {
      point_cloud_value_[point_id_base + i].intensity = points_buf[2 * i + 1];
    }
  }

  this->prev_pkt_ts_ = timestamp;
}

template <typename T_PointCloud>
inline bool DecoderVanjee718H<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size) {
  Vanjee718hHeaderPkt &header = (*(Vanjee718hHeaderPkt *)packet);
  header.ToLittleEndian();

  if (header.main_cmd != 0x02 || header.sub_cmd != 0x02) {
    return false;
  }

  Vanjee718hMsopPkt &pkt = (*(Vanjee718hMsopPkt *)packet);
  pkt.ToLittleEndian();

  bool ret = false;
  if (pkt_id_mask_result_ == 0) {
    for (int i = 0; i < pkt.total_pkts_num; i++) {
      pkt_id_mask_result_ |= (0x01 << i);
    }
  }

  int32_t loss_packets_num = (pkt.header.frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = pkt.header.frame_id;

  int32_t resolution_index = 0;
  uint8_t frequency = 15;
  if (pkt.resolution == 0) {
    frequency = 10;
    resolution_index = 0;
  } else if (pkt.resolution == 1) {
    frequency = 15;
    resolution_index = 1;
  } else if (pkt.resolution == 2) {
    frequency = 14;
    resolution_index = 2;
  } else {
    return ret;
  }

  double pkt_ts = 0.0;
  this->point_cloud_detect_params_.point_cloud_pkt_host_ts = getTimeHost() * 1e-6;
  if (!this->param_.use_lidar_clock) {
    pkt_ts = this->point_cloud_detect_params_.point_cloud_pkt_host_ts;
  } else {
    if (pkt.header.protocol_version < 2000 && (pkt.header.protocol_version & 0x0100) == 0x0100) {
      constexpr double gapTime1900_1970 = (25567LL * 24 * 3600);
      uint32_t sec = packet[size - 17] + (packet[size - 18] << 8) + (packet[size - 19] << 16) + (packet[size - 20] << 24);
      uint32_t usec = packet[size - 13] + (packet[size - 14] << 8) + (packet[size - 15] << 16) + (packet[size - 16] << 24);
      pkt_ts = (double)sec + (double)usec / TIME_FRAC_SCALE - gapTime1900_1970;
      pkt_ts = pkt_ts < 0 ? 0 : pkt_ts;
    } else {
      pkt_ts = 0;
    }
  }

  if ((this->param_.device_ctrl_state_enable || this->param_.send_lidar_param_enable) && pkt.header.protocol_version < 2000 &&
      ((pkt.header.protocol_version & 0x0100) == 0x0100 || (pkt.header.protocol_version & 0x0200) == 0x0200)) {
    uint16_t device_status = packet[size - 9] | (packet[size - 10] << 8);
    if (device_status == 0x0001 || device_status == 0x0002) {
      this->deviceStatePublish(0, device_status, device_status, pkt_ts);
    }
  }

  if (pkt.circle_id != circle_id_of_pre_pkt_) {
    if (pkt.circle_id > circle_id_of_pre_pkt_ || circle_id_of_pre_pkt_ - pkt.circle_id >= 10) {
      resetCirclePointCloud();
    } else {
      return ret;
    }
  }

  setPointsValue(pkt_ts, packet + POINTS_DATA_OFFSET, pkt.pkt_type, pkt.pkt_id, pkt.points_num, resolution_index);
  pkt_id_mask_ |= (0x01 << (pkt.pkt_id - 1));

  if ((pkt_id_mask_ & pkt_id_mask_result_) == pkt_id_mask_result_) {
    zoneAlarmUpload(pkt.zone_output_value);

    if (this->point_cloud_detect_params_.enable) {
      this->pointCloudDetectParamsUpdate();
    }

    if (this->param_.filter_alg_enable) {
      if (distance_cache_.size() == POINTS_PER_CIRCLE) {
        lidar_718::GSFilter(distance_cache_.data());
        for (size_t idx = 0; idx < POINTS_PER_CIRCLE; idx++) {
          float distance = distance_cache_[idx] * 1e-3f;
          const int32_t angle = (int32_t)((180 + idx * ANGLE_RESOLUTION) * 1000) % 360000;
          if (distance > 0.007f) {
            this->point_cloud_detect_params_.valid_point_num++;
          }
          distance = applyAngleAndHideFilter(distance, angle);
          point_cloud_value_[idx].the_1st_echo_distance = distance;
          point_cloud_value_[idx].the_1st_echo_x = distance * COS(angle);
          point_cloud_value_[idx].the_1st_echo_y = distance * SIN(angle);
        }
      } else {
        distance_cache_.clear();
        circle_id_of_pre_pkt_ = pkt.circle_id;
        return false;
      }
    }

    const auto &luminous_moments = all_points_luminous_moment_718h_[resolution_index];
    this->first_point_ts_ = pkt_ts_ - luminous_moments.back();
    this->last_point_ts_ = pkt_ts_;

    if (this->param_.point_cloud_enable) {
      this->point_cloud_->points.reserve(this->point_cloud_->points.size() + point_cloud_value_.size());
      for (size_t i = 0; i < point_cloud_value_.size(); i++) {
        typename T_PointCloud::PointT point;
        if (this->distance_section_.in(point_cloud_value_[i].the_1st_echo_distance) && point_cloud_value_[i].the_1st_echo_distance >= 0.01) {
          float x = point_cloud_value_[i].the_1st_echo_x;
          float y = point_cloud_value_[i].the_1st_echo_y;
          float z = point_cloud_value_[i].the_1st_echo_z;

          this->transformPoint(x, y, z);
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, point_cloud_value_[i].intensity);
        } else {
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            setX(point, 0);
            setY(point, 0);
            setZ(point, 0);
          }
          setIntensity(point, 0.0);
        }
        setTimestamp(point, point_cloud_value_[i].timestamp);
        setRing(point, point_cloud_value_[i].ring);
        setTag(point, point_cloud_value_[i].tag);
#ifdef ENABLE_GTEST
        setPointId(point, i);
        setHorAngle(point, ((int32_t)((180 + i * ANGLE_RESOLUTION) * 1000) % 360000) / 1000.0);
        setVerAngle(point, 0.0);
        setDistance(point, point_cloud_value_[i].the_1st_echo_distance);
#endif
        this->point_cloud_->points.emplace_back(point);
      }
      this->cb_split_frame_(this->const_param_.chan_num, this->cloudTs());
    }

    if (this->param_.laser_scan_enable) {
      this->scan_data_->ranges.reserve(point_cloud_value_.size());
      this->scan_data_->intensities.reserve(point_cloud_value_.size());
      for (size_t i = 0; i < point_cloud_value_.size(); i++) {
        if (this->distance_section_.in(point_cloud_value_[i].the_1st_echo_distance) && point_cloud_value_[i].the_1st_echo_distance >= 0.01) {
          this->scan_data_->ranges.emplace_back(point_cloud_value_[i].the_1st_echo_distance);
          this->scan_data_->intensities.emplace_back(point_cloud_value_[i].intensity);
        } else {
          if (!this->param_.dense_points) {
            this->scan_data_->ranges.emplace_back(NAN);
          } else {
            this->scan_data_->ranges.emplace_back(0);
          }
          this->scan_data_->intensities.emplace_back(0);
        }
      }
      this->scan_data_->angle_min = -180;
      this->scan_data_->angle_max = 180;
      this->scan_data_->angle_increment = ANGLE_RESOLUTION;
      this->scan_data_->time_increment = luminous_moments[1] - luminous_moments[0];
      this->scan_data_->scan_time = 1.0 / (float)frequency;
      this->scan_data_->range_min = this->param_.min_distance;
      this->scan_data_->range_max = this->param_.max_distance;

      this->cb_scan_data_(this->cloudTs());

      this->scan_data_->ranges.clear();
      this->scan_data_->intensities.clear();
    }

    if (this->param_.filter_alg_enable) {
      distance_cache_.clear();
    }
    ret = true;
  }

  circle_id_of_pre_pkt_ = pkt.circle_id;

  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee718H<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract718H> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_scan_data_get_)) {
    p = std::make_shared<Protocol_ScanDataGet718H>();
  } else if (*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_firmware_version_get_)) {
    p = std::make_shared<Protocol_FirmwareVersionGet718H>();
  } else if (*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_bank_switch_type_set_)) {
    p = std::make_shared<Protocol_BankSwitchTypeSet718H>();
  } else if (*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_bank_id_set_)) {
    p = std::make_shared<Protocol_BankIdSet718H>();
  } else if (*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_bank_id_get_)) {
    p = std::make_shared<Protocol_BankIdGet718H>();
  } else if (*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_network_param_get_)) {
    p = std::make_shared<Protocol_NetworkParamGet718H>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_ScanData718H)) {
    std::shared_ptr<Params_ScanData718H> param = std::dynamic_pointer_cast<Params_ScanData718H>(params);
    if (param->data_get_flag_ && !scan_data_recv_flag_) {
      WJ_INFOL << "get wlr718h scan data succ !" << WJ_REND;
      scan_data_recv_flag_ = true;
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else if (typeid(*params) == typeid(Params_FirmwareVersion718H)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_FirmwareVersion718H> param = std::dynamic_pointer_cast<Params_FirmwareVersion718H>(params);

    firmware_version_ = param->firmware_version_;
    get_lidar_param_[(uint16_t)LidarParam::firmware_version] = param->firmware_version_;
    WJ_INFOL << "Get lidar firmware version succ ( " << param->firmware_version_ << " )" << WJ_REND;

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

      publishBasicParamIfReady();
    }

  } else if (typeid(*params) == typeid(Params_BankSwitchTypeSet718H)) {
    std::shared_ptr<Params_BankSwitchTypeSet718H> param = std::dynamic_pointer_cast<Params_BankSwitchTypeSet718H>(params);
    if (param->flag_) {
      WJ_INFOL << "set wlr718h bank id switch type succ !" << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    }
  } else if (typeid(*params) == typeid(Params_BankIdSet718H)) {
    std::shared_ptr<Params_BankIdSet718H> param = std::dynamic_pointer_cast<Params_BankIdSet718H>(params);
    if (param->flag_) {
      WJ_INFOL << "set wlr718h bank id succ !" << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository718H::CreateInstance()->sp_bank_id_get_->GetCmdKey()].setStopFlag(false);
    }
  } else if (typeid(*params) == typeid(Params_BankIdGet718H)) {
    std::shared_ptr<Params_BankIdGet718H> param = std::dynamic_pointer_cast<Params_BankIdGet718H>(params);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);

    get_lidar_param_[(uint16_t)LidarParam::alarm_zone_switch] = std::to_string((uint16_t)param->bank_id_);

    if (this->param_.send_lidar_param_enable) {
      auto it = this->getLidarParam((uint16_t)LidarParam::alarm_zone_switch);
      if (it != this->lidar_param_vec_.end()) {
        LidarParameterInterface lidar_param;
        lidar_param.cmd_id = (uint16_t)LidarParam::alarm_zone_switch;
        lidar_param.cmd_type = 0;
        lidar_param.repeat_interval = 0;
        this->getLidarParameterDataFormat(lidar_param, get_lidar_param_);
        this->lidarParameterPublish(lidar_param, this->prev_pkt_ts_);

        this->delLidarParam(it);
      }
    }

    WJ_INFOL << "Get wlr718h bank id succ (" << (uint16_t)param->bank_id_ << ") !" << WJ_REND;

  } else if (typeid(*params) == typeid(Params_NetworkParam718H)) {
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    std::shared_ptr<Params_NetworkParam718H> param = std::dynamic_pointer_cast<Params_NetworkParam718H>(params);

    std::stringstream ss;
    ss << std::hex << std::setw(2) << std::setfill('0') << ((uint16_t)param->mac_[0] & 0xff) << "-" << ((uint16_t)param->mac_[1] & 0xff) << "-"
       << ((uint16_t)param->mac_[2] & 0xff) << "-" << ((uint16_t)param->mac_[3] & 0xff) << "-" << ((uint16_t)param->mac_[4] & 0xff) << "-"
       << ((uint16_t)param->mac_[5] & 0xff);
    lidar_mac_ = ss.str();
    WJ_INFOL << "Get lidar mac succ ( " << lidar_mac_ << " )" << WJ_REND;

    publishBasicParamIfReady();

  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar
}  // namespace vanjee