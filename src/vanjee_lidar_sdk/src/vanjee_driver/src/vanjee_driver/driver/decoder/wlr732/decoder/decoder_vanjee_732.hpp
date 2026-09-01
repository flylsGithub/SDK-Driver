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

#include <vanjee_driver/driver/decoder/wlr732/protocol/frames/cmd_repository_732.hpp>
#include <vanjee_driver/driver/decoder/wlr732/protocol/frames/protocol_ldangle_get_732.hpp>
#include <vanjee_driver/driver/decoder/wlr732/protocol/frames/protocol_motor_params_get_732.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee732Channel {
  uint16_t distance;
  uint8_t pulse_width;
  uint8_t confidence;

} Vanjee732Channel;

typedef struct _Vanjee732Block {
  uint16_t header;
  uint16_t azimuth;
  Vanjee732Channel channel[32];
} Vanjee732Block;

typedef struct _Vanjee732DifopPkt {
  uint8_t gps_state;
  uint8_t time_syn_state;
  uint8_t time_syn_source;
  uint8_t time_syn_error;
  uint32_t second;
  uint32_t microsecond;
  uint16_t frame_id;
  uint8_t remain[2];
} Vanjee732DifopPkt;

typedef struct _Vanjee732MsopPkt {
  Vanjee732Block blocks[10];
  uint8_t remain1[2];
  uint16_t circle_id;
  uint8_t remain2[8];
  Vanjee732DifopPkt difop;
} Vanjee732MsopPkt;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee732 : public Decoder<T_PointCloud> {
 private:
  std::vector<std::vector<double>> all_points_luminous_moment_732_;  // Cache 32 channels, one circle point cloud time difference
  const double luminous_period_of_ld_ = 0.00005555;                  // Time interval at adjacent horizontal angles

  int32_t azimuth_trans_pre_ = -1.0;
  int32_t pre_circle_id_ = -1;
  uint8_t publish_mode_ = 0;

  uint32_t laser_degree_per_sec_ = 0;
  std::vector<double> channel_light_moment_732_ =
      std::vector<double>{1.23,  3.13,  5.03,  6.93,  8.83,  10.73, 12.63, 14.53, 16.43, 18.23, 20.03, 21.83, 23.63, 25.43, 27.23, 29.03,
                          30.83, 32.53, 34.23, 35.93, 37.63, 39.33, 41.03, 42.73, 44.43, 45.43, 46.43, 47.43, 48.43, 49.43, 50.43, 51.43};

  bool angle_get_flag_ = false;
  bool motor_params_get_flag_ = false;
  RotateDirection rotate_direction_ = RotateDirection::anticlockwise;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderConstParam &getConstParam();
  ChanAngles chan_angles_;

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 450;
  void initLdLuminousMoment(void);

  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee732() = default;
  explicit DecoderVanjee732(const WJDecoderParam &param);
};

template <typename T_PointCloud>
void DecoderVanjee732<T_PointCloud>::initLdLuminousMoment() {
  double glowing_moments[32] = {0.00000123, 0.00000313, 0.00000503, 0.00000693, 0.00000883, 0.00001073, 0.00001263, 0.00001453,
                                0.00001643, 0.00001823, 0.00002003, 0.00002183, 0.00002363, 0.00002543, 0.00002723, 0.00002903,
                                0.00003083, 0.00003253, 0.00003423, 0.00003593, 0.00003763, 0.00003933, 0.00004103, 0.00004273,
                                0.00004443, 0.00004543, 0.00004643, 0.00004743, 0.00004843, 0.00004943, 0.00005043, 0.00005143};
  all_points_luminous_moment_732_.resize(4);
  all_points_luminous_moment_732_[0].resize(115200);
  all_points_luminous_moment_732_[1].resize(57600);
  all_points_luminous_moment_732_[2].resize(38400);
  all_points_luminous_moment_732_[3].resize(28800);
  for (uint16_t col = 0; col < 3600; col++) {
    for (uint8_t row = 0; row < this->const_param_.chan_num; row++) {
      if (col < 900) {
        for (int i = 0; i < 4; i++)
          all_points_luminous_moment_732_[i][col * this->const_param_.chan_num + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      } else if (col >= 900 && col < 1200) {
        for (int i = 0; i < 3; i++)
          all_points_luminous_moment_732_[i][col * this->const_param_.chan_num + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      } else if (col >= 1200 && col < 1800) {
        for (int i = 0; i < 2; i++)
          all_points_luminous_moment_732_[i][col * this->const_param_.chan_num + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      } else {
        all_points_luminous_moment_732_[0][col * this->const_param_.chan_num + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      }
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderConstParam &DecoderVanjee732<T_PointCloud>::getConstParam() {
  static WJDecoderConstParam param = {32, 0.004f};
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee732<T_PointCloud>::DecoderVanjee732(const WJDecoderParam &param)
    : Decoder<T_PointCloud>(getConstParam(), param), chan_angles_(this->const_param_.chan_num) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;
  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);

  this->start_angle_ = this->param_.start_angle * 1000;
  this->end_angle_ = this->param_.end_angle * 1000;

  if (this->param_.config_from_file) {
    chan_angles_.loadFromFile(this->param_.angle_path_ver);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee732<T_PointCloud>::decodeMsopPkt(const uint8_t *packet, size_t size) {
  if (!this->param_.point_cloud_enable || size != sizeof(Vanjee732MsopPkt))
    return false;
  const Vanjee732MsopPkt &pkt = *(Vanjee732MsopPkt *)packet;
  bool ret = false;
  double pkt_ts = 0;

  int32_t loss_circles_num = (pkt.circle_id + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1 && pre_circle_id_ >= 0) {
    WJ_WARNING << "loss " << (loss_circles_num - 1) << " circle" << WJ_REND;
    this->point_cloud_->points.clear();
  }
  pre_circle_id_ = pkt.circle_id;

  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    pkt_ts = pkt.difop.second + pkt.difop.microsecond * 1e-6;
    pkt_ts = pkt_ts < 0 ? 0 : pkt_ts;
  }

  uint8_t resolution_index = 0;
  int32_t resolution = ((pkt.blocks[1].azimuth - pkt.blocks[0].azimuth + 36000) % 36000) * 10;

  if (resolution == 100) {
    resolution_index = 0;
    laser_degree_per_sec_ = 1800;
  } else if (resolution == 200) {
    resolution_index = 1;
    laser_degree_per_sec_ = 3600;
  } else if (resolution == 300) {
    resolution_index = 2;
    laser_degree_per_sec_ = 5400;
  } else if (resolution == 400) {
    resolution_index = 3;
    laser_degree_per_sec_ = 7200;
  } else {
    return ret;
  }

  for (uint16_t blk = 0; blk < 10; blk++) {
    const Vanjee732Block &block = pkt.blocks[blk];
    int32_t azimuth = block.azimuth * 10;
    int32_t azimuth_trans = block.azimuth * 10;

    if ((this->split_strategy_->newBlock(azimuth_trans) || (loss_circles_num == 1 && blk == 0 && azimuth_trans != resolution)) &&
        this->point_cloud_->points.size() != 0) {
      int32_t point_gap_num = (azimuth_trans / resolution - 1) * this->const_param_.chan_num;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_732_[resolution_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_732_[resolution_index][all_points_luminous_moment_732_[resolution_index].size() - 1];

      this->cb_split_frame_(this->const_param_.chan_num, this->cloudTs());
      ret = true;
    }

    double timestamp_point;
    uint32_t cur_blk_first_point_id = (azimuth / resolution - 1) * this->const_param_.chan_num;
    for (uint16_t chan = 0; chan < 32; chan++) {
      float xy, x, y, z;
      const Vanjee732Channel &channel = block.channel[chan];

      float distance = channel.distance * this->const_param_.distance_res;
      int32_t angle_vert = (chan_angles_.vertAdjust(chan) + 360000) % 360000;

      int32_t angle_horiz = chan_angles_.horizAdjust(chan, azimuth, rotate_direction_);
      angle_horiz += laser_degree_per_sec_ * (channel_light_moment_732_[chan] - channel_light_moment_732_[0]) * 1e-3;
      angle_horiz = (angle_horiz + 180000 + 360000) % 360000;

      if (rotate_direction_ == RotateDirection::clockwise) {
        angle_horiz = 360000 - angle_horiz;
      }

      uint32_t point_id = cur_blk_first_point_id + chan;
      if (this->param_.ts_first_point == true) {
        timestamp_point = all_points_luminous_moment_732_[resolution_index][point_id];
      } else {
        timestamp_point = all_points_luminous_moment_732_[resolution_index][point_id] -
                          all_points_luminous_moment_732_[resolution_index][all_points_luminous_moment_732_[resolution_index].size() - 1];
      }

      int32_t angle_horiz_mask = angle_horiz;
      if (this->start_angle_ < this->end_angle_) {
        if (angle_horiz_mask < (this->start_angle_) || angle_horiz_mask > (this->end_angle_)) {
          distance = 0;
        }
      } else {
        if (angle_horiz_mask < (this->start_angle_) && angle_horiz_mask > (this->end_angle_)) {
          distance = 0;
        }
      }

      if (this->hide_range_params_.size() > 0 && distance != 0 &&
          this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
        distance = 0;
      }

      if (this->distance_section_.in(distance)) {
        xy = distance * COS(angle_vert);
        x = xy * COS(angle_horiz);
        y = xy * SIN(angle_horiz);
        z = distance * SIN(angle_vert);
        this->transformPoint(x, y, z);

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.pulse_width);
        setTimestamp(point, timestamp_point);
        setRing(point, chan + this->first_line_id_);
        setTag(point, 0);
#ifdef ENABLE_GTEST
        setPointId(point, point_id);
        setHorAngle(point, angle_horiz / 1000.0);
        setVerAngle(point, angle_vert / 1000.0);
        setDistance(point, distance);
#endif
        this->point_cloud_->points.emplace_back(point);
      } else {
        typename T_PointCloud::PointT point;
        if (!this->param_.dense_points) {
          setX(point, NAN);
          setY(point, NAN);
          setZ(point, NAN);
        } else {
          setX(point, 0);
          setY(point, 0);
          setZ(point, 0);
        }

        setIntensity(point, 0);
        setTimestamp(point, timestamp_point);
        setRing(point, chan + this->first_line_id_);
        setTag(point, 0);
#ifdef ENABLE_GTEST
        setPointId(point, point_id);
        setHorAngle(point, angle_horiz / 1000.0);
        setVerAngle(point, angle_vert / 1000.0);
        setDistance(point, distance);
#endif
        this->point_cloud_->points.emplace_back(point);
      }
    }
    if (azimuth_trans == 360000) {
      this->last_point_ts_ = pkt_ts;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_732_[resolution_index][all_points_luminous_moment_732_[resolution_index].size() - 1];
      this->cb_split_frame_(this->const_param_.chan_num, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee732<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);
  std::shared_ptr<ProtocolAbstract> p;

  if (*sp_cmd == *(CmdRepository732::CreateInstance()->sp_ld_angle_get_)) {
    p = std::make_shared<Protocol_LDAngleGet732>();
  } else if (*sp_cmd == *(CmdRepository732::CreateInstance()->sp_motor_params_get_)) {
    p = std::make_shared<Protocol_MotorParamsGet732>();
  } else {
    return;
  }

  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;

  if (typeid(*params) == typeid(Params_LDAngle732)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_LDAngle732> param = std::dynamic_pointer_cast<Params_LDAngle732>(params);

    std::vector<double> vert_angles;
    std::vector<double> horiz_angles;

    for (int chan = 0; chan < param->chan_num_; chan++) {
      vert_angles.push_back((double)(param->ver_angle_[chan] / 1000.0));
      horiz_angles.push_back((double)(param->hor_angle_[chan] / 1000.0));
    }

    chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->chan_num_, vert_angles, horiz_angles);

    if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
      WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      angle_get_flag_ = true;
      if (angle_get_flag_ && motor_params_get_flag_) {
        Decoder<T_PointCloud>::angles_ready_ = true;
      }
    }
  } else if (typeid(*params) == typeid(Params_MotorParams732)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_MotorParams732> param = std::dynamic_pointer_cast<Params_MotorParams732>(params);
    if (param->rotate_direction_ == 0) {
      rotate_direction_ = RotateDirection::anticlockwise;
    } else {
      rotate_direction_ = RotateDirection::clockwise;
    }
    if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
      WJ_INFOL << "Get motor rotate direction..." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      motor_params_get_flag_ = true;
      if (angle_get_flag_ && motor_params_get_flag_) {
        Decoder<T_PointCloud>::angles_ready_ = true;
      }
    }
  } else {
  }
}
}  // namespace lidar

}  // namespace vanjee
