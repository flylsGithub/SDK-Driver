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
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class DifopVanjee722 : public DifopBase {
 public:
  virtual void initGetDifoCtrlDataMapPtr();
  virtual void addItem2GetDifoCtrlDataMapPtr(const DeviceCtrl& device_ctrl);
  virtual void addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_param);
};

void DifopVanjee722::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  // GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet722>()->GetRequest()));
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_ld_angle_get_->GetCmdKey(),getDifoCtrlData_LdAngleGet);

  // GetDifoCtrlClass getDifoCtrlData_LdOffsetGet(*(std::make_shared<Protocol_LDOffsetGet722>()->GetRequest()));
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_ld_offset_get_->GetCmdKey(),getDifoCtrlData_LdOffsetGet);

  GetDifoCtrlClass getDifoCtrlData_LdValueGet(*(std::make_shared<Protocol_LDValueGet722>()->GetRequest()));
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_ld_value_get_->GetCmdKey(), getDifoCtrlData_LdValueGet);

  std::shared_ptr<std::vector<uint8_t>> content1 = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0x00, 0x01, 0x02, 0x58});
  GetDifoCtrlClass getDifoCtrlData_LDEccentricityParamGet1(*(std::make_shared<Protocol_LDEccentricityParamGet722>()->GetRequest(content1)));
  (*getDifoCtrlData_map_ptr_).emplace(0x0001, getDifoCtrlData_LDEccentricityParamGet1);

  std::shared_ptr<std::vector<uint8_t>> content2 = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0x00, 0x02, 0x02, 0x58});
  GetDifoCtrlClass getDifoCtrlData_LDEccentricityParamGet2(*(std::make_shared<Protocol_LDEccentricityParamGet722>()->GetRequest(content2)));
  (*getDifoCtrlData_map_ptr_).emplace(0x0002, getDifoCtrlData_LDEccentricityParamGet2);

  GetDifoCtrlClass getDifoCtrlData_ImuLineGet(*(std::make_shared<Protocol_ImuLineGet722>()->GetRequest()));
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_imu_line_Param_get_->GetCmdKey(), getDifoCtrlData_ImuLineGet);

  GetDifoCtrlClass getDifoCtrlData_IMUAddGet(*(std::make_shared<Protocol_ImuAddGet722>()->GetRequest()));
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_imu_add_Param_get_->GetCmdKey(), getDifoCtrlData_IMUAddGet);

  GetDifoCtrlClass getDifoCtrlData_ImuTempGet(*(std::make_shared<Protocol_ImuTempGet722>()->GetRequest()), false, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_temperature_param_get_->GetCmdKey(), getDifoCtrlData_ImuTempGet);

  std::shared_ptr<std::vector<uint8_t>> content3 = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0x01, 0x00});
  GetDifoCtrlClass getDifoCtrlData_ProtocolVersionSet(*(std::make_shared<Protocol_ProtocolVersionSet722>()->GetRequest(content3)), true);
  (*getDifoCtrlData_map_ptr_)
      .emplace(CmdRepository722::CreateInstance()->set_protocol_version_cmd_id_ptr_->GetCmdKey(), getDifoCtrlData_ProtocolVersionSet);

  GetDifoCtrlClass getDifoCtrlData_WorkModeGet(*(std::make_shared<Protocol_WorkModeGet722>()->GetRequest()), true);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_get_work_mode_->GetCmdKey(), getDifoCtrlData_WorkModeGet);

  GetDifoCtrlClass getDifoCtrlData_FirmwareGet(*(std::make_shared<Protocol_FirmwareVersionGet722>()->GetRequest()), false, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_firmware_version_get_->GetCmdKey(), getDifoCtrlData_FirmwareGet);

  GetDifoCtrlClass getDifoCtrlData_TimeSyncGet(*(std::make_shared<Protocol_TimeSyncGet722>()->GetRequest()), true);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository722::CreateInstance()->sp_time_sync_get_->GetCmdKey(), getDifoCtrlData_TimeSyncGet);
}

void DifopVanjee722::addItem2GetDifoCtrlDataMapPtr(const DeviceCtrl& device_ctrl) {
  if (device_ctrl.cmd_id == 1) {
    if (device_ctrl.cmd_param == 0 || device_ctrl.cmd_param == 1 || device_ctrl.cmd_param == 2) {
      std::shared_ptr<Params_WorkModeSet722> params_WorkModeSet722 = std::shared_ptr<Params_WorkModeSet722>(new Params_WorkModeSet722());
      params_WorkModeSet722->work_mode_ = (uint8_t)device_ctrl.cmd_param;
      GetDifoCtrlClass getDifoCtrlData_WorkModeSet(*(std::make_shared<Protocol_WorkModeSet722>(params_WorkModeSet722)->SetRequest()), false);

      uint16_t cmd = CmdRepository722::CreateInstance()->sp_set_work_mode_->GetCmdKey();
      auto it = (*(getDifoCtrlData_map_ptr_)).find(cmd);
      if (it != (*getDifoCtrlData_map_ptr_).end()) {
        ((*getDifoCtrlData_map_ptr_))[cmd] = getDifoCtrlData_WorkModeSet;
      } else {
        ((*getDifoCtrlData_map_ptr_)).emplace(cmd, getDifoCtrlData_WorkModeSet);
      }
    }
  }
}

void DifopVanjee722::addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_param) {
  if (lidar_param.cmd_id == 1) {
    if (lidar_param.cmd_type == 1) {
      uint8_t work_mode = 0xff;
      JsonParser parser(lidar_param.data);
      auto json_value = parser.parse();
      if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
        if (obj->has("work_mode")) {
          auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("work_mode"));
          if (result == nullptr || result->value() > 2) {
            WJ_WARNING << "Invalid work mode parameter (0-2)." << WJ_REND;
            return;
          }
          work_mode = static_cast<uint8_t>(result->value());
        }
      }
      if (work_mode > 2) {
        WJ_WARNING << "Invalid work mode parameter (0-2)." << WJ_REND;
        return;
      }
      std::shared_ptr<Params_WorkModeSet722> params_WorkModeSet722 = std::shared_ptr<Params_WorkModeSet722>(new Params_WorkModeSet722());
      params_WorkModeSet722->work_mode_ = work_mode;
      GetDifoCtrlClass getDifoCtrlData_WorkModeSet(*(std::make_shared<Protocol_WorkModeSet722>(params_WorkModeSet722)->SetRequest()), false);

      uint16_t cmd = CmdRepository722::CreateInstance()->sp_set_work_mode_->GetCmdKey();
      auto it = (*(getDifoCtrlData_map_ptr_)).find(cmd);
      if (it != (*getDifoCtrlData_map_ptr_).end()) {
        ((*getDifoCtrlData_map_ptr_))[cmd] = getDifoCtrlData_WorkModeSet;
      } else {
        ((*getDifoCtrlData_map_ptr_)).emplace(cmd, getDifoCtrlData_WorkModeSet);
      }
    } else {
      uint16_t cmd = CmdRepository722::CreateInstance()->sp_get_work_mode_->GetCmdKey();
      if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
        ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
      }
    }
  } else if (lidar_param.cmd_id == (uint16_t)LidarParam::firmware_version || lidar_param.cmd_id == (uint16_t)LidarParam::sn ||
             lidar_param.cmd_id == (uint16_t)LidarParam::basic_param) {
    uint16_t cmd = CmdRepository722::CreateInstance()->sp_firmware_version_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else if (lidar_param.cmd_id == 0x000b) {
    if (lidar_param.cmd_type == 1) {
      uint8_t clock_source = 0xff;
      JsonParser parser(lidar_param.data);
      auto json_value = parser.parse();
      if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
        if (obj->has("clock_source")) {
          auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("clock_source"));
          if (result == nullptr || result->value() < 0 || result->value() > 3) {
            WJ_WARNING << "Invalid time sync parameter (clock_source: 0-3)." << WJ_REND;
            return;
          }
          clock_source = static_cast<uint8_t>(result->value());
        }
      }
      if (clock_source > 3) {
        WJ_WARNING << "Invalid time sync parameter (clock_source: 0-3)." << WJ_REND;
        return;
      }

      std::shared_ptr<Params_TimeSyncSet722> params = std::shared_ptr<Params_TimeSyncSet722>(new Params_TimeSyncSet722());
      params->clock_source_set_ = clock_source;

      if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
        if (obj->has("parse_flag")) {
          auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("parse_flag"));
          if (result == nullptr || result->value() < 0 || result->value() > 1) {
            params->parse_flag_ = 0;
          } else {
            params->parse_flag_ = static_cast<uint8_t>(result->value());
          }
        } else {
          params->parse_flag_ = 0;
        }
      }

      switch (clock_source) {
        case 0:
          params->value_[0] = 1;
          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("utc_ts")) {
              auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("utc_ts"));
              if (result == nullptr || result->value() < 0 || result->value() > 4294967295) {
                WJ_WARNING << "Invalid time sync parameter (utc_ts: 0-4294967295)." << WJ_REND;
                return;
              } else {
                auto data = static_cast<uint32_t>(result->value());
                params->value_[1] = (uint8_t)(data & 0xff);
                params->value_[2] = (uint8_t)((data >> 8) & 0xff);
                params->value_[3] = (uint8_t)((data >> 16) & 0xff);
                params->value_[4] = (uint8_t)((data >> 24) & 0xff);
              }
            }
          }

          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("time_mode")) {
              auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("time_mode"));
              if (result == nullptr || result->value() < 0 || result->value() > 2) {
                WJ_WARNING << "Invalid time sync parameter (time_mode: 0-2)." << WJ_REND;
                return;
              } else {
                params->value_[5] = static_cast<uint8_t>(result->value());
              }
            }
          }
          for (uint32_t i = 0; i < 4; i++) {
            params->value_[6 + i] = 0;
          }
          break;

        case 1:
          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("gps_name_sentence")) {
              auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("gps_name_sentence"));
              if (result == nullptr || result->value() < 0 || result->value() > 1) {
                WJ_WARNING << "Invalid time sync parameter (gps_name_sentence: 0-1)." << WJ_REND;
                return;
              } else {
                params->value_[0] = static_cast<uint8_t>(result->value());
              }
            }
          }
          for (uint32_t i = 0; i < 9; i++) {
            params->value_[1 + i] = 0;
          }
          break;

        case 2:
          params->value_[0] = 0;
          params->value_[1] = 0;
          params->value_[2] = 0;

          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("ptp_announce_interval")) {
              auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("ptp_announce_interval"));
              if (result == nullptr || result->value() > 3 || result->value() < -2) {
                WJ_WARNING << "Invalid time sync parameter (ptp_announce_interval)." << WJ_REND;
                return;
              } else {
                params->value_[3] = static_cast<int8_t>(result->value());
              }
            }
          }
          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("ptp_sync_interval")) {
              auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("ptp_sync_interval"));
              if (result == nullptr || result->value() > 3 || result->value() < -7) {
                WJ_WARNING << "Invalid time sync parameter (ptp_sync_interval)." << WJ_REND;
                return;
              } else {
                params->value_[4] = static_cast<uint8_t>(result->value());
              }
            }
          }
          for (uint32_t i = 0; i < 5; i++) {
            params->value_[5 + i] = 0;
          }
          break;

        case 3:
          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("ntp_ip")) {
              auto result = std::dynamic_pointer_cast<JsonArray>(obj->get("ntp_ip"));
              if (result == nullptr || result->size() != 4) {
                WJ_WARNING << "Invalid time sync parameter (ntp_ip)." << WJ_REND;
                return;
              } else {
                params->value_[0] = static_cast<uint8_t>(std::dynamic_pointer_cast<JsonNumber>(result->get(0))->value());
                params->value_[1] = static_cast<uint8_t>(std::dynamic_pointer_cast<JsonNumber>(result->get(1))->value());
                params->value_[2] = static_cast<uint8_t>(std::dynamic_pointer_cast<JsonNumber>(result->get(2))->value());
                params->value_[3] = static_cast<uint8_t>(std::dynamic_pointer_cast<JsonNumber>(result->get(3))->value());
              }
            }
          }
          if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
            if (obj->has("ntp_sync_interval")) {
              auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("ntp_sync_interval"));
              if (result == nullptr || result->value() < 1 || result->value() > 255) {
                WJ_WARNING << "Invalid time sync parameter (ntp_sync_interval: 1-255)." << WJ_REND;
                return;
              } else {
                params->value_[4] = static_cast<uint8_t>(result->value());
              }
            }
          }
          for (uint32_t i = 0; i < 5; i++) {
            params->value_[5 + i] = 0;
          }
          break;

        default:
          break;
      }

      GetDifoCtrlClass getDifoCtrlData_TimeSyncSet(*(std::make_shared<Protocol_TimeSyncSet722>(params)->SetRequest()), false);

      uint16_t cmd = CmdRepository722::CreateInstance()->sp_time_sync_set_->GetCmdKey();
      auto it = (*(getDifoCtrlData_map_ptr_)).find(cmd);
      if (it != (*getDifoCtrlData_map_ptr_).end()) {
        ((*getDifoCtrlData_map_ptr_))[cmd] = getDifoCtrlData_TimeSyncSet;
      } else {
        ((*getDifoCtrlData_map_ptr_)).emplace(cmd, getDifoCtrlData_TimeSyncSet);
      }
    } else {
      uint16_t cmd = CmdRepository722::CreateInstance()->sp_time_sync_get_->GetCmdKey();
      if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
        ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
      }
    }
  } else {
    ;
  }
}
}  // namespace lidar
}  // namespace vanjee
