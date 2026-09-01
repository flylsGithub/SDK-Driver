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

#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/cmd_repository_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_bank_id_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_bank_id_set.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_bank_switch_type_set.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_topboard_operate_params_get.hpp>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class DifopVanjee718H : public DifopBase {
 public:
  virtual void initGetDifoCtrlDataMapPtr();
  virtual void addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_param);

 private:
  bool set_bank_switch_type_flag_ = false;
};

void DifopVanjee718H::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  const uint8 arr[] = {0x01, 0x00, 0x00, 0x00};
  std::shared_ptr<std::vector<uint8>> content = std::make_shared<std::vector<uint8>>();
  content->insert(content->end(), arr, arr + sizeof(arr) / sizeof(uint8));

  GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet718H>()->GetRequest(content)), false, 3000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_scan_data_get_->GetCmdKey(), getDifoCtrlData_ScanDataGet);

  GetDifoCtrlClass getDifoCtrlData_FirmwareGet(*(std::make_shared<Protocol_FirmwareVersionGet718H>()->GetRequest()), false, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_firmware_version_get_->GetCmdKey(), getDifoCtrlData_FirmwareGet);

  const uint8 arr1[] = {0x01, 0x00, 0x00, 0x00};
  std::shared_ptr<std::vector<uint8>> content1 = std::make_shared<std::vector<uint8>>();
  content1->insert(content1->end(), arr1, arr1 + sizeof(arr1) / sizeof(uint8));
  GetDifoCtrlClass getDifoCtrlData_BankSwitchTypeSet(*(std::make_shared<Protocol_BankSwitchTypeSet718H>()->GetRequest(content1)), true, 100);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_bank_switch_type_set_->GetCmdKey(), getDifoCtrlData_BankSwitchTypeSet);

  GetDifoCtrlClass getDifoCtrlData_BankIdGet(*(std::make_shared<Protocol_BankIdGet718H>()->GetRequest()), true, 100);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_bank_id_get_->GetCmdKey(), getDifoCtrlData_BankIdGet);

  GetDifoCtrlClass getDifoCtrlData_BankIdSet(*(std::make_shared<Protocol_BankIdSet718H>()->GetRequest()), true, 100);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_bank_id_set_->GetCmdKey(), getDifoCtrlData_BankIdSet);

  GetDifoCtrlClass getDifoCtrlData_NetworkParamGet(*(std::make_shared<Protocol_NetworkParamGet718H>()->GetRequest()), false, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_network_param_get_->GetCmdKey(), getDifoCtrlData_NetworkParamGet);

  // GetDifoCtrlClass getDifoCtrlData_TopBoardOperateGet(*(std::make_shared<Protocol_TopBoardOperateParamsGet718H>()->GetRequest()), false, 3000);
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_top_board_operate_params_get_->GetCmdKey(),getDifoCtrlData_TopBoardOperateGet);
}

void DifopVanjee718H::addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_param) {
  if (lidar_param.cmd_id == (uint16_t)LidarParam::firmware_version) {
    uint16_t cmd = CmdRepository718H::CreateInstance()->sp_firmware_version_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else if (lidar_param.cmd_id == (uint16_t)LidarParam::alarm_zone_switch) {
    if (lidar_param.cmd_type == 1) {
      if (!set_bank_switch_type_flag_) {
        (*getDifoCtrlData_map_ptr_)[CmdRepository718H::CreateInstance()->sp_bank_switch_type_set_->GetCmdKey()].setStopFlag(false);
        set_bank_switch_type_flag_ = true;
      }

      uint8_t bank_id = 0xff;
      JsonParser parser(lidar_param.data);
      auto json_value = parser.parse();
      if (auto obj = std::dynamic_pointer_cast<JsonObject>(json_value)) {
        if (obj->has("bank_id")) {
          auto result = std::dynamic_pointer_cast<JsonNumber>(obj->get("bank_id"));
          if (result == nullptr || result->value() < 1 || result->value() > 16) {
            WJ_WARNING << "Invalid bank id parameter (1-16)." << WJ_REND;
            return;
          }
          bank_id = static_cast<uint8_t>(result->value());
        }
      }
      if (bank_id < 1 || bank_id > 16) {
        WJ_WARNING << "Invalid bank id parameter (1-16)." << WJ_REND;
        return;
      }
      std::shared_ptr<Params_BankIdSet718H> param = std::shared_ptr<Params_BankIdSet718H>(new Params_BankIdSet718H());
      param->bank_id_ = bank_id;
      GetDifoCtrlClass getDifoCtrlData_BankIdSet(*(std::make_shared<Protocol_BankIdSet718H>(param)->SetRequest()), false, 100);

      uint16_t cmd = CmdRepository718H::CreateInstance()->sp_bank_id_set_->GetCmdKey();
      auto it = (*(getDifoCtrlData_map_ptr_)).find(cmd);
      if (it != (*getDifoCtrlData_map_ptr_).end()) {
        ((*getDifoCtrlData_map_ptr_))[cmd] = getDifoCtrlData_BankIdSet;
      } else {
        ((*getDifoCtrlData_map_ptr_)).emplace(cmd, getDifoCtrlData_BankIdSet);
      }
    } else {
      uint16_t cmd = CmdRepository718H::CreateInstance()->sp_bank_id_get_->GetCmdKey();
      if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
        ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
      }
    }
  } else if (lidar_param.cmd_id == (uint16_t)LidarParam::basic_param) {
    uint16_t cmd_firmware_version = CmdRepository718H::CreateInstance()->sp_firmware_version_get_->GetCmdKey();
    uint16_t cmd_network_param = CmdRepository718H::CreateInstance()->sp_network_param_get_->GetCmdKey();

    if (((*getDifoCtrlData_map_ptr_))[cmd_firmware_version].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd_firmware_version].setStopFlag(false);
    }

    if (((*getDifoCtrlData_map_ptr_))[cmd_network_param].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd_network_param].setStopFlag(false);
    }
  } else {
    ;
  }
}
}  // namespace lidar
}  // namespace vanjee