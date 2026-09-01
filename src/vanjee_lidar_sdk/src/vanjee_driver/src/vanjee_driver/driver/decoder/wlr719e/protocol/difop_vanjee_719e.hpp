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

#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/cmd_repository_719e.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_scan_data_get.hpp>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class DifopVanjee719E : public DifopBase {
 public:
  explicit DifopVanjee719E();
  virtual void initGetDifoCtrlDataMapPtr();
  virtual void addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_param);
};

inline DifopVanjee719E::DifopVanjee719E() {
  ProtocolBase pb;
  pb.SetByteOrder(ProtocolBase::DataEndiannessMode::little_endian);
  pb.SetProtocolVersion(ProtocolBase::ProtocolVersionDifop::version_safe);
  this->setOrgProtocolBase(pb);
}

void DifopVanjee719E::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16_t, GetDifoCtrlClass>>();

  GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet719E>()->GetRequest()), false, 3000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719E::CreateInstance()->sp_scan_data_get_->GetCmdKey(), getDifoCtrlData_ScanDataGet);

  std::shared_ptr<std::vector<uint8_t>> content = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0x01, 0x00});
  GetDifoCtrlClass getDifoCtrlData_ProtocolVersionSet(*(std::make_shared<Protocol_ProtocolVersionSet719E>()->GetRequest(content)), true);
  (*getDifoCtrlData_map_ptr_)
      .emplace(CmdRepository719E::CreateInstance()->set_protocol_version_cmd_id_ptr_->GetCmdKey(), getDifoCtrlData_ProtocolVersionSet);

  GetDifoCtrlClass getDifoCtrlData_FirmwareGet(*(std::make_shared<Protocol_FirmwareVersionGet719E>()->GetRequest()), true, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719E::CreateInstance()->sp_firmware_version_get_->GetCmdKey(), getDifoCtrlData_FirmwareGet);

  std::shared_ptr<std::vector<uint8_t>> content1 = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0xFF});
  GetDifoCtrlClass getDifoCtrlData_FieldDataGet(*(std::make_shared<Protocol_FieldDataGet719E>()->GetRequest(content1)), true, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719E::CreateInstance()->sp_field_data_get_->GetCmdKey(), getDifoCtrlData_FieldDataGet);

  std::shared_ptr<std::vector<uint8_t>> content2 = std::make_shared<std::vector<uint8_t>>(std::initializer_list<uint8_t>{0xFF});
  GetDifoCtrlClass getDifoCtrlData_MonitorCaseGet(*(std::make_shared<Protocol_MonitoringCaseGet719E>()->GetRequest(content1)), true, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719E::CreateInstance()->sp_monitoring_case_get_->GetCmdKey(), getDifoCtrlData_MonitorCaseGet);

  GetDifoCtrlClass getDifoCtrlData_VerifyResultsGet(*(std::make_shared<Protocol_VerifyResultsGet719E>()->GetRequest()), true, 1000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719E::CreateInstance()->sp_verify_results_get_->GetCmdKey(), getDifoCtrlData_VerifyResultsGet);
}

void DifopVanjee719E::addItem2GetDifoCtrlDataMapPtr(const LidarParameterInterface& lidar_param) {
  if (lidar_param.cmd_id == (uint16_t)LidarParam::firmware_version || lidar_param.cmd_id == (uint16_t)LidarParam::basic_param) {
    uint16_t cmd = CmdRepository719E::CreateInstance()->sp_firmware_version_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else if (lidar_param.cmd_id == (uint16_t)LidarParam::safety_field_data) {
    uint16_t cmd = CmdRepository719E::CreateInstance()->sp_field_data_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else if (lidar_param.cmd_id == (uint16_t)LidarParam::safety_monitoring_case) {
    // uint16_t cmd = CmdRepository719E::CreateInstance()->sp_monitoring_case_get_->GetCmdKey();
    uint16_t cmd = CmdRepository719E::CreateInstance()->sp_field_data_get_->GetCmdKey();
    if (((*getDifoCtrlData_map_ptr_))[cmd].getStopFlag()) {
      ((*getDifoCtrlData_map_ptr_))[cmd].setStopFlag(false);
    }
  } else {
    ;
  }
}
}  // namespace lidar
}  // namespace vanjee