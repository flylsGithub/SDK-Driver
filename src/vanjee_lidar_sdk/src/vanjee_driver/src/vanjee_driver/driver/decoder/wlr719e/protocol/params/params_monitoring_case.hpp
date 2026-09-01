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

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _VanjeeMonitoringCaseIntrusionInfo719E {
  uint8_t info[2];
} VanjeeMonitoringCaseIntrusionInfo719E;

typedef struct _VanjeeMonitoringCaseInfo719E {
  uint8_t case_id;
  uint16_t static_input;
  uint8_t dynamic_input[5];
  uint8_t intrusion_num;
  VanjeeMonitoringCaseIntrusionInfo719E intrusion[8];
} VanjeeMonitoringCaseInfo719E;
#pragma pack(pop)
class Params_MonitoringCase719E : public ParamsAbstract {
 public:
  bool last_pkt_flag_;
  uint32_t send_total_pkt_num_;

  uint8_t monitoring_case_list_num_;
  uint8_t monitoring_case_list_id_;
  uint8_t monitoring_case_num_;
  uint16_t input_delay_;
  std::vector<VanjeeMonitoringCaseInfo719E> data_;

 public:
  virtual std::shared_ptr<std::vector<uint8_t>> GetBytes() {
    std::shared_ptr<std::vector<uint8_t>> buf = std::make_shared<std::vector<uint8_t>>();
    return nullptr;
  }

  virtual void Load(ProtocolBase& protocol) {
    last_pkt_flag_ = protocol.CmdParams.data()[1] == 0 ? false : true;
    auto buf = protocol.Content.data();
    send_total_pkt_num_ = buf[0];

    monitoring_case_list_num_ = buf[0];
    monitoring_case_list_id_ = buf[1];
    monitoring_case_num_ = buf[2];
    input_delay_ = (buf[3] | (buf[4] << 8));

    for (int i = 0; i < 1; i++) {
      uint32_t offset = 5 + sizeof(VanjeeMonitoringCaseInfo719E) * i;
      VanjeeMonitoringCaseInfo719E& case_info = *(VanjeeMonitoringCaseInfo719E*)(buf + offset);
      data_.emplace_back(case_info);
    }
  }
};
}  // namespace lidar
}  // namespace vanjee