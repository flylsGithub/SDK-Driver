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
class Params_LidarState722D : public ParamsAbstract {
 public:
  uint8_t flag_;
  uint8_t protocol_version_;

  std::string firmware_version_;
  std::string sn_;

  uint16_t motor_speed_;
  int16_t emission_module_temp_;
  uint16_t emission_module_sta_;
  uint8_t motor_sta_;
  uint8_t imu_acc_range_;
  uint8_t gps_sta_;
  double time_stamp_;
  uint16_t pkt_id_;

 public:
  virtual std::shared_ptr<std::vector<uint8>> GetBytes() {
    std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
    return nullptr;
  }

  virtual void Load(ProtocolBase& protocol) {
    std::array<uint8_t, 16> sn;
    std::array<uint8_t, 20> firmware_version;
    uint32_t index = 0;
    auto buf = protocol.Content.data();
    protocol_version_ = buf[95];

    uint8_t* data_sn = reinterpret_cast<uint8_t*>(buf);
    std::copy(data_sn, data_sn + sn.size(), std::begin(sn));
    for (uint32_t i = 0; i < sn.size(); i++) {
      if (sn[i] == '\0')
        break;
      sn_ += sn[i];
    }
    index += sn.size();
    index += 52;

    uint8_t* data_firmware_version = reinterpret_cast<uint8_t*>(buf + index);
    std::copy(data_firmware_version, data_firmware_version + firmware_version.size(), std::begin(firmware_version));
    for (uint32_t i = 0; i < firmware_version.size(); i++) {
      if (firmware_version[i] == '\0')
        break;
      firmware_version_ += firmware_version[i];
    }
    index += firmware_version.size() + 14;

    emission_module_temp_ = buf[index] | (buf[index + 1] << 8);
    index += 4;

    motor_speed_ = buf[index] | (buf[index + 1] << 8);
    index += 14;

    emission_module_sta_ = buf[index] | (buf[index + 1] << 8);
    index += 4;

    motor_sta_ = buf[index];
    index += 42;

    imu_acc_range_ = buf[index];
    index += 4;

    gps_sta_ = buf[index];
    index += 22;

    WJTimestampYMD tm{(int32_t)(buf[index]), buf[index + 1], buf[index + 2], buf[index + 3], buf[index + 4], buf[index + 5]};
    double usec = (buf[index + 6] + (buf[index + 7] << 8) + (buf[index + 8] << 16) + (buf[index + 9] << 24)) * 1e-6;
    time_stamp_ = parseTimeYMD(&tm) * 1e-6 + usec;
    index += 18;

    pkt_id_ = buf[index] | (buf[index + 1] << 8);
  }
};
}  // namespace lidar
}  // namespace vanjee