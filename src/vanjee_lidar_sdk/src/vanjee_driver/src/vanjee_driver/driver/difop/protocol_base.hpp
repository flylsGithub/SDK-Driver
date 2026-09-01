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

class CheckClass;
namespace vanjee {
namespace lidar {
class CheckClass {
 public:
  enum CHECKTYPE { No_Check = 0x00, Xor_Check = 0x01, Crc16_Check = 0x02 };

 public:
  static uint8 Xor(const std::vector<uint8> &buf, uint32 start, uint32 len) {
    uint8 xor_ = 0x00;

    for (uint32 i = 0; i < len; i++) {
      xor_ ^= buf[start + i];
    }

    return xor_;
  }

  static uint16 Crc16(const std::vector<uint8> &buf, uint32 start, uint32 len) {
    static uint16 crctab[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};
    uint16 crc16 = 0x00;

    for (uint32 i = 0; i < len; i++) {
      crc16 = (uint16)(crctab[(crc16 >> 8) ^ buf[start + i]] ^ (crc16 << 8));
    }

    return crc16;
  }

  static uint16 Crc16_719e(const std::vector<uint8> &buf, uint32 start, uint32 len) {
    static uint16 crctab_719e[256] = {
        0x0000, 0xbd33, 0xcf3b, 0x7208, 0x2b2b, 0x9618, 0xe410, 0x5923, 0x5656, 0xeb65, 0x996d, 0x245e, 0x7d7d, 0xc04e, 0xb246, 0x0f75,
        0xacac, 0x119f, 0x6397, 0xdea4, 0x8787, 0x3ab4, 0x48bc, 0xf58f, 0xfafa, 0x47c9, 0x35c1, 0x88f2, 0xd1d1, 0x6ce2, 0x1eea, 0xa3d9,
        0xec05, 0x5136, 0x233e, 0x9e0d, 0xc72e, 0x7a1d, 0x0815, 0xb526, 0xba53, 0x0760, 0x7568, 0xc85b, 0x9178, 0x2c4b, 0x5e43, 0xe370,
        0x40a9, 0xfd9a, 0x8f92, 0x32a1, 0x6b82, 0xd6b1, 0xa4b9, 0x198a, 0x16ff, 0xabcc, 0xd9c4, 0x64f7, 0x3dd4, 0x80e7, 0xf2ef, 0x4fdc,
        0x6d57, 0xd064, 0xa26c, 0x1f5f, 0x467c, 0xfb4f, 0x8947, 0x3474, 0x3b01, 0x8632, 0xf43a, 0x4909, 0x102a, 0xad19, 0xdf11, 0x6222,
        0xc1fb, 0x7cc8, 0x0ec0, 0xb3f3, 0xead0, 0x57e3, 0x25eb, 0x98d8, 0x97ad, 0x2a9e, 0x5896, 0xe5a5, 0xbc86, 0x01b5, 0x73bd, 0xce8e,
        0x8152, 0x3c61, 0x4e69, 0xf35a, 0xaa79, 0x174a, 0x6542, 0xd871, 0xd704, 0x6a37, 0x183f, 0xa50c, 0xfc2f, 0x411c, 0x3314, 0x8e27,
        0x2dfe, 0x90cd, 0xe2c5, 0x5ff6, 0x06d5, 0xbbe6, 0xc9ee, 0x74dd, 0x7ba8, 0xc69b, 0xb493, 0x09a0, 0x5083, 0xedb0, 0x9fb8, 0x228b,
        0xdaae, 0x679d, 0x1595, 0xa8a6, 0xf185, 0x4cb6, 0x3ebe, 0x838d, 0x8cf8, 0x31cb, 0x43c3, 0xfef0, 0xa7d3, 0x1ae0, 0x68e8, 0xd5db,
        0x7602, 0xcb31, 0xb939, 0x040a, 0x5d29, 0xe01a, 0x9212, 0x2f21, 0x2054, 0x9d67, 0xef6f, 0x525c, 0x0b7f, 0xb64c, 0xc444, 0x7977,
        0x36ab, 0x8b98, 0xf990, 0x44a3, 0x1d80, 0xa0b3, 0xd2bb, 0x6f88, 0x60fd, 0xddce, 0xafc6, 0x12f5, 0x4bd6, 0xf6e5, 0x84ed, 0x39de,
        0x9a07, 0x2734, 0x553c, 0xe80f, 0xb12c, 0x0c1f, 0x7e17, 0xc324, 0xcc51, 0x7162, 0x036a, 0xbe59, 0xe77a, 0x5a49, 0x2841, 0x9572,
        0xb7f9, 0x0aca, 0x78c2, 0xc5f1, 0x9cd2, 0x21e1, 0x53e9, 0xeeda, 0xe1af, 0x5c9c, 0x2e94, 0x93a7, 0xca84, 0x77b7, 0x05bf, 0xb88c,
        0x1b55, 0xa666, 0xd46e, 0x695d, 0x307e, 0x8d4d, 0xff45, 0x4276, 0x4d03, 0xf030, 0x8238, 0x3f0b, 0x6628, 0xdb1b, 0xa913, 0x1420,
        0x5bfc, 0xe6cf, 0x94c7, 0x29f4, 0x70d7, 0xcde4, 0xbfec, 0x02df, 0x0daa, 0xb099, 0xc291, 0x7fa2, 0x2681, 0x9bb2, 0xe9ba, 0x5489,
        0xf750, 0x4a63, 0x386b, 0x8558, 0xdc7b, 0x6148, 0x1340, 0xae73, 0xa106, 0x1c35, 0x6e3d, 0xd30e, 0x8a2d, 0x371e, 0x4516, 0xf825};
    uint16_t crc16 = 0x00;
    for (uint32_t i = 0; i < len; i++) {
      uint8_t tblIndex = (uint8_t)((crc16 ^ buf[start + i]) & 0xff);
      crc16 = (uint16_t)(crctab_719e[tblIndex] ^ (crc16 >> 8));
    }
    return crc16;
  }

  static uint32_t Crc32_719e(const std::vector<uint8_t> &buf, uint32_t start, uint32_t len) {
    static uint32_t crc32_tab_719e[256] = {
        0x00000000, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c, 0x26a1e7e8, 0xd4ca64eb, 0x8ad958cf, 0x78b2dbcc, 0x6be22838,
        0x9989ab3b, 0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24, 0x105ec76f, 0xe235446c, 0xf165b798, 0x030e349b, 0xd7c45070, 0x25afd373,
        0x36ff2087, 0xc494a384, 0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc, 0xbc267848, 0x4e4dfb4b, 0x20bd8ede,
        0xd2d60ddd, 0xc186fe29, 0x33ed7d2a, 0xe72719c1, 0x154c9ac2, 0x061c6936, 0xf477ea35, 0xaa64d611, 0x580f5512, 0x4b5fa6e6, 0xb93425e5,
        0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa, 0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x05125dad, 0x1642ae59,
        0xe4292d5a, 0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a, 0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595, 0x417b1dbc, 0xb3109ebf,
        0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957, 0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0x0c38d26c,
        0xfe53516f, 0xed03a29b, 0x1f682198, 0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927, 0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38,
        0xdbfc821c, 0x2997011f, 0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0x0f36e6f7, 0x61c69362, 0x93ad1061, 0x80fde395,
        0x72966096, 0xa65c047d, 0x5437877e, 0x4767748a, 0xb50cf789, 0xeb1fcbad, 0x197448ae, 0x0a24bb5a, 0xf84f3859, 0x2c855cb2, 0xdeeedfb1,
        0xcdbe2c45, 0x3fd5af46, 0x7198540d, 0x83f3d70e, 0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6, 0xfb410cc2,
        0x092a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de, 0xdde0eb2a, 0x2f8b6829, 0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c,
        0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93, 0x082f63b7, 0xfa44e0b4, 0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f,
        0xdce5075c, 0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b, 0xb4091bff, 0x466298fc, 0x1871a4d8, 0xea1a27db,
        0xf94ad42f, 0x0b21572c, 0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033, 0xa24bb5a6, 0x502036a5, 0x4370c551, 0xb11b4652, 0x65d122b9,
        0x97baa1ba, 0x84ea524e, 0x7681d14d, 0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975, 0x0e330a81, 0xfc588982,
        0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d, 0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622, 0x38cc2a06, 0xcaa7a905, 0xd9f75af1,
        0x2b9cd9f2, 0xff56bd19, 0x0d3d3e1a, 0x1e6dcdee, 0xec064eed, 0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x0417b1db, 0xf67c32d8,
        0xe52cc12c, 0x1747422f, 0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff, 0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0, 0xd3d3e1ab,
        0x21b862a8, 0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x07198540, 0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90,
        0x9e902e7b, 0x6cfbad78, 0x7fab5e8c, 0x8dc0dd8f, 0xe330a81a, 0x115b2b19, 0x020bd8ed, 0xf0605bee, 0x24aa3f05, 0xd6c1bc06, 0xc5914ff2,
        0x37faccf1, 0x69e9f0d5, 0x9b8273d6, 0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e, 0xf36e6f75, 0x0105ec76,
        0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69, 0xd5cf889d, 0x27a40b9e, 0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e, 0xbe2da0a5,
        0x4c4623a6, 0x5f16d052, 0xad7d5351};

    uint32_t crc32 = 0x00;
    for (size_t i = 0; i < len; i++) {
      uint8_t index = (uint8_t)((crc32 ^ buf[i + start]) & 0xff);
      crc32 = (crc32_tab_719e[index] ^ (crc32 >> 8)) & 0xffffffff;
    }
    return crc32;
  }
};

class ProtocolBase {
 public:
  /// @brief version_v1: WLR-722Z and WLR-722F, version_safe: WLR-719E, version_base: others;
  enum ProtocolVersionDifop { version_base = 0x00, version_v1 = 0x01, version_safe = 0x02 };
  ProtocolVersionDifop protocol_version_ = ProtocolVersionDifop::version_base;
  /// @brief BigEndian-1 LittleEndian-2;
  enum DataEndiannessMode { big_endian = 0x01, little_endian = 0x02 };
  uint8 ByteOrder = DataEndiannessMode::big_endian;
  static const uint16 FRAME_MIN_LENGTH = 28;
  static const uint16 FRAME_MIN_LENGTH_V1 = 18;
  ByteVectorPtr Buffer;

  ByteVector Head = {0xFF, 0XAA};
  ByteVector Length = {0x00, 0x00};
  ByteVector Idx = {0x00, 0x00};
  ByteVector Timestamp = {0x00, 0x00, 0x00, 0x00};
  uint8 CheckType = 0x01;
  uint8 Type = 0x00;
  ByteVector DeviceType = {0x00, 0x00};
  ByteVector Config = {0x00, 0x00, 0x00, 0x00};
  ByteVector Remain = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8 MainCmd = 0x00;
  uint8 SubCmd = 0x00;
  ByteVector CmdParams = {0x00, 0x00};
  ByteVector Content;
  ByteVector Check = {0x00, 0x00};
  ByteVector Tail = {0xEE, 0xEE};

 public:
  ProtocolBase(uint16 idx, uint32 timestamp, uint8 checkType, uint8 type, const ByteVector &deviceType, const ByteVector &remain, uint8 mainCmd,
               uint8 subCmd, const ByteVector &cmdParams, const ByteVector &content, uint8_t byte_order = DataEndiannessMode::big_endian,
               uint16 head = 0xFFAA) {
    ByteOrder = byte_order;
    if (ByteOrder == DataEndiannessMode::big_endian) {
      Idx[0] = (idx >> 8) & 0xFF;
      Idx[1] = (idx >> 0) & 0xFF;

      Timestamp[0] = (timestamp >> 24) & 0xFF;
      Timestamp[1] = (timestamp >> 16) & 0xFF;
      Timestamp[2] = (timestamp >> 8) & 0xFF;
      Timestamp[3] = (timestamp >> 0) & 0xFF;
    } else {
      Idx[0] = (idx >> 0) & 0xFF;
      Idx[1] = (idx >> 8) & 0xFF;

      Timestamp[0] = (timestamp >> 0) & 0xFF;
      Timestamp[1] = (timestamp >> 8) & 0xFF;
      Timestamp[2] = (timestamp >> 16) & 0xFF;
      Timestamp[3] = (timestamp >> 24) & 0xFF;
    }

    CheckType = checkType;
    Type = type;
    DeviceType = deviceType;
    Remain = remain;
    MainCmd = mainCmd;
    SubCmd = subCmd;
    CmdParams = cmdParams;
    Content = content;
    Head[0] = (uint8_t)((head >> 8) & 0xFF);
    Head[1] = (uint8_t)(head & 0xFF);
  }

  ProtocolBase(const ByteVector &idx, const ByteVector &timestamp, uint8 checkType, uint8 type, const ByteVector &deviceType,
               const ByteVector &remain, uint8 mainCmd, uint8 subCmd, const ByteVector &cmdParams, const ByteVector &content,
               uint8_t byte_order = DataEndiannessMode::big_endian, uint16 head = 0xFFAA)
      : Idx(idx), Timestamp(timestamp), DeviceType(deviceType), Remain(remain), CmdParams(cmdParams), Content(content) {
    ByteOrder = byte_order;
    CheckType = checkType;
    Type = type;
    MainCmd = mainCmd;
    SubCmd = subCmd;
    Head[0] = (uint8_t)((head >> 8) & 0xFF);
    Head[1] = (uint8_t)(head & 0xFF);
  }

  ProtocolBase(uint8 checkType, uint8 type, const ByteVector &deviceType, uint8 mainCmd, uint8 subCmd, const ByteVector &content,
               uint8_t byte_order = DataEndiannessMode::big_endian, uint16 head = 0xFFAA)
      : DeviceType(deviceType), Content(content) {
    ByteOrder = byte_order;
    CheckType = checkType;
    Type = type;
    Config[0] = deviceType[1];
    Config[1] = (uint8)(type | (checkType << 1));
    Config[2] = 0;
    Config[3] = 0;
    MainCmd = mainCmd;
    SubCmd = subCmd;
    Head[0] = (uint8_t)((head >> 8) & 0xFF);
    Head[1] = (uint8_t)(head & 0xFF);
  }

  ProtocolBase(uint8_t byte_order = DataEndiannessMode::big_endian) {
    ByteOrder = byte_order;
  }

  bool Parse(ByteVector &buf) {
    if (protocol_version_ == ProtocolVersionDifop::version_v1) {
      return ParseVersionV1(buf);
    } else if (protocol_version_ == ProtocolVersionDifop::version_safe) {
      return ParseVersionSafe(buf);
    } else {
      return ParseVersionBase(buf);
    }
  }

  bool ParseVersionBase(ByteVector &buf) {
    Buffer = std::make_shared<ByteVector>(buf);
    if (Buffer->size() < FRAME_MIN_LENGTH)
      return false;

    if (!(Head[0] == (*Buffer)[0] && Head[1] == (*Buffer)[1] && Tail[0] == (*Buffer)[Buffer->size() - 2] && Tail[1] == (*Buffer)[Buffer->size() - 1]))
      return false;
    int idx = 2;

    auto itr = Buffer->begin() + idx;
    std::copy(itr, itr + Length.size(), Length.begin());
    itr = itr + Length.size();
    std::copy(itr, itr + Idx.size(), Idx.begin());
    itr = itr + Idx.size();
    std::copy(itr, itr + Timestamp.size(), Timestamp.begin());
    itr = itr + Timestamp.size();
    CheckType = *itr++;
    Type = *itr++;
    std::copy(itr, itr + DeviceType.size(), DeviceType.begin());
    itr = itr + DeviceType.size();
    std::copy(itr, itr + Remain.size(), Remain.begin());
    itr = itr + Remain.size();
    MainCmd = *itr++;
    SubCmd = *itr++;

    std::copy(itr, itr + CmdParams.size(), CmdParams.begin());
    itr = itr + CmdParams.size();
    Content.clear();
    Content.shrink_to_fit();
    Content.assign(itr, Buffer->end() - 4);
    itr = itr + Content.size();

    std::copy(itr, itr + Check.size(), Check.begin());
    itr = itr + Check.size();

    if (CheckType == CheckClass::No_Check)
      return true;

    if (CheckType == CheckClass::Xor_Check) {
      if (CheckClass::Xor(buf, 2, buf.size() - 6) == (uint16)((Check[0] << 8) + Check[1])) {
        return true;
      } else {
        return false;
      }
    }

    if (CheckType == CheckClass::Crc16_Check) {
      if (CheckClass::Crc16(buf, 2, buf.size() - 6) == (uint16)((Check[0] << 8) + Check[1])) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

  bool ParseVersionV1(ByteVector &buf) {
    Buffer = std::make_shared<ByteVector>(buf);
    if (Buffer->size() < FRAME_MIN_LENGTH_V1)
      return false;

    if (!(Head[0] == (*Buffer)[0] && Head[1] == (*Buffer)[1] && Tail[0] == (*Buffer)[Buffer->size() - 2] && Tail[1] == (*Buffer)[Buffer->size() - 1]))
      return false;
    int idx = 2;

    auto itr = Buffer->begin() + idx;
    std::copy(itr, itr + Length.size(), Length.begin());
    itr = itr + Length.size();

    std::copy(itr, itr + Config.size(), Config.begin());
    itr = itr + Config.size();

    DeviceType[0] = 0;
    DeviceType[1] = Config[0];
    CheckType = (Config[1] & 0x06) >> 1;
    Type = Config[1] & 0x01;

    MainCmd = *itr++;
    SubCmd = *itr++;
    Content.clear();
    Content.shrink_to_fit();
    Content.assign(itr, Buffer->end() - 4);
    itr = itr + Content.size();

    std::copy(itr, itr + Check.size(), Check.begin());
    itr = itr + Check.size();

    if (CheckType == CheckClass::No_Check)
      return true;

    if (CheckType == CheckClass::Xor_Check) {
      if (CheckClass::Xor(buf, 2, buf.size() - 6) == (uint16)((Check[0] << 8) + Check[1])) {
        return true;
      } else {
        return false;
      }
    }

    if (CheckType == CheckClass::Crc16_Check) {
      if (CheckClass::Crc16(buf, 2, buf.size() - 6) == (uint16)((Check[0] << 8) + Check[1])) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

  bool ParseVersionSafe(ByteVector &buf) {
    Buffer = std::make_shared<ByteVector>(buf);
    if (Buffer->size() < FRAME_MIN_LENGTH)
      return false;
    bool crc32_flag = false;
    if (Buffer->size() > 255) {
      crc32_flag = true;
    }

    if (!(Head[0] == (*Buffer)[0] && Head[1] == (*Buffer)[1] && Tail[0] == (*Buffer)[Buffer->size() - 2] && Tail[1] == (*Buffer)[Buffer->size() - 1]))
      return false;
    int idx = 2;

    auto itr = Buffer->begin() + idx;
    std::copy(itr, itr + Length.size(), Length.begin());
    itr = itr + Length.size();
    std::copy(itr, itr + Idx.size(), Idx.begin());
    itr = itr + Idx.size();
    std::copy(itr, itr + Timestamp.size(), Timestamp.begin());
    itr = itr + Timestamp.size();
    CheckType = *itr++;
    Type = *itr++;
    std::copy(itr, itr + DeviceType.size(), DeviceType.begin());
    itr = itr + DeviceType.size();
    std::copy(itr, itr + Remain.size(), Remain.begin());
    itr = itr + Remain.size();
    MainCmd = *itr++;
    SubCmd = *itr++;

    std::copy(itr, itr + CmdParams.size(), CmdParams.begin());
    itr = itr + CmdParams.size();
    Content.clear();
    Content.shrink_to_fit();
    Content.assign(itr, Buffer->end() - (crc32_flag ? 6 : 4));
    itr = itr + Content.size();

    // std::copy(itr, itr + Check.size(), Check.begin());
    // itr = itr + Check.size();

    Check.clear();
    Check.shrink_to_fit();
    Check.assign(itr, Buffer->end() - 2);
    itr = itr + Check.size();

    if (CheckType == CheckClass::No_Check)
      return true;

    if (CheckType == CheckClass::Xor_Check) {
      if (CheckClass::Xor(buf, 2, buf.size() - 6) == (uint16)((Check[0] << 8) + Check[1])) {
        return true;
      } else {
        return false;
      }
    }
    if (CheckType == CheckClass::Crc16_Check) {
      if (crc32_flag) {
        uint32_t check = 0;
        if (ByteOrder == DataEndiannessMode::big_endian)
          check = (uint32_t)((Check[0] << 24) + (Check[1] << 16) + (Check[2] << 8) + Check[3]);
        else
          check = (uint32_t)((Check[3] << 24) + (Check[2] << 16) + (Check[1] << 8) + Check[0]);

        if (CheckClass::Crc32_719e(buf, 2, buf.size() - 8) == check) {
          return true;
        } else {
          return false;
        }
      } else {
        uint16 check = 0;
        if (ByteOrder == DataEndiannessMode::big_endian)
          check = (uint16)((Check[0] << 8) + Check[1]);
        else
          check = (uint16)((Check[1] << 8) + Check[0]);

        if (CheckClass::Crc16_719e(buf, 2, buf.size() - 6) == check) {
          return true;
        } else {
          return false;
        }
      }
    }
    return false;
  }

  ByteVectorPtr GetBytes(ProtocolVersionDifop protocolVersion = ProtocolVersionDifop::version_base) {
    ByteVectorPtr buf = std::make_shared<std::vector<uint8>>();
    if (protocolVersion == ProtocolVersionDifop::version_v1) {
      return GetBytesVersionV1();
    } else if (protocolVersion == ProtocolVersionDifop::version_safe) {
      return GetBytesVersionSafe();
    } else {
      return GetBytesVersionBase();
    }
  }

  ByteVectorPtr GetBytesVersionBase() {
    ByteVectorPtr buf = std::make_shared<std::vector<uint8>>();
    int len = Length.size() + Idx.size() + Timestamp.size() + 1 + 1 + DeviceType.size() + Remain.size() + 1 + 1 + CmdParams.size() + Content.size() +
              Check.size();

    if (ByteOrder == DataEndiannessMode::big_endian) {
      Length[0] = (uint8)((len >> 8) & 0xFF);
      Length[1] = (uint8)(len & 0xFF);
    } else {
      Length[0] = (uint8)(len & 0xFF);
      Length[1] = (uint8)((len >> 8) & 0xFF);
    }

    buf->insert(buf->end(), Head.begin(), Head.end());
    buf->insert(buf->end(), Length.begin(), Length.end());
    buf->insert(buf->end(), Idx.begin(), Idx.end());
    buf->insert(buf->end(), Timestamp.begin(), Timestamp.end());
    buf->emplace_back(CheckType);
    buf->emplace_back(Type);
    buf->insert(buf->end(), DeviceType.begin(), DeviceType.end());
    buf->insert(buf->end(), Remain.begin(), Remain.end());
    buf->emplace_back(MainCmd);
    buf->emplace_back(SubCmd);
    buf->insert(buf->end(), CmdParams.begin(), CmdParams.end());
    buf->insert(buf->end(), Content.begin(), Content.end());
    if (CheckType == (uint8)CheckClass::Xor_Check) {
      Check[0] = 0;
      Check[1] = CheckClass::Xor(*buf, 2, buf->size() - 2);
    } else if (CheckType == (uint8)CheckClass::Crc16_Check) {
      Check[0] = 0;
      Check[1] = CheckClass::Crc16(*buf, 2, buf->size() - 2);
    }

    buf->insert(buf->end(), Check.begin(), Check.end());
    buf->insert(buf->end(), Tail.begin(), Tail.end());
    return buf;
  }

  ByteVectorPtr GetBytesVersionV1() {
    ByteVectorPtr buf = std::make_shared<std::vector<uint8>>();
    int len = Length.size() + 4 + 1 + 1 + Content.size() + Check.size();

    if (ByteOrder == DataEndiannessMode::big_endian) {
      Length[0] = (uint8)((len >> 8) & 0xFF);
      Length[1] = (uint8)(len & 0xFF);
    } else {
      Length[0] = (uint8)(len & 0xFF);
      Length[1] = (uint8)((len >> 8) & 0xFF);
    }

    buf->insert(buf->end(), Head.begin(), Head.end());
    buf->insert(buf->end(), Length.begin(), Length.end());
    buf->insert(buf->end(), Config.begin(), Config.end());

    buf->emplace_back(MainCmd);
    buf->emplace_back(SubCmd);
    buf->insert(buf->end(), Content.begin(), Content.end());
    if (CheckType == (uint8)CheckClass::Xor_Check) {
      Check[0] = 0;
      Check[1] = CheckClass::Xor(*buf, 2, buf->size() - 2);
    } else if (CheckType == (uint8)CheckClass::Crc16_Check) {
      Check[0] = 0;
      Check[1] = CheckClass::Crc16(*buf, 2, buf->size() - 2);
    }

    buf->insert(buf->end(), Check.begin(), Check.end());
    buf->insert(buf->end(), Tail.begin(), Tail.end());
    return buf;
  }

  ByteVectorPtr GetBytesVersionSafe() {
    ByteVectorPtr buf = std::make_shared<std::vector<uint8>>();
    int len = Length.size() + Idx.size() + Timestamp.size() + 1 + 1 + DeviceType.size() + Remain.size() + 1 + 1 + CmdParams.size() + Content.size() +
              Check.size();

    if (ByteOrder == DataEndiannessMode::big_endian) {
      Length[0] = (uint8)((len >> 8) & 0xFF);
      Length[1] = (uint8)(len & 0xFF);
    } else {
      Length[0] = (uint8)(len & 0xFF);
      Length[1] = (uint8)((len >> 8) & 0xFF);
    }

    buf->insert(buf->end(), Head.begin(), Head.end());
    buf->insert(buf->end(), Length.begin(), Length.end());
    buf->insert(buf->end(), Idx.begin(), Idx.end());
    buf->insert(buf->end(), Timestamp.begin(), Timestamp.end());
    buf->emplace_back(CheckType);
    buf->emplace_back(Type);
    buf->insert(buf->end(), DeviceType.begin(), DeviceType.end());
    buf->insert(buf->end(), Remain.begin(), Remain.end());
    buf->emplace_back(MainCmd);
    buf->emplace_back(SubCmd);
    buf->insert(buf->end(), CmdParams.begin(), CmdParams.end());
    buf->insert(buf->end(), Content.begin(), Content.end());
    if (CheckType == (uint8)CheckClass::Xor_Check) {
      Check[0] = 0;
      Check[1] = CheckClass::Xor(*buf, 2, buf->size() - 2);
    } else if (CheckType == (uint8)CheckClass::Crc16_Check) {
      uint16_t check = CheckClass::Crc16_719e(*buf, 2, buf->size() - 2);
      if (ByteOrder == DataEndiannessMode::big_endian) {
        Check[0] = (check >> 8) & 0xff;
        Check[1] = check & 0xff;
      } else {
        Check[0] = check & 0xff;
        Check[1] = (check >> 8) & 0xff;
      }
    }

    buf->insert(buf->end(), Check.begin(), Check.end());
    buf->insert(buf->end(), Tail.begin(), Tail.end());
    return buf;
  }

  std::shared_ptr<ProtocolBase> CreateNew() {
    std::shared_ptr<ProtocolBase> pb(new ProtocolBase());
    pb->protocol_version_ = protocol_version_;
    pb->ByteOrder = ByteOrder;

    pb->Head = Head;
    pb->Length = Length;
    pb->Idx = Idx;
    pb->Timestamp = Timestamp;
    pb->DeviceType = DeviceType;
    pb->Remain = Remain;
    pb->CmdParams = CmdParams;
    pb->Check = Check;
    pb->Tail = Tail;

    pb->CheckType = CheckType;
    pb->Type = Type;
    pb->Config = Config;

    return pb;
  }

  void SetHeader(uint16_t head) {
    Head[0] = (uint8_t)((head >> 8) & 0xff);
    Head[1] = (uint8_t)(head & 0xff);
  }

  uint16_t GetHeader() {
    return (uint16_t)(Head[0] << 8 | Head[1]);
  }

  void SetByteOrder(uint8_t byte_order) {
    ByteOrder = byte_order;
  }

  uint8_t GetByteOrder() {
    return ByteOrder;
  }

  void SetProtocolVersion(ProtocolVersionDifop protocol_version) {
    protocol_version_ = protocol_version;
  }
};

}  // namespace lidar
}  // namespace vanjee