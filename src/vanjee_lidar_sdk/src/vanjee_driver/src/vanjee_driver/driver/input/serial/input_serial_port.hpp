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
#ifdef _WIN32
#include <fcntl.h>
#include <windows.h>

#include <iostream>
#include <string>

#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/input/input.hpp>

namespace vanjee {
namespace lidar {

class InputSerialPort : public Input {
 public:
  InputSerialPort(const WJInputParam &input_param, uint8_t serial_id);
  bool init() override;
  bool start() override;
  virtual ~InputSerialPort();

 private:
  void recvPacket();
  void recvPacketSub();
  void recvPacketCmd();

  bool openPort(HANDLE &fd, std::string port_name, uint32_t baud_rate);
  void closePort(HANDLE &fd);
  int sendData(const uint8 *data, uint32 size);
  int waitRead(int32_t timeout);
  int32 send_(uint8 *buf, uint32 size) override;

 private:
  std::string port_name_;
  int baud_rate_;
  HANDLE fd_;
  HANDLE sub_fd_;
  HANDLE cmd_fd_;
};

InputSerialPort::InputSerialPort(const WJInputParam &input_param, uint8_t serial_id)
    : Input(input_param), baud_rate_(3125000), fd_(INVALID_HANDLE_VALUE), sub_fd_(INVALID_HANDLE_VALUE), cmd_fd_(INVALID_HANDLE_VALUE) {
}

inline bool InputSerialPort::init() {
  if (init_flag_) {
    return true;
  }

  bool flag = false;
  if (input_param_.port_name != "" && input_param_.baud_rate > 0) {
    bool ret = openPort(fd_, input_param_.port_name, input_param_.baud_rate);
    if (!ret) {
      WJ_ERROR << "failed to create serial port!" << WJ_REND;
      goto failMsop;
    }
    flag = true;
  }

  if (input_param_.sub_port_name != "" && input_param_.sub_baud_rate > 0) {
    bool ret = openPort(sub_fd_, input_param_.sub_port_name, input_param_.sub_baud_rate);
    if (!ret) {
      WJ_ERROR << "failed to create sub serial port!" << WJ_REND;
      goto failMsop;
    }
    flag = true;
  }
  if (input_param_.cmd_port_name != "" && input_param_.cmd_baud_rate > 0) {
    bool ret = openPort(cmd_fd_, input_param_.cmd_port_name, input_param_.cmd_baud_rate);
    if (!ret) {
      WJ_ERROR << "failed to create cmd serial port!" << WJ_REND;
      goto failMsop;
    }
    flag = true;
  }
  if (!flag) {
    WJ_ERROR << "no configured serial port information detected!" << WJ_REND;
    goto failMsop;
  }

  init_flag_ = true;
  return true;

failMsop:
  return false;
}

inline bool InputSerialPort::start() {
  if (start_flag_) {
    return true;
  }
  if (!init_flag_) {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  if (input_param_.port_name != "" && input_param_.baud_rate > 0) {
    recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacket, this));
  }
  if (input_param_.sub_port_name != "" && input_param_.sub_baud_rate > 0) {
    sub_recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacketSub, this));
  }
  if (input_param_.cmd_port_name != "" && input_param_.cmd_baud_rate > 0) {
    cmd_recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacketCmd, this));
  }
  start_flag_ = true;
  return true;
}

inline InputSerialPort::~InputSerialPort() {
  stop();
  if (fd_ != INVALID_HANDLE_VALUE) {
    closePort(fd_);
  }
  if (sub_fd_ != INVALID_HANDLE_VALUE) {
    closePort(sub_fd_);
  }
  if (cmd_fd_ != INVALID_HANDLE_VALUE) {
    closePort(cmd_fd_);
  }
}

bool InputSerialPort::openPort(HANDLE &fd, std::string port_name, uint32_t baud_rate) {
  closePort(fd);
  fd = CreateFile(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
  if (fd == INVALID_HANDLE_VALUE) {
    std::cerr << "Failed to open port: " << port_name << std::endl;
    return false;
  }

  DCB dcbSerialParams = {0};
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  dcbSerialParams.BaudRate = baud_rate;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;

  if (!SetCommState(fd, &dcbSerialParams)) {
    CloseHandle(fd);
    fd = INVALID_HANDLE_VALUE;
    return false;
  }

  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 1;
  timeouts.ReadTotalTimeoutConstant = 1000;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 1;

  if (!SetCommTimeouts(fd, &timeouts)) {
    std::cerr << "Error setting timeouts. Error code: " << GetLastError() << std::endl;
    return false;
  }

  return true;
}

void InputSerialPort::closePort(HANDLE &fd) {
  if (fd != INVALID_HANDLE_VALUE) {
    CloseHandle(fd);
    fd = INVALID_HANDLE_VALUE;
  }
}

int InputSerialPort::sendData(const uint8 *data, uint32 size) {
  // if (cmd_fd_ == INVALID_HANDLE_VALUE && fd_ == INVALID_HANDLE_VALUE) {
  //   std::cerr << "Port not open" << std::endl;
  //   return -1;
  // }
  DWORD bytes_read;
  if (cmd_fd_ != INVALID_HANDLE_VALUE) {
    return WriteFile(cmd_fd_, data, size, &bytes_read, nullptr);
  } else if (fd_ != INVALID_HANDLE_VALUE) {
    return WriteFile(fd_, data, size, &bytes_read, nullptr);
  } else {
    std::cerr << "Port not open" << std::endl;
    return -1;
  }
}

int32 InputSerialPort::send_(uint8 *buf, uint32 size) {
  int32 ret = -1;
  if (cmd_fd_ != INVALID_HANDLE_VALUE || fd_ != INVALID_HANDLE_VALUE) {
    ret = sendData(buf, size);
  }
  return ret;
}

int InputSerialPort::waitRead(int32_t timeout) {
  int ret;
  SetCommMask(fd_, EV_RXCHAR);
  DWORD dwaitResult = WaitForSingleObject(fd_, timeout);
  if (dwaitResult == WAIT_OBJECT_0) {
    ret = 1;
  } else if (dwaitResult == WAIT_TIMEOUT) {
    ret = 0;
  } else {
    ret = -1;
  }

  return ret;
}

inline void InputSerialPort::recvPacket() {
  while (!to_exit_recv_) {
    if (fd_ != INVALID_HANDLE_VALUE) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      DWORD bytesRead;
      ReadFile(fd_, pkt->buf(), pkt->bufSize(), &bytesRead, nullptr);
      if (bytesRead > 0) {
        pkt->setData(0, (int)bytesRead, "0.0.0.0");
        pushPacket(pkt);
      } else if (bytesRead == 0) {
        cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      }
    }
  }
}

inline void InputSerialPort::recvPacketSub() {
  while (!to_exit_recv_) {
    if (sub_fd_ != INVALID_HANDLE_VALUE) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      DWORD bytesRead;
      ReadFile(sub_fd_, pkt->buf(), pkt->bufSize(), &bytesRead, nullptr);
      if (bytesRead > 0) {
        pkt->setData(0, (int)bytesRead, "0.0.0.1");
        pushPacket(pkt);
      } else if (bytesRead == 0) {
        cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      }
    }
  }
}

inline void InputSerialPort::recvPacketCmd() {
  while (!to_exit_recv_) {
    if (cmd_fd_ != INVALID_HANDLE_VALUE) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      DWORD bytesRead;
      ReadFile(cmd_fd_, pkt->buf(), pkt->bufSize(), &bytesRead, nullptr);
      // if (bytesRead > 0) {
      //   pkt->setData(0, (int)bytesRead, "0.0.0.2");
      //   pushPacket(pkt);
      // } else if (bytesRead == 0) {
      //   cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
      // }
    }
  }
}

}  // namespace lidar
}  // namespace vanjee
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/driver/input/serial/my_termbits.hpp>

namespace vanjee {
namespace lidar {

class InputSerialPort : public Input {
 public:
  InputSerialPort(const WJInputParam &input_param, uint8_t serial_id);
  bool init() override;
  bool start() override;
  virtual ~InputSerialPort();

 private:
  void recvPacket();
  void recvPacketSub();
  void recvPacketCmd();

  bool openPort(int &fd, std::string port_name, uint32_t baud_rate);
  void closePort();
  ssize_t sendData(const uint8 *data, uint32 size);
  int waitRead(int fd, int32_t timeout);
  int32 send_(uint8 *buf, uint32 size) override;

 private:
  std::string port_name_;
  int baud_rate_;
  int fd_;
  int sub_fd_;
  int cmd_fd_;
};

InputSerialPort::InputSerialPort(const WJInputParam &input_param, uint8_t serial_id)
    : Input(input_param), baud_rate_(3125000), fd_(-1), sub_fd_(-1), cmd_fd_(-1) {
}

inline bool InputSerialPort::init() {
  if (init_flag_) {
    return true;
  }
  bool flag = false;
  if (input_param_.port_name != "" && input_param_.baud_rate > 0) {
    bool ret = openPort(fd_, input_param_.port_name, input_param_.baud_rate);
    if (!ret) {
      WJ_ERROR << "failed to create serial port!" << WJ_REND;
      goto failMsop;
    }
    flag = true;
  }

  if (input_param_.sub_port_name != "" && input_param_.sub_baud_rate > 0) {
    bool ret = openPort(sub_fd_, input_param_.sub_port_name, input_param_.sub_baud_rate);
    if (!ret) {
      WJ_ERROR << "failed to create sub serial port!" << WJ_REND;
      goto failMsop;
    }
    flag = true;
  }

  if (input_param_.cmd_port_name != "" && input_param_.cmd_baud_rate > 0) {
    bool ret = openPort(cmd_fd_, input_param_.cmd_port_name, input_param_.cmd_baud_rate);
    if (!ret) {
      WJ_ERROR << "failed to create cmd serial port!" << WJ_REND;
      goto failMsop;
    }
    flag = true;
  }
  if (!flag) {
    WJ_ERROR << "no configured serial port information detected!" << WJ_REND;
    goto failMsop;
  }

  init_flag_ = true;
  return true;

failMsop:
  return false;
}

inline bool InputSerialPort::start() {
  if (start_flag_) {
    return true;
  }
  if (!init_flag_) {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  if (input_param_.port_name != "" && input_param_.baud_rate > 0) {
    recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacket, this));
  }
  if (input_param_.sub_port_name != "" && input_param_.sub_baud_rate > 0) {
    sub_recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacketSub, this));
  }
  if (input_param_.cmd_port_name != "" && input_param_.cmd_baud_rate > 0) {
    cmd_recv_thread_ = std::thread(std::bind(&InputSerialPort::recvPacketCmd, this));
  }
  start_flag_ = true;
  return true;
}

inline InputSerialPort::~InputSerialPort() {
  stop();
  if (fd_ != -1 || sub_fd_ != -1 || cmd_fd_ != -1) {
    closePort();
  }
}

bool InputSerialPort::openPort(int &fd, std::string port_name, uint32_t baud_rate) {
  fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1) {
    std::cerr << "Failed to open port: " << port_name << std::endl;
    return false;
  }

  struct termios2 newtio, oldtio;
  if (ioctl(fd, TCGETS2, &oldtio) != 0) {
    return false;
  }
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  newtio.c_cflag |= CS8;

  newtio.c_cflag &= ~PARENB;

  newtio.c_cflag |= BOTHER;
  newtio.c_ispeed = baud_rate;
  newtio.c_ospeed = baud_rate;

  newtio.c_cflag &= ~CSTOPB;

  newtio.c_cc[VMIN] = 0;
  newtio.c_cc[VTIME] = 0;

  tcflush(fd, TCIFLUSH);
  if (ioctl(fd, TCSETS2, &newtio) != 0) {
    return false;
  }
  return true;
}

void InputSerialPort::closePort() {
  if (fd_ != -1) {
    close(fd_);
    fd_ = -1;
  }
  if (sub_fd_ != -1) {
    close(sub_fd_);
    sub_fd_ = -1;
  }
  if (cmd_fd_ != -1) {
    close(cmd_fd_);
    cmd_fd_ = -1;
  }
}

ssize_t InputSerialPort::sendData(const uint8 *data, uint32 size) {
  // if (fd_ == -1 || cmd_fd_ != -1) {
  //   std::cerr << "Port not open" << std::endl;
  //   return -1;
  // }
  if (cmd_fd_ != -1) {
    return write(cmd_fd_, data, size);
  } else if (fd_ != -1) {
    return write(fd_, data, size);
  } else {
    std::cerr << "Port not open" << std::endl;
    return -1;
  }
}

int32 InputSerialPort::send_(uint8 *buf, uint32 size) {
  int32 ret = -1;
  if (fd_ != -1 || cmd_fd_ != -1) {
    ret = sendData(buf, size);
  }
  return ret;
}

int InputSerialPort::waitRead(int fd, int32_t timeout) {
  int ret;
  fd_set rfds;
  struct timeval tv_timeout;

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  if (timeout >= 0) {
    tv_timeout.tv_sec = timeout / 1000;
    tv_timeout.tv_usec = (timeout % 1000) * 1000;
    ret = select(fd + 1, &rfds, NULL, NULL, &tv_timeout);
  } else {
    ret = select(fd + 1, &rfds, NULL, NULL, NULL);
  }
  return ret;
}

inline void InputSerialPort::recvPacket() {
#if ENABLE_SERIAL_BLOCK_MODE
  while (!to_exit_recv_) {
    int ret_w = 0;
    if (fd_ < 0)
      return;

    ret_w = waitRead(fd_, 1000);
    if (ret_w > 0) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret_r = read(fd_, pkt->buf(), pkt->bufSize());
      if (ret_r > 0) {
        pkt->setData(0, ret_r, "0.0.0.0");
        pushPacket(pkt);
      } else if (ret_r <= 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }
    } else {
      cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
    }
  }
#else
  bool timeout_cnt_flag = false;
  uint32_t timeout_num = 1e6;
  uint32_t ret_zero_cnt = 0;
  uint64_t start_ts = getTimeHost();
  while (!to_exit_recv_) {
    if (fd_ != -1) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret = read(fd_, pkt->buf(), pkt->bufSize());
      if (ret > 0) {
        pkt->setData(0, ret, "0.0.0.0");
        pushPacket(pkt);
        start_ts = getTimeHost();
      } else if (ret <= 0) {
        if (getTimeHost() - start_ts > 1e6) {
          start_ts = getTimeHost();
          cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }
    }
  }
#endif
}

inline void InputSerialPort::recvPacketSub() {
#if ENABLE_SERIAL_BLOCK_MODE
  while (!to_exit_recv_) {
    int ret_w = 0;
    if (sub_fd_ < 0)
      return;

    ret_w = waitRead(sub_fd_, 1000);
    if (ret_w > 0) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret_r = read(sub_fd_, pkt->buf(), pkt->bufSize());
      if (ret_r > 0) {
        pkt->setData(0, ret_r, "0.0.0.1");
        pushPacket(pkt);
      } else if (ret_r <= 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }
    } else {
      cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
    }
  }
#else
  uint64_t start_ts = getTimeHost();
  while (!to_exit_recv_) {
    if (sub_fd_ != -1) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret = read(sub_fd_, pkt->buf(), pkt->bufSize());
      if (ret > 0) {
        pkt->setData(0, ret, "0.0.0.1");
        pushPacket(pkt);
        start_ts = getTimeHost();
      } else if (ret <= 0) {
        if (getTimeHost() - start_ts > 1e6) {
          start_ts = getTimeHost();
          cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }
    }
  }
#endif
}

inline void InputSerialPort::recvPacketCmd() {
#if ENABLE_SERIAL_BLOCK_MODE
  while (!to_exit_recv_) {
    int ret_w = 0;
    if (cmd_fd_ != -1) {
      ret_w = waitRead(cmd_fd_, 1000);
      if (ret_w > 0) {
        std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
        ssize_t ret_r = read(cmd_fd_, pkt->buf(), pkt->bufSize());
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#else
  while (!to_exit_recv_) {
    if (cmd_fd_ != -1) {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(1500);
      ssize_t ret = read(cmd_fd_, pkt->buf(), pkt->bufSize());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif
}

}  // namespace lidar
}  // namespace vanjee
#endif
