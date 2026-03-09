// Copyright (C) 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
#define SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_

// std
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
// project
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/logger/log.hpp"

namespace pka::serial_driver {

template <int capacity = 16>
class FixedPacketTool {
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  //表示禁止使用默认构造函数
  FixedPacketTool() = delete;
  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter) : transporter_(transporter) 
  {
    if (!transporter) 
    {
      throw std::invalid_argument("transporter is nullptr");
    }
    PKA_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
  }
  //析构函数
  ~FixedPacketTool() 
  { 
    enbaleRealtimeSend(false); 
  }

  bool isOpen() { return transporter_->isOpen(); }
  void enbaleRealtimeSend(bool enable);
  void enbaleDataPrint(bool enable) { use_data_print_ = enable; }
  bool sendPacket(const FixedPacket<capacity> &packet);
  bool recvPacket(FixedPacket<capacity> &packet);

  std::string getErrorMessage() { return transporter_->errorMessage(); }

private:
  bool checkPacket(uint8_t *tmp_buffer, int recv_len);
  bool simpleSendPacket(const FixedPacket<capacity> &packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  // data
  uint8_t tmp_buffer_[capacity];       // NOLINT
  uint8_t recv_buffer_[capacity * 2];  // NOLINT
  int recv_buf_len_ = 0; //
  //用于实时发送
  // for realtime sending
  bool use_realtime_send_{false};
  bool use_data_print_{false};
  std::mutex realtime_send_mut_;
  std::unique_ptr<std::thread> realtime_send_thread_;
  std::queue<FixedPacket<capacity>> realtime_packets_;
};

template <int capacity>
bool FixedPacketTool<capacity>::checkPacket(uint8_t *buffer, int recv_len) 
{
  // 检查长度
  if (recv_len != capacity) 
  {
    // std::cout<<"1"<<std::endl;
    return false;
  }
  // 检查帧头，帧尾,
  if ((buffer[0] != 0xff) || (buffer[capacity - 1] != 0x0d)) 
  {
    // std::cout<<"2"<<std::endl;
    return false;
  }
  // TODO(gezp): 检查check_byte(buffer[capacity-2]),可采用异或校验(BCC)
  return true;
}

template <int capacity>
bool FixedPacketTool<capacity>::simpleSendPacket(const FixedPacket<capacity> &packet) 
{
  if (transporter_->write(packet.buffer(), capacity) == capacity) 
  {
    return true;
  } 
  else 
  {
    // reconnect
    PKA_ERROR("serial_driver", "transporter_->write() failed");
    transporter_->close();
    transporter_->open();
    return false;
  }
}

template <int capacity>
void FixedPacketTool<capacity>::enbaleRealtimeSend(bool enable) 
{
  if (enable == use_realtime_send_) 
  {
    return;
  }
  if (enable) 
  {
    use_realtime_send_ = true;
    realtime_send_thread_ = std::make_unique<std::thread>([&]() {
      FixedPacket<capacity> packet;
      while (use_realtime_send_) {
        bool empty = true;
        {
          std::lock_guard<std::mutex> lock(realtime_send_mut_);
          empty = realtime_packets_.empty();
          if (!empty) {
            packet = realtime_packets_.front();
            realtime_packets_.pop();
          }
        }
        if (!empty) 
        {
          simpleSendPacket(packet);
        }
        else 
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    });
  } 
  else 
  {
    use_realtime_send_ = false;
    realtime_send_thread_->join();
    realtime_send_thread_.reset();
  }
}

template <int capacity>
bool FixedPacketTool<capacity>::sendPacket(const FixedPacket<capacity> &packet) 
{
  if (use_realtime_send_) 
  {
    std::lock_guard<std::mutex> lock(realtime_send_mut_);
    realtime_packets_.push(packet);
    return true;
  }
  else 
  {
    return simpleSendPacket(packet);
  }
}


//
template <int capacity>
bool FixedPacketTool<capacity>::recvPacket(FixedPacket<capacity> &packet) 
{
  int recv_len = transporter_->read(tmp_buffer_, capacity);
  // std::cout<<"recv_len:"<<recv_len<<std::endl;
  if (recv_len > 0) 
  {
    // std::cout<<std::hex<<static_cast<int>(tmp_buffer_[0])<<std::endl;
    // print data
    // if (true) 
    // {
    //   for (int i = 0; i < recv_len; i++) 
    //   {
    //     std::cout << std::hex << static_cast<int>(tmp_buffer_[i]) << " ";
    //   }
    //   std::cout << "\n";
    // }
    // if(use_data_print_)
    // {
    //   for(int i = 1;i<recv_len-1;i++)
    //   {

    //   }
    //   std::cout<<"\n";
    // }

    // check packet
    
    if (checkPacket(tmp_buffer_, recv_len)) 
    {
      // std::cout<<"right digits"<<std::endl;
      packet.copyFrom(tmp_buffer_);
      return true;
    }
    //可能有问题(需修改)
    else 
    {
      // 如果是断帧，拼接缓存，并遍历校验，获得合法数据
      PKA_INFO("serial_driver", "checkPacket() failed, check if it is a broken frame");
      if (recv_buf_len_ + recv_len > capacity * 2) //recv_len = 16 capacity = 16
      {
        recv_buf_len_ = 0;
      }
      // 拼接缓存
      memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len); // memcpy(recv_buffer_ , tmp_buffer_,recv_len);
      recv_buf_len_ = recv_buf_len_ + recv_len; //recv_buf_len = 16
      // 遍历校验
      for (int i = 0; (i + capacity) <= recv_buf_len_; i++) 
      {
        if (checkPacket(recv_buffer_ + i, capacity)) 
        {
          packet.copyFrom(recv_buffer_ + i);
          // std::cout<<"header:"<<std::hex<<static_cast<int>(*(recv_buffer_+i))<<std::endl;
          // std::cout<<"last:"<<std::hex<<static_cast<int>(*(recv_buffer_+i+capacity-1))<<std::endl;
          // 读取一帧后，更新接收缓存
          int k = 0;
          for (int j = i + capacity; j < recv_buf_len_; j++, k++) 
          {
            recv_buffer_[k] = recv_buffer_[j];
          }
          recv_buf_len_ = k;
          return true;
        }
      }
      // 表明断帧，或错误帧。
      PKA_WARN("serial_driver",
               "checkPacket() failed with recv_len:{}, frame head:{}, frame end:{}",
               recv_len,
               tmp_buffer_[0],
               tmp_buffer_[recv_len - 1]);
      return false;
    }
  }
  else 
  {
    PKA_ERROR("serial_driver", "transporter_->read() failed");
    // reconnect
    transporter_->close();
    transporter_->open();
    // 串口错误
    return false;
  }
}

using FixedPacketTool16 = FixedPacketTool<16>;
using FixedPacketTool32 = FixedPacketTool<32>;
using FixedPacketTool64 = FixedPacketTool<64>;

}  // namespace pka::serial_driver

#endif  // SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
