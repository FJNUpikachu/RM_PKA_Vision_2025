// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/pkaLoggerCenter.hpp"
#include "rm_utils/math/utils.hpp"

namespace pka::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  // TRACKING_ARMOR -> TRACKING_SPINNING 转速阈值
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  // TRACKING_SPINNING -> TRACKING_CENTER 转速阈值
  spinning_to_center_v_yaw_ = node->declare_parameter("solver.spinning_to_center_v_yaw", 90.0);
  // 预测延时
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  // 控制延时
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);

  // 新选板参数（yaml 填角度制，此处转弧度供内部使用）
  low_speed_fov_      = node->declare_parameter("solver.low_speed_fov",      23.0) * M_PI / 180.0;  // deg->rad
  lock_switch_thresh_ = node->declare_parameter("solver.lock_switch_thresh",   6.0) * M_PI / 180.0;  // deg->rad
  coming_angle_       = node->declare_parameter("solver.coming_angle",         57.0) * M_PI / 180.0;  // deg->rad
  leaving_angle_      = node->declare_parameter("solver.leaving_angle",        17.0) * M_PI / 180.0;  // deg->rad
  lock_id_ = -1;

  // 补偿器类型
  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  // 创建弹道补偿器
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  // 迭代次数
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  // 弹速
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  // 重力加速度
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  // 阻力
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  // 创建手动补偿器类
  manual_compensator_ = std::make_unique<ManualCompensator>();
  // 初始化参数angle_offset
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if(!manual_compensator_->updateMapFlow(angle_offset)) 
  {
    // 打印手动补偿器更新失败
    PKA_WARN("armor_solver", "Manual compensator update failed!");
  }

  // 初始化状态为跟踪装甲板
  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;
  lock_id_ = -1;

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters
  // 获得最新的参数
  try 
  {
    auto node = node_.lock();
    max_tracking_v_yaw_      = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    spinning_to_center_v_yaw_= node->get_parameter("solver.spinning_to_center_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    low_speed_fov_      = node->get_parameter("solver.low_speed_fov").as_double()      * M_PI / 180.0;
    lock_switch_thresh_ = node->get_parameter("solver.lock_switch_thresh").as_double() * M_PI / 180.0;
    coming_angle_       = node->get_parameter("solver.coming_angle").as_double()       * M_PI / 180.0;
    leaving_angle_      = node->get_parameter("solver.leaving_angle").as_double()      * M_PI / 180.0;
    // 重置智能指针，释放资源
    node.reset();
  } catch (const std::runtime_error &e) 
  {
    PKA_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  // 获得云台最近的roll、yaw和pitch
  try 
  {
    // 得到从gimbal_link到target的转换关系
    auto gimbal_tf = tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    // 
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } 
  catch (tf2::TransformException &ex) 
  {
    PKA_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Use flying time to approximately predict the position of target
  // 使用飞行时间大致预测目标的位置
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  // 计算飞行时间
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);

  double dt =(current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;

  // Choose the best armor to shoot
  // 选择最好的装甲板击打
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
    target_position, target_yaw, target.radius_1, target.radius_2, target.d_zc, target.d_za, target.armors_num);

  // 构建选板参数
  ArmorSelectParams select_params;
  select_params.solver_state      = static_cast<int>(state);
  select_params.low_speed_fov     = low_speed_fov_;
  select_params.lock_switch_thresh= lock_switch_thresh_;
  select_params.coming_angle      = coming_angle_;
  select_params.leaving_angle     = leaving_angle_;
  select_params.v_yaw             = target.v_yaw;

  int idx = selectBestArmor(armor_positions, target_position, select_params);
  auto chosen_armor_position = armor_positions.at(idx);
  if (chosen_armor_position.norm() < 0.1) 
  {
    throw std::runtime_error("No valid armor to shoot");
  }

  // Calculate yaw, pitch, distance
  // 计算yaw，pitch和distance
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
  double distance = chosen_armor_position.norm();

  // Initialize gimbal_cmd
  // 初始化云台指令
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

  switch (state) 
  {
    // ----------------------------------------------------------------
    // TRACKING_ARMOR：低速，单板瞄准
    // 升速条件：|v_yaw| > max_tracking_v_yaw_  -> TRACKING_SPINNING
    // ----------------------------------------------------------------
    case TRACKING_ARMOR: 
    {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) 
      {
        overflow_count_++;
      } 
      else 
      {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) 
      {
        state = TRACKING_SPINNING;
        PKA_DEBUG("armor_solver","TRACKING_ARMOR -> TRACKING_SPINNING");
        overflow_count_ = 0;
        lock_id_ = -1;
      }

      if (controller_delay_ != 0) 
      {
        target_position.x() += controller_delay_ * target.velocity.x;
        target_position.y() += controller_delay_ * target.velocity.y;  
        target_position.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position,
                                            target_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.d_zc,
                                            target.d_za,
                                            target.armors_num);
        select_params.solver_state = static_cast<int>(state);
        idx = selectBestArmor(armor_positions, target_position, select_params);
        chosen_armor_position = armor_positions.at(idx);
        gimbal_cmd.distance = chosen_armor_position.norm();
        if (chosen_armor_position.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
        }
        calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
      }
      break;
    }

    // ----------------------------------------------------------------
    // TRACKING_SPINNING：中速小陀螺，选"即将到正面"的板
    // 降速条件：|v_yaw| < max_tracking_v_yaw_  -> TRACKING_ARMOR
    // 升速条件：|v_yaw| > spinning_to_center_v_yaw_ -> TRACKING_CENTER
    // ----------------------------------------------------------------
    case TRACKING_SPINNING:
    {
      if (std::abs(target.v_yaw) > spinning_to_center_v_yaw_)
      {
        // 转速继续升高，进入瞄准中心模式
        overflow_count_++;
        if (overflow_count_ > transfer_thresh_)
        {
          state = TRACKING_CENTER;
          PKA_DEBUG("armor_solver","TRACKING_SPINNING -> TRACKING_CENTER");
          overflow_count_ = 0;
        }
      }
      else if (std::abs(target.v_yaw) < max_tracking_v_yaw_)
      {
        // 转速下降，回到单板跟踪
        overflow_count_++;
        if (overflow_count_ > transfer_thresh_)
        {
          state = TRACKING_ARMOR;
          PKA_DEBUG("armor_solver","TRACKING_SPINNING -> TRACKING_ARMOR");
          overflow_count_ = 0;
          lock_id_ = -1;
        }
      }
      else
      {
        // 转速稳定在中速区间，维持当前状态
        overflow_count_ = 0;
      }
      break;
    }

    // ----------------------------------------------------------------
    // TRACKING_CENTER：高速，瞄准中心持续开火
    // 降速条件：|v_yaw| < spinning_to_center_v_yaw_ -> TRACKING_SPINNING
    // ----------------------------------------------------------------
    case TRACKING_CENTER: 
    {
      if (std::abs(target.v_yaw) < spinning_to_center_v_yaw_) 
      {
        overflow_count_++;
        if (overflow_count_ > transfer_thresh_)
        {
          state = TRACKING_SPINNING;
          PKA_DEBUG("armor_solver","TRACKING_CENTER -> TRACKING_SPINNING");
          overflow_count_ = 0;
        }
      } 
      else 
      {
        overflow_count_ = 0;
      }
      // 瞄准中心，持续开火
      gimbal_cmd.fire_advice = true;
      calcYawAndPitch(target_position, rpy_, yaw, pitch);
      break;
    }
  }

  // Compensate angle by angle_offset_map
  // 按angle_offset_map补偿角度
  // target_position.head(2).norm() = sqrt(pow(target_position.x,2)+pow(target_position.y,2));
  auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);


  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;  
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) 
  {
    PKA_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  // 计算每一个装甲板那的位置
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) 
  {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) 
    {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } 
    else 
    {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

// 选择最好的装甲板进行击打
int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const ArmorSelectParams &params) const noexcept {
  const size_t armors_num = armor_positions.size();
  if (armors_num == 0) { return 0; }

  // 目标中心相对odom的方位角
  double center_yaw = std::atan2(target_center.y(), target_center.x());

  // 计算每块装甲板相对中心方位角的偏差
  std::vector<double> delta(armors_num);
  for (size_t i = 0; i < armors_num; i++) {
    double armor_yaw = std::atan2(armor_positions[i].y(), armor_positions[i].x());
    delta[i] = limitRad(armor_yaw - center_yaw);
  }

  // 选最近（delta绝对值最小）的装甲板
  auto selectNearest = [&]() -> int {
    int best = 0;
    for (size_t i = 1; i < armors_num; i++) {
      if (std::abs(delta[i]) < std::abs(delta[best])) best = static_cast<int>(i);
    }
    return best;
  };

  // TRACKING_CENTER：选板结果不影响实际瞄准点，直接选最近
  if (params.solver_state == static_cast<int>(TRACKING_CENTER)) {
    return selectNearest();
  }

  // TRACKING_ARMOR：低速模式，锁定+滞后防横跳
  if (params.solver_state == static_cast<int>(TRACKING_ARMOR)) {
    // 找出在视野半角内的候选装甲板
    std::vector<int> candidates;
    for (size_t i = 0; i < armors_num; i++) {
      if (std::abs(delta[i]) <= params.low_speed_fov)
        candidates.push_back(static_cast<int>(i));
    }
    // 视野内无候选，清除锁定，选最近
    if (candidates.empty()) {
      lock_id_ = -1;
      return selectNearest();
    }
    // 视野内只有一块，直接锁定
    if (candidates.size() == 1) {
      lock_id_ = -1;
      return candidates[0];
    }
    // 视野内有两块，带滞后的锁定逻辑
    int id0 = candidates[0], id1 = candidates[1];
    if (lock_id_ != id0 && lock_id_ != id1) {
      // 初次进入或上次锁定不在候选中，选更近的
      lock_id_ = (std::abs(delta[id0]) < std::abs(delta[id1])) ? id0 : id1;
    }
    int other = (lock_id_ == id0) ? id1 : id0;
    // 只有当另一块比当前锁定明显更近时才切换（滞后防横跳）
    if (std::abs(delta[other]) < std::abs(delta[lock_id_]) - params.lock_switch_thresh) {
      lock_id_ = other;
    }
    return lock_id_;
  }

  double v_yaw = params.v_yaw;
  for (size_t i = 0; i < armors_num; i++) {
    if (std::abs(delta[i]) > params.coming_angle) continue;
    if (v_yaw > 0 && delta[i] > 0 && delta[i] < params.leaving_angle) {
      return static_cast<int>(i);
    }
    if (v_yaw < 0 && delta[i] < 0 && delta[i] > -params.leaving_angle) {
      return static_cast<int>(i);
    }
  }
  return selectNearest();
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) 
  {
    // 进行角度的迭代
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace pka::auto_aim