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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
// ros2
#include <tf2_ros/buffer.h>
#include <angles/angles.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"

namespace pka::auto_aim {

// -----------------------------------------------------------------------
// 选板参数结构体
// -----------------------------------------------------------------------
struct ArmorSelectParams {
  int    solver_state;        // 当前 Solver 状态（0=TRACKING_ARMOR, 1=TRACKING_CENTER, 2=TRACKING_SPINNING）
  double low_speed_fov;       // TRACKING_ARMOR  视野半角阈值 (rad)，视野内有哪些板可选
  double lock_switch_thresh;  // TRACKING_ARMOR  锁定切换滞后阈值 (rad)，防横跳
  double coming_angle;        // TRACKING_SPINNING 进入视野角度阈值 (rad)
  double leaving_angle;       // TRACKING_SPINNING 离开视野角度阈值 (rad)
  double v_yaw = 0.0;         // 目标角速度 (rad/s)，供 TRACKING_SPINNING 使用
};

// Solver class used to solve the gimbal command from tracked target
// Solver类用于求解云台跟踪目标的指令
class Solver {
public:
  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  // explicit Solver(std::string trajectory_compensator_type, float max_tracking_v_yaw);
  ~Solver() = default;

  // Solve the gimbal command from tracked target
  // 解算跟踪目标的 gimbal 命令
  // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
  // 如果从odom到gimbal_link的转换不可用则抛出异常
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);


  // 状态枚举 
  /*
  跟踪装甲板：0
  跟踪机器人中心：1
  小陀螺模式：2
  */
  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1, TRACKING_SPINNING = 2 } state;

  std::vector<std::pair<double, double>> getTrajectory() const noexcept; 

private:
  // Get the armor positions from the target robot
  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 const double yaw,
                                                 const double r1,
                                                 const double r2,
                                                 const double d_zc,
                                                 const double d_za,
                                                 const size_t armors_num) const noexcept;

  // Select the best armor to shoot
  // Return: selected idx in {0, 1, ..., armors_num - 1}
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      const ArmorSelectParams &params) const noexcept;

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       const std::array<double, 3> rpy,
                       double &yaw,
                       double &pitch) const noexcept;

  bool isOnTarget(const double cur_yaw,
                  const double cur_pitch,
                  const double target_yaw,
                  const double target_pitch,
                  const double distance) const noexcept;

  // Normalize angle to [-pi, pi]
  static double limitRad(double rad) noexcept {
    while (rad >  M_PI) rad -= 2 * M_PI;
    while (rad < -M_PI) rad += 2 * M_PI;
    return rad;
  }

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;

  std::array<double, 3> rpy_;

  double prediction_delay_;
  double controller_delay_;

  double shooting_range_w_;
  double shooting_range_h_;

  double max_tracking_v_yaw_;       // TRACKING_ARMOR  -> TRACKING_SPINNING 转速阈值 (rad/s)
  double spinning_to_center_v_yaw_; // TRACKING_SPINNING -> TRACKING_CENTER  转速阈值 (rad/s)
  int overflow_count_;
  int transfer_thresh_;

  // 新选板参数
  double low_speed_fov_;       // TRACKING_ARMOR 视野半角阈值 (rad)
  double lock_switch_thresh_;  // TRACKING_ARMOR 锁定切换滞后阈值 (rad)
  double coming_angle_;        // TRACKING_SPINNING 进入视野角度阈值 (rad)
  double leaving_angle_;       // TRACKING_SPINNING 离开视野角度阈值 (rad)

  // 锁定装甲板id，用于防横跳（-1表示未锁定）
  mutable int lock_id_;

  std::weak_ptr<rclcpp::Node> node_;
};
}  // namespace pka::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_