# Pikachu战队2025 Vision Project

 本项目为福建师范大学Pikachu战队2025赛季视觉主项目框架，本框架引用中南大学FYT战队2024赛季视觉开源作为框架进行开发。


## 一、项目结构

```
.
│
├── rm_bringup (启动及参数文件)
│
├── rm_robot_description (机器人urdf文件，坐标系的定义)
│
├── rm_interfaces (自定义msg、srv)
│
├── rm_hardware_driver
│   │
│   ├── rm_camera_driver (相机驱动)（大华新相机包）
│   │
│   └── rm_serial_driver (串口驱动)
│
├── rm_auto_aim (自瞄算法)
│
│
├── rm_utils (工具包) 
│   ├── math (包括PnP解算、弹道补偿等)
│   │
│   └── logger (日志库)
│
└── rm_upstart (自启动配置)
```

## 二、环境

**注：安装环境时请搞清楚各个库之间的相互依赖关系，请按顺序安装，如Ceres库作为Sophus的库依赖，而Sophus库则作为G2O库的依赖!**

### 1. 基础
- Ubuntu 22.04
- ROS2 Humble
- 大华相机驱动/海康相机驱动

### 2. 自瞄 

- 相机包驱动安装（先cd到相机驱动所在的包再打开终端执行以下指令）（！必须安装相机驱动 ！）

  ```bash
  chmod +x MVviewer_Ver2.3.1_Linux_x86_Build20210926.run 
  sudo ./MVviewer_Ver2.3.1_Linux_x86_Build20210926.run
  ```

- fmt库
  ```bash
  sudo apt install libfmt-dev
  ```
  
- 安装Ceres库相关依赖
  
   ```bash
   sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
   ```

   通过包管理器安装的依赖仍有不全，需补充下载abseil-cpp和googletest库
   
   ```bash
   wget https://github.com/google/googletest/releases/download/v1.16.0/googletest-1.16.0.tar.gz
   tar -xzvf ./googletest-1.16.0.tar.gz
   cd ./googletest-1.16.tar.gz
   mkdir build && cd build 
   cmake ..
   make -j1
   sudo make install
   ```

   ```bash
   wget https://github.com/abseil/abseil-cpp/releases/download/20250127.1/abseil-cpp-20250127.1.tar.gz
   tar -xzvf abseil-cpp-20250127.1.tar.gz
   cd ./abseil-cpp-20250127.1
   mkdir build && cd build 
   cmake ..
   make -j1
   sudo make install
   ## 注意abseil不能直接clone项目，必须下载release中的文件，否则会导致make install时找不到版本头文件
   ```
   
- Ceres库 (能量机关曲线拟合)
  
   ```bash
   sudo apt install libceres-dev
   ```
   
- Ceres库（利用源码编译安装）（本Ceres库需要自行去寻找源码编译包，注：虽然已经之间安装了Ceres库，但仍存在找不到此库的一部分依赖，故推荐使用源码编译安装！）
  
   ```bash
   cd ceres-solver-2.2.0
   mkdir build && cd build 
   cmake ..
   make -j1
   sudo make install
   ```
   
   
   
- Sophus库 (G2O库依赖)
  
   **注：git下来的Sophus库的CMakeLists.txt中的cmake_minimum_required(VERSION xx.xx)的版本要求可能会高于系统的版本，将xx.xx改成符合系统的版本即可**
   
   ```bash
   git clone https://github.com/strasdat/Sophus
   cd Sophus
   mkdir build && cd build
   cmake ..
   make -j1
   sudo make install
   ```
   
   
   
- G2O库 (优化装甲板Yaw角度)
  
    ```bash
    sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
    git clone https://github.com/RainerKuemmerle/g2o
    cd g2o
    mkdir build && cd build
    cmake ..
    make -j1
    sudo make install
    ```
### 3. 能量机关(后续更新这步暂时不用)
- OpenVINO库 (能量机关识别)
  
   参考[OpenVINO官方文档](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)，建议同时安装GPU相关依赖

- Ceres库 (能量机关曲线拟合)
    ```bash
    sudo apt install libceres-dev
    ```

### 5. 其他

本文档中可能有缺漏，如有，可以用`rosdep`安装剩下依赖

```bash
rosdep install --from-paths src --ignore-src -r -y
```

**注：在进行完本指令之后如果直接编译运行会出现serial库的一些问题需再执行一句指令便可成功编译和运行**

```bash
sudo apt remove libasio-dev
```


## 三、编译与运行

**修改rm_bringup/config/launch_params.yaml，选择需要启动的功能，将serial模式设置成虚拟下位机（不懂看注释！），相机模式设置成相机模式，其他不动即可，成功编译运行会出现no camera是正常的！**

```bash
# 编译
colcon build --symlink-install --parallel-workers 2 #本仓库包含的功能包过多，建议限制同时编译的线程数
# 手动运行
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```

**测试是否启动成功**
```bash
ros2 topic list
# 正常时会出现很多条，如果话题数量很少则说明未启动成功
# armor_detector/...
# armor_solver/...
# ...
```

**注：后续会更新的日志包**

默认日志和内录视频路径为`~/fyt2024-log/`

> 我们的日志库是用fmt搓的，不使用ros2的日志库


## 四、自启动

- 编译程序后，进入rm_upstart文件夹

```bash
cd RM_PKA_...(对应文件夹)
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务
  
```bash
cd rm_upstart
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出
```
查看screen.output或~/fyt2024-log下的日志
```

- 开启服务
```bash
systemctl start rm
```

- 开启自启动
```bash
systemctl enable rm
```
**注：建议此时重启pc进行测试**
```bash
# 重新启动
sudo reboot
# 关闭系统
sudo poweroff
```

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```


## 五、调车步骤

- 查看相机图片：使用rviz2
```bash
rviz2
```
->订阅result_image话题


- 使用foxglove看曲线和话题数据

在PKA自瞄包下先启动，然后在同目录下再开一个终端

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

此时打开foxglove：打开连接->ws://localhost:xxxx

> 获取hostname

```bash
# 获取ip地址
hostname -i
# 禁用防火墙
sudo ufw disable
```
ws://ip地址:xxxx
->点击topic可看到各类话题即为成功


## 六、调车建议
调rm_bringup/config内参数

1.相机参数：改`camera_driver_params.yaml`内exposure_time和gain

2.EKF：`armor_solver_params.yaml`
订阅target和measurement的x y z曲线 平移装甲板看曲线收敛情况
一条平滑 一条有毛刺 两条曲线需要基本重合

> 想要云台响应过快 ：调小测量噪声协方差矩阵 (R小)
> 云台太抖动      ：调小过程噪声协方差矩阵 (Q小)

3.关于选板：`armor_solver_params.yaml`

max_match_distance: 0.5
两帧间目标可匹配的最大距离

max_match_yaw_diff: 1.0
两帧间目标同一块装甲板可匹配的最大yaw角差（大于这个值则认为装甲板发生跳变）

tracking_thres: 2
`DETECTING` 状态进入 `TRACKING` 状态需要连续识别到的帧数

lost_time_thres: 1.0
`TRACKING` 状态进入 `LOST` 状态需要连续丢失的时间（s）

4.串口时间戳：`serial_driver_params.yaml`
时间戳补偿 timestamp_offset: 0.006


### 相机标定
```bash
ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.015 image:=/image_raw
```


## 维护者及开源许可证

> 赛季结束开源

Maintainer : FYT Vision Group

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 致谢

感谢这个赛季视觉组的每一个成员的付出，感谢以下开源项目：

- [rm_vision](https://gitlab.com/rm_vision) rv是本项目的基础，提供了一套可参考的，规范、易用、高效的视觉算法框架
- [rmoss](https://github.com/robomaster-oss/rmoss_core) rmoss项目为RoboMaster提供通用基础功能模块包，本项目的串口驱动模块基于rmoss_base进行开发
- [沈阳航空航天大学TUP战队2022赛季步兵视觉开源](https://github.com/tup-robomaster/TUP-InfantryVision-2022) 为本项目的能量机关识别与预测算法提供了参考
- [沈阳航空航天大学YOLOX关键点检测模型](https://github.com/tup-robomaster/TUP-NN-Train-2) 提供了本项目能量机关识别模型训练代码
- [四川大学OpenVINO异步推理代码](https://github.com/Ericsii/rm_vision-OpenVINO) 提供了本项目能量机关识别模型部署的代码
- [上海交通大学自适应扩展卡尔曼滤波](https://github.com/julyfun/rm.cv.fans/tree/main) 使用Ceres自动微分功能，自动计算Jacobian矩阵


## 更新日志（FYT）

- 参考上交开源，实现了EKF的自动求Jacobian矩阵
- 增加了粒子滤波器，为状态估计提供新的选择
- 修复了打符崩溃的问题（OpenVINO在推理时不能创建新的InferRequest，通过互斥锁解决）
- 将自瞄解算修改为定时器回调，固定解算的频率
- 增加手动补偿器ManualCompensator
- 重写PnP选解逻辑
- 修改了BA优化的代码，抽象出新的类ArmorPoseEstimator
