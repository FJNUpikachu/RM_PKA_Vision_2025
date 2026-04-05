# armor_detector

订阅相机参数及图像流进行装甲板的识别并解算三维位置，输出识别到的装甲板在输入frame下的三维位置 (一般是以相机光心为原点的相机坐标系)

# 别开PCA！别开PCA！别开PCA！它是陷阱！

## pka::ArmorDetectorNode

装甲板识别节点

### 发布话题 

*  `armor_detector/armors` (`rm_interfaces/msg/Armors`) - 识别到的装甲板信息
*  `armor_detector/debug_lights` (`rm_interfaces/msg/DebugLights`) - Debug灯条信息
*  `armor_detector/debug_armors` (`rm_interfaces/msg/DebugArmors`) - Debug装甲板信息
*  `armor_detector/result_img` (`sensor_msgs/msg/Image`) - 识别结果可视化图像
*  `armor_detector/binary_img` (`sensor_msgs/msg/Image`) - 二值化图像
*  `armor_detector/number_img` (`sensor_msgs/msg/Image`) - 数字识别roi

### 订阅话题

*  `image_raw` (`sensor_msgs/msg/Image`) - 相机图像
*  `camera_info` (`sensor_msgs/msg/CameraInfo`) - 相机参数

### 服务

*  `armor_detector/set_mode` (`rm_interfaces/srv/SetMode`) - 设置模式

### 参数 

* `debug` (`bool`, default: false) - 是否开启调试模式
* `classify_threshold` (`double`, default: 0.8) - 数字分类阈值
* `ignore_class` (`vector<string>`, default: ["negativie"]) - 跳过的类别
* `binary_thres` (`int`, default: 100) - 二值化阈值
* `light.min_ratio` (`double`, default: 0.08) - 灯条最小长宽比
* `light.max_ratio` (`double`, default: 0.4) - 灯条最大长宽比
* `light.max_angle` (`double`, default: 40) - 灯条最大倾斜角度
* `light.color_diff_thresh` (`int`, default: 25) - 灯条颜色差异阈值`
* `armor.min_light_ratio` (`double`, default: 0.6) - 装甲板最小长宽比
* `armor.min_small_center_distance` (`double`, default: 0.8) - 小装甲板最小中心距离长宽比
* `armor.max_small_center_distance` (`double`, default: 3.2) - 小装甲板最大中心距离长宽比
* `armor.min_large_center_distance` (`double`, default: 1.8) - 大装甲板最小中心距离长宽比
* `armor.max_large_center_distance` (`double`, default: 6.4) - 大装甲板最大中心距离长宽比
* `armor.max_angle` (`double`, default: 35.0) - 装甲板最大倾斜角度

### 日志
#### 2026.4.3
- 尝试使用旧代码逻辑在Node中通过tf2获取旋转矩阵并在ArmorPoseEstimator中进行优化，减少走tf2的次数，提升性能

#### 2026.4.4
- 完成所有代码更改
- 将代码中的迭代法更换为梯度下降求最值，以换取更高效率

#### 2026.4.5
- 通过基础功能测试，但发现优化后仍有yaw精度问题，推测是重投影误差精度问题，遂更改代价函数，通过平方和的方式拉大误差，更改后精度有显著提升
- 梯度下降仍有一定性能缺陷，遂更改为三分法以获取最优性能，进一步提升效率

### 后续优化方案
后续将换为强先验性质的，基于LM方法的BA优化，以获取最优姿态，或更改代价函数，除四点以外将边和角度加入代价计算中，加以比例约束以获取更好的优化