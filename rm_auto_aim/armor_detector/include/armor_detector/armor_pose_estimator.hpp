#ifndef ARMOR_DETECTOR_POSE_ESTIMATOR_HPP_
#define ARMOR_DETECTOR_POSE_ESTIMATOR_HPP_

// std
#include <unordered_map>
// eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
// ros

#include <sensor_msgs/msg/camera_info.hpp>
// tf2
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
// files
#include <armor_detector/types.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <rm_utils/pkaLoggerCenter.hpp>

namespace pka::auto_aim {

class ArmorPoseEstimator {
private:
    // cam info
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // transform matrixs
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2odom_;

    // switch
    bool optOption;

    // search range
    double search_range = 140.0;

    // obj points map
    std::unordered_map<ArmorType, std::vector<cv::Point3f>> obj_points_map_;

    // calculate the dist to center
    float cal2CenterDist(const cv::Point2f& image_point);
    
    // tool func
    Eigen::Matrix3d calculateAsumMatrix(double yaw, double pitch);
    Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d &R);
    Eigen::Quaterniond rpy2Quaternion(const Eigen::Vector3d& rpy);
    Eigen::Matrix3d rpy2RotationMatrix(const Eigen::Vector3d& rpy);
    double trisectionSearch(const Armor& armor, const cv::Mat& tvec, double l, double r, double eps);
    double shortest_angular_distance(double angle);

    // core
    void optimizeYaw(Armor& armor, cv::Mat& tvec);
    double calculateReprojectionError(const Armor& armor, const cv::Mat& cam_tvec, double yaw);

public:
    ArmorPoseEstimator() = delete;
    ArmorPoseEstimator(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

    std::vector<rm_interfaces::msg::Armor> doPoseExtract(std::vector<Armor>& armors);

    void updateTransforms(const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Matrix3d& R_gimbal2odom);

    void enableOpt(bool choice) { this->optOption = choice; }
    void setSearchRange(double search_range) { this->search_range = search_range; }
};

}

#endif