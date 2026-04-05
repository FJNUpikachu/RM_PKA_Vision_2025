#include <armor_detector/armor_pose_estimator.hpp>

namespace pka::auto_aim {

ArmorPoseEstimator::ArmorPoseEstimator(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
    this->camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info->k.data())).clone();
    this->dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(camera_info->d.data())).clone();

    // fill the points
    this->obj_points_map_[ArmorType::SMALL] = Armor::buildObjectPoints<cv::Point3f>
        (SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT);
    this->obj_points_map_[ArmorType::LARGE] = Armor::buildObjectPoints<cv::Point3f>
        (LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT);
}

void ArmorPoseEstimator::updateTransforms
(const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Matrix3d& R_gimbal2odom) {
    this->R_camera2gimbal_ = R_camera2gimbal;
    this->R_gimbal2odom_ = R_gimbal2odom;
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d &R) {
    // Transform to camera frame
    Eigen::Quaterniond q(R);
    // Get armor yaw
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    Eigen::Vector3d rpy;
    tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
    return rpy;
}

Eigen::Quaterniond ArmorPoseEstimator::rpy2Quaternion(const Eigen::Vector3d& rpy)
{
    return Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

Eigen::Matrix3d ArmorPoseEstimator::rpy2RotationMatrix(const Eigen::Vector3d& rpy) {
    return this->rpy2Quaternion(rpy).toRotationMatrix();
}

double ArmorPoseEstimator::shortest_angular_distance(double angle) {
    while (angle > CV_PI) angle -= 2 * CV_PI;
    while (angle <= -CV_PI) angle += 2 * CV_PI;
    return angle;
}

double ArmorPoseEstimator::trisectionSearch
(const Armor& armor, const cv::Mat& tvec, double l, double r, double eps) {
    while (r - l > eps) {
        double m1 = l + (r - l) / 3.0f;
        double m2 = r - (r - l) / 3.0f;

        double f1 = this->calculateReprojectionError(armor, tvec, m1);
        double f2 = this->calculateReprojectionError(armor, tvec, m2);

        if (f1 < f2) r = m2;
        else l = m1;
    }

    return (l + r) / 2.0f;
}

float ArmorPoseEstimator::cal2CenterDist(const cv::Point2f& image_point) {
    auto cam_center_x = this->camera_matrix_.at<double>(0, 2);
    auto cam_center_y = this->camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cam_center_x, cam_center_y));
}

Eigen::Matrix3d ArmorPoseEstimator::calculateAsumMatrix(double yaw, double pitch) {
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    const Eigen::Matrix3d result {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
        {-sin_pitch, 0, cos_pitch}
    };

    return result;
}

double ArmorPoseEstimator::calculateReprojectionError
(const Armor& armor, const cv::Mat& cam_tvec, double yaw) {
    // reprojected points
    std::vector<cv::Point2f> reprojected_points(4);

    // set obj points
    const auto& obj_points = armor.type == ArmorType::SMALL ? 
        this->obj_points_map_[ArmorType::SMALL] : this->obj_points_map_[ArmorType::LARGE];

    // rvec
    double pitch = armor.number == "outpost" ? 
        -15.0 / 180.0 * CV_PI : 15.0 / 180.0 * CV_PI;
    auto R_armor2odom = this->calculateAsumMatrix(yaw, pitch);
    Eigen::Matrix3d R_armor2camera =
        this->R_camera2gimbal_.transpose() * this->R_gimbal2odom_.transpose() * R_armor2odom;
    cv::Mat rvec, rmat;
    cv::eigen2cv(R_armor2camera, rmat);
    cv::Rodrigues(rmat, rvec);

    // reproject
    cv::projectPoints(obj_points, rvec, cam_tvec, this->camera_matrix_, this->dist_coeffs_, 
        reprojected_points);

    // calculate error
    double error = 0.0f;
    for (int i = 0; i < 4; i++) {
        auto d = armor.landmarks()[i] - reprojected_points[i];
        error += d.x * d.x + d.y * d.y;
    }

    return error;
}

void ArmorPoseEstimator::optimizeYaw(Armor& armor, cv::Mat& tvec) {
    auto gimbal_rpy = this->rotationMatrixToRPY(this->R_gimbal2odom_);

    double del = (this->search_range / 2.0f) / 180.0f * CV_PI;
    double l = gimbal_rpy[2] - del;
    double r = gimbal_rpy[2] + del;
    double eps = 0.1f / 180.0f * CV_PI;
    
    double best_yaw = this->shortest_angular_distance(this->trisectionSearch(armor, tvec, l, r, eps));

    armor.rpy_in_odom[2] = best_yaw;
    Eigen::Matrix3d R_armor2camera_updated = this->R_camera2gimbal_.transpose() * 
        this->R_gimbal2odom_.transpose() * this->rpy2RotationMatrix(armor.rpy_in_odom);
    armor.rpy_in_camera = this->rotationMatrixToRPY(R_armor2camera_updated);
}

std::vector<rm_interfaces::msg::Armor> 
ArmorPoseEstimator::doPoseExtract(std::vector<Armor>& armors) {
    std::vector<rm_interfaces::msg::Armor> armor_msgs;

    for (auto& armor : armors) {
        // init msg
        rm_interfaces::msg::Armor armor_msg;

        // obj points
        const auto& object_points = armor.type == ArmorType::SMALL ? 
            this->obj_points_map_[ArmorType::SMALL] : this->obj_points_map_[ArmorType::LARGE];
        
        // transform info
        cv::Mat rvec, tvec;

        // solve pnp
        auto ok = cv::solvePnP(object_points, armor.landmarks(),
            this->camera_matrix_, this->dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if (!ok) continue;

        // process t relationship
        Eigen::Vector3d xyz_in_camera;
        cv::cv2eigen(tvec, xyz_in_camera);

        // process r relationship
        cv::Mat rmat;
        Eigen::Matrix3d R_armor2camera;
        cv::Rodrigues(rvec, rmat);
        cv::cv2eigen(rmat, R_armor2camera);
        armor.rpy_in_camera = this->rotationMatrixToRPY(R_armor2camera);
        Eigen::Matrix3d R_armor2odom = this->R_gimbal2odom_ * this->R_camera2gimbal_ * R_armor2camera;
        armor.rpy_in_odom = this->rotationMatrixToRPY(R_armor2odom);

        // optimize yaw
        if (this->optOption) {
            this->optimizeYaw(armor, tvec);
        }

        // rotation's q transformation
        auto armor_q = this->rpy2Quaternion(armor.rpy_in_camera);

        // msg build
        armor_msg.number = armor.number;
        armor_msg.type = armorTypeToString(armor.type);
        armor_msg.distance_to_image_center = this->cal2CenterDist(armor.center);
        armor_msg.pose.position.x = xyz_in_camera[0];
        armor_msg.pose.position.y = xyz_in_camera[1];
        armor_msg.pose.position.z = xyz_in_camera[2];
        armor_msg.pose.orientation.w = armor_q.w();
        armor_msg.pose.orientation.x = armor_q.x();
        armor_msg.pose.orientation.y = armor_q.y();
        armor_msg.pose.orientation.z = armor_q.z();

        // push into vector
        armor_msgs.emplace_back(armor_msg);
    }

    return armor_msgs;
}

}