#ifndef IO__TALOS_HPP
#define IO__TALOS_HPP

#include <chrono>
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>

namespace io::talos {

bool connected();

bool read_latest_image(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

bool read_latest_pose(Eigen::Quaterniond & q, Eigen::Vector3d & position);

bool write_gimbal_cmd(double yaw_deg, double pitch_deg, double distance_m, bool fire_advice);

}  // namespace io::talos

#endif  // IO__TALOS_HPP
