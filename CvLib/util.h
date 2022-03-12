#pragma once

#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "cameraParam.h"

class Util
{
public:
	Util() {};
	~Util() {};
public:
	static std::vector<std::string> loadDirFiles(const std::string dir_path);
	static std::vector<cv::Point3f> create_board3dPts(int width, int height, double margin);
	static void solvePnP(const std::vector<Eigen::Vector3d>& obj_pts, const std::vector<Eigen::Vector2d>& img_pts, const CameraParam& cparam, Eigen::Isometry3d& pose);
	static bool optimizePose(const std::vector<Eigen::Vector3d>& obj_pts, const std::vector<Eigen::Vector2d>& img_pts, const CameraParam& cparam, Eigen::Isometry3d& pose, const int iteration_num, const int error_thresh);
};
bool calcPose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, CameraParam& cparam, Eigen::Isometry3d& current_pose);
bool optimizePose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, CameraParam& cparam, Eigen::Isometry3d& pose, const int iteration_num = 10, const double error_thresh = 0.001);

