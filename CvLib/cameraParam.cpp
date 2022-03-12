#include "cameraParam.h"
#include <yaml-cpp/yaml.h>

void CameraParam::loadParam(const std::string file_path)
{
	try {
		YAML::Node camParam = YAML::LoadFile(file_path);
	    width = camParam["width"].as<int>();
	    height = camParam["height"].as<int>();
	    fx = camParam["fx"].as<double>();
	    fy = camParam["fy"].as<double>();
	    cx = camParam["cx"].as<double>();
	    cy = camParam["cy"].as<double>();
	    k1 = camParam["k1"].as<double>();
	    k2 = camParam["k2"].as<double>();
	    p1 = camParam["p1"].as<double>();
	    p2 = camParam["p2"].as<double>();
	    k3 = camParam["k3"].as<double>();
	}
	catch (YAML::Exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return;
}

cv::Mat CameraParam::getCvCamParam() const
{
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)
		<< fx, 0, cx,
		0, fy, cy,
		0, 0, 1);
	
	return cameraMatrix;
}

cv::Mat CameraParam::getCvDistCoeffs() const
{
	cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

	return distCoeffs;
}


