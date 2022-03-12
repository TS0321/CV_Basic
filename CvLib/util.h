#pragma once

#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class Util
{
public:
	Util() {};
	~Util() {};
public:
	static std::vector<std::string> loadDirFiles(const std::string dir_path);
	static std::vector<cv::Point3f> create_board3dPts(int width, int height, double margin);
};