#pragma once
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

class CameraParam
{
public:
	CameraParam() {};
	CameraParam(int width, int height, double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3)
		: width(width), height(height), fx(fx), fy(fy), cx(cx), cy(cy), k1(k1), k2(k2), p1(p1), p2(p2), k3(k3){};
	~CameraParam() {};

public:
	void loadParam(const std::string file_path);
	cv::Mat getCvCamParam() const;
	cv::Mat getCvDistCoeffs() const;

private:
	int width;
	int height;
	double fx;
	double fy;
	double cx;
	double cy;
	double k1;
	double k2;
	double p1;
	double p2;
	double k3;
};