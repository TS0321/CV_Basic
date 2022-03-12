#pragma once
#include <iostream>
#include <string>

class CameraParam
{
public:
	CameraParam() {};
	~CameraParam() {};

public:
	void loadParam(std::string file_path);

private:
	int widht;
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