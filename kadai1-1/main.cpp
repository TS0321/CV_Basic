#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "../CvLib/util.h"
//#define DEBUG

//calibrationボードのdotの三次元位置を算出する関数
std::vector<cv::Point3f> create_board3dPts(int width, int height, double margin)
{
	std::vector<cv::Point3f> calib3dPts;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			cv::Point3f pt = cv::Point3f(x * margin, y * margin, 0);
			calib3dPts.push_back(pt);
		}
	}
	return calib3dPts;
}

int main(void)
{
	//calib画像データの読み込み
	std::vector<std::string> file_names;
	file_names = Util::loadDirFiles("../../data/calib");
	std::vector<cv::Mat> calib_imgs;
	for (const auto& file_name : file_names)
	{
		cv::Mat img = cv::imread(file_name);
#ifdef DEBUG
		//読み込んだ画像データの表示
		//cv::imshow("img", img);
		//cv::waitKey(0);
#endif
		calib_imgs.push_back(img);
	}

	std::vector<cv::Point3f> board_3dPts;
	int width = 12;		//dotのx軸方向の数
	int height = 8;		//dotのy軸方向の数
	double margin = 20.0;	//dotとdotの間の距離
	board_3dPts = create_board3dPts(width, height, margin);
#ifdef DEBUG
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			std::cout << calib_3dPts[y * width + x] << ", ";
		}
		std::cout << std::endl;
	}
#endif

	std::vector<std::vector<cv::Point3f>> calib_3dPts;
	std::vector<std::vector<cv::Point2f>> calib_2dPts;
	
	for (const auto& img : calib_imgs)
	{
		std::vector<cv::Point2f> board_2dPts;
  		bool isFound = cv::findCirclesGrid(img, cv::Size(width, height), board_2dPts, cv::CALIB_CB_SYMMETRIC_GRID);
#ifdef DEBUG
		cv::drawChessboardCorners(img, cv::Size(width, height), board_2dPts, isFound);
		cv::imshow("find_img", img);
		cv::waitKey(0);
#endif
		calib_2dPts.push_back(board_2dPts);
		calib_3dPts.push_back(board_3dPts);
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	cv::calibrateCamera(calib_3dPts, calib_2dPts, calib_imgs[0].size(), cameraMatrix, distCoeffs, rvecs, tvecs);

	//パラメータの書き出し
	YAML::Node cameraParam;
	try
	{
		cameraParam["width"] = calib_imgs[0].cols;
		cameraParam["height"] = calib_imgs[0].rows;
		cameraParam["fx"] = cameraMatrix.at<double>(0, 0);
		cameraParam["fy"] = cameraMatrix.at<double>(1, 1);
		cameraParam["cx"] = cameraMatrix.at<double>(0, 2);
		cameraParam["cy"] = cameraMatrix.at<double>(1, 2);
		cameraParam["k1"] = distCoeffs.at<double>(0);
		cameraParam["k2"] = distCoeffs.at<double>(1);
		cameraParam["p1"] = distCoeffs.at<double>(2);
		cameraParam["p2"] = distCoeffs.at<double>(3);
		cameraParam["k3"] = distCoeffs.at<double>(4);
		
		YAML::Emitter out;
		out << cameraParam;
		const std::string calib_fileName = "CameraParameter.yaml";
		std::ofstream file("../../CamParam/" + calib_fileName);
		file << out.c_str();
		file.close();

		std::cout << "camera parameter is generated. " << std::endl;
	}
	catch (YAML::Exception& e)
	{
		std::cerr << e.what() << std::endl;
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}


	return 0;
}