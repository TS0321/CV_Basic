#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "../../CvLib/cameraParam.h"
#include "../../CvLib/util.h"

class Test
{
public:
	Test() {};
	~Test() {};
private:
	CameraParam cparam;
public:
	void execute()
	{
		//�J�����p�����[�^�̓ǂݍ���
		const std::string camParam_path = "../../CamParam/CameraParameter.yaml";
		cparam.loadParam(camParam_path);
#ifdef DEBUG
		std::cout << test.cparam.getCvCamParam() << std::endl;
#endif
		//�ʒu�p������Ɏg���摜��ǂݍ���
		std::vector<std::string> file_names = Util::loadDirFiles("../../data/calib");
		std::vector<cv::Mat> imgs;
		for (const auto& file : file_names)
		{
			cv::Mat img = cv::imread(file);
			imgs.push_back(img);
		}

		cv::Mat cameraMatrix = cparam.getCvCamParam();
		cv::Mat distCoeffs = cparam.getCvDistCoeffs();

		int pattern_width = 12;
		int pattern_height = 8;
		int pattern_margin = 20;
		std::vector<cv::Point3f> obj_pts = Util::create_board3dPts(pattern_width, pattern_height, pattern_margin);
		std::vector<Eigen::Vector3d> obj_pts_eigen;
		for (const auto& obj_pt : obj_pts)
		{
			Eigen::Vector3d obj_pt_eigen(obj_pt.x, obj_pt.y, obj_pt.z);
			obj_pts_eigen.push_back(obj_pt_eigen);
		}

		for (const auto& img : imgs)
		{
			std::vector<cv::Point2f> img_pts;
			bool isFound = cv::findCirclesGrid(img, cv::Size(pattern_width, pattern_height), img_pts, cv::CALIB_CB_SYMMETRIC_GRID);
			cv::Mat rvec, tvec;
			if (isFound)
			{
				std::vector<Eigen::Vector2d> img_pts_eigen;
				for (const auto& img_pt : img_pts)
				{
					Eigen::Vector2d img_pt_eigen(img_pt.x, img_pt.y);
					img_pts_eigen.push_back(img_pt_eigen);
				}
				Eigen::Isometry3d pose;
				//Util::solvePnP(obj_pts_eigen, img_pts_eigen, cparam, pose);
				calcPose(obj_pts_eigen, img_pts_eigen, cparam, pose);
				//cv::solvePnP(obj_pts, img_pts, cameraMatrix, distCoeffs, rvec, tvec, false);
				Eigen::Matrix3f R_eig = pose.rotation().cast<float>();
				cv::eigen2cv(R_eig, rvec);
				cv::Rodrigues(rvec, rvec);
				Eigen::Vector3f t = pose.translation().cast<float>();
				cv::eigen2cv(t, tvec);
				cv::aruco::drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 40.0);
				cv::imshow("img", img);
				cv::waitKey(0);
			}
		}
	}
};

int main(void)
{
	Test test;
	test.execute();




	return 0;
}