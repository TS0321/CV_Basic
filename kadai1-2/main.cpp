#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

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
		//カメラパラメータの読み込み
		const std::string camParam_path = "../../CamParam/CameraParameter.yaml";
		cparam.loadParam(camParam_path);
#ifdef DEBUG
		std::cout << test.cparam.getCvCamParam() << std::endl;
#endif
		//位置姿勢推定に使う画像を読み込む
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

		for (const auto& img : imgs)
		{
			std::vector<cv::Point2f> img_pts;
			bool isFound = cv::findCirclesGrid(img, cv::Size(pattern_width, pattern_height), img_pts, cv::CALIB_CB_SYMMETRIC_GRID);
			cv::Mat rvec, tvec;
			if (isFound)
			{
				cv::solvePnP(obj_pts, img_pts, cameraMatrix, distCoeffs, rvec, tvec, false);
				cv::aruco::drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 20.0);
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