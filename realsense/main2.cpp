#include <random>
#include "opencv2/opencv.hpp"
#include "realsense2/rs.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <algorithm>


void convertColoredDepthImage(const cv::Mat& depthImg, cv::Mat& coloredDepth)
{
	coloredDepth = depthImg.clone();
	double min;
	double max;
	cv::minMaxIdx(depthImg, &min, &max);
	coloredDepth -= min;
	cv::convertScaleAbs(coloredDepth, coloredDepth, 255 / (max - min));
	cv::applyColorMap(coloredDepth, coloredDepth, cv::COLORMAP_JET);
}

bool detect_marker(cv::Mat& image)
{
	cv::Mat grayImg = image;
	//cv::imshow("grayImg", grayImg);
	//cv::waitKey(0);

	//ヒストグラム平坦化
	//cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	//clahe->apply(grayImg, grayImg);
	//cv::imshow("clahe", grayImg);
	//cv::waitKey(0);

	//二値化
	cv::Mat binaryImg;
	cv::threshold(grayImg, binaryImg, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	//cv::imshow("binaryImg", binaryImg);
	//cv::waitKey(0);

	//ラベリング処理
	cv::Mat labelImg;
	cv::Mat stats;
	cv::Mat centroids;
	int nLabel = cv::connectedComponentsWithStats(binaryImg, labelImg, stats, centroids);

	//ラベリング結果の描画色を決定
	std::vector<cv::Vec3b> colors(nLabel);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 0; i < nLabel; ++i)
	{
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	//ラベリング結果の描画
	cv::Mat labeledImg(grayImg.size(), CV_8UC3);
	for (int i = 0; i < labeledImg.rows; ++i)
	{
		int* lb = labelImg.ptr<int>(i);
		cv::Vec3b* pix = labeledImg.ptr<cv::Vec3b>(i);
		for (int j = 0; j < labeledImg.cols; ++j)
		{
			pix[j] = colors[lb[j]];
		}
	}

	std::vector<int> markerCandidateIdx;

	//外れ値除去
	const int min_areaThresh = 1000;
	const int max_areaThresh = 100000;
	for (int i = 0; i < nLabel; ++i) {
		int* param = stats.ptr<int>(i);

		const int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		const int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		const int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		const int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		const int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
		
		//画像の面積が小さい場合に省く
		if (area < min_areaThresh || area > max_areaThresh)
		{
			continue;
		}

		//画像端はのぞく
		if (x <= 0 || y <= 0 || x + width >= image.cols - 1 || y + height >= image.rows - 1)
		{
			continue;
		}

		cv::rectangle(image, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
		markerCandidateIdx.push_back(i);
	}
	cv::imshow("labeledImg", image);
	cv::waitKey(1);

	for (int i = 0; i < markerCandidateIdx.size(); i++)
	{
		const int* param = stats.ptr<int>(markerCandidateIdx[i]);
		const int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		const int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		const int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		const int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

		std::vector<std::vector<cv::Point>> contours;
		cv::Mat roi_marker = cv::Mat(binaryImg, cv::Rect(x, y, width, height));
		cv::findContours(roi_marker, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(x, y));
		cv::Mat contour_img = cv::Mat::zeros(binaryImg.size(), CV_8UC1);
		int max_contourIdx = -1;
		double max_area = -DBL_MAX;
		for (int j = 0; j < contours.size(); j++)
		{
			double area = cv::contourArea(contours[j]);
			if (area > max_area)
			{
				max_area = area;
				max_contourIdx = j;
			}
		}

		std::vector<cv::Point> contour = contours[max_contourIdx];
		const int contour_pointNum = contour.size();
		cv::cvtColor(grayImg, contour_img, cv::COLOR_GRAY2BGR);
		const double corner_thresh = 0.8;
		std::vector<std::pair<int, int>> corner_peak;
		std::vector<double> corner_score;
		bool peak_flag = false;
		int start_idx = 0;
		int end_idx = 0;
		for (int k = 0; k < contour_pointNum; k++)
		{
			cv::Point prev_pt = contour[(k + contour_pointNum - 5) % contour_pointNum];
			cv::Point pt = contour[k];
			cv::Point next_pt = contour[(k + contour_pointNum + 5) % contour_pointNum];

			Eigen::Vector2d A(prev_pt.x - pt.x, prev_pt.y - pt.y);
			Eigen::Vector2d B(next_pt.x - pt.x, next_pt.y - pt.y);
			double score = std::abs(A.normalized().dot(B.normalized()));
			corner_score.push_back(score);
			if (!peak_flag && score < corner_thresh)
			{
				start_idx = k;
				peak_flag = true;
			}
			if (peak_flag && score > corner_thresh)
			{
				end_idx = (k + contour_pointNum - 1) % contour_pointNum;
				peak_flag = false;
				corner_peak.push_back(std::make_pair(start_idx, end_idx));
			}
			else if (peak_flag && k == contour_pointNum - 1)
			{
				end_idx = k;
				peak_flag = false;
				corner_peak.push_back(std::make_pair(start_idx, end_idx));
			}
		}

		if (corner_peak[0].first == 0 && corner_peak[corner_peak.size() - 1].second == contour_pointNum - 1)
		{
			corner_peak[0].first = corner_peak[corner_peak.size() - 1].second;
			corner_peak.erase(corner_peak.end() - 1);
		}

		std::vector<int> corner_id;
		for (auto& [start, end] : corner_peak)
		{
			int min_id = INT_MIN;
			double min_score = DBL_MAX;
			std::cout << "start : " << start << ", end : " << end << std::endl;

			if (start > end)
			{
				start = start - contour_pointNum;
			}
			std::cout << "start : " << start << ", end : " << end << std::endl;
			for (int i = start; i <= end; i++)
			{
				if (corner_score[(i + contour_pointNum) % contour_pointNum] < min_score)
				{
					min_id = i;
					min_score = corner_score[(i + contour_pointNum) % contour_pointNum];
				}
			}
			corner_id.push_back(min_id);
		}

		for (const auto& id : corner_id)
		{
			std::cout << "id : " << id << std::endl;
			cv::circle(contour_img, contour[id], 3, cv::Scalar(0, 255, 0));
		}
		cv::imshow("corner", contour_img);
		cv::waitKey(1);


	}
	


	return true;
}

int main(int argc, char* argv[])
{
	int width = 848;
	int height = 480;
	int fps = 30;
	rs2::config config;
	config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

	// start pipeline
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile = pipeline.start(config);

	auto infrared_frame1 = pipeline.get_active_profile().get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
	auto infrared_frame2 = pipeline.get_active_profile().get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
	auto intrinsic_1 = infrared_frame1.get_intrinsics();
	auto intrinsic_2 = infrared_frame2.get_intrinsics();
	auto extrinsic_1to2 = infrared_frame1.get_extrinsics_to(infrared_frame2);
	std::cout << "intrinsic1_fx : " << intrinsic_1.fx << std::endl;
	std::cout << "intrinsic1_fy : " << intrinsic_1.fy << std::endl;
	std::cout << "intrinsic1_cx : " << intrinsic_1.ppx << std::endl;
	std::cout << "intrinsic1_cy : " << intrinsic_1.ppy << std::endl;
	std::cout << "intrinsic2_fx : " << intrinsic_2.fx << std::endl;
	std::cout << "intrinsic2_fy : " << intrinsic_2.fy << std::endl;
	std::cout << "intrinsic2_cx : " << intrinsic_2.ppx << std::endl;
	std::cout << "intrinsic2_cy : " << intrinsic_2.ppy << std::endl;

	const double fx = intrinsic_1.fx;
	const double fy = intrinsic_1.fy;
	const double cx = intrinsic_1.ppx;
	const double cy = intrinsic_1.ppy;
	const double k1 = intrinsic_1.coeffs[0];
	const double k2 = intrinsic_1.coeffs[1];
	const double p1 = intrinsic_1.coeffs[2];
	const double p2 = intrinsic_1.coeffs[3];
	const double k3 = intrinsic_1.coeffs[4];

	cv::Mat cvT = (cv::Mat_<double>(3, 1) << extrinsic_1to2.translation[0], extrinsic_1to2.translation[1], extrinsic_1to2.translation[2]);
	cv::Mat cvR = (cv::Mat_<double>(3, 3) << extrinsic_1to2.rotation[0], extrinsic_1to2.rotation[3], extrinsic_1to2.rotation[6],
		extrinsic_1to2.rotation[1], extrinsic_1to2.rotation[4], extrinsic_1to2.rotation[7],
		extrinsic_1to2.rotation[2], extrinsic_1to2.rotation[5], extrinsic_1to2.rotation[8]);
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
	cv::Mat R1, R2, P1, P2, Q;

	cv::stereoRectify(cameraMatrix, distCoeffs, cameraMatrix, distCoeffs, cv::Size(width, height), cvR, cvT, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY);
	std::cout << "cvT : " << cvT << std::endl;
	std::cout << "cvR : " << cvR << std::endl;
	std::cout << "R1 : " << R1 << std::endl;
	std::cout << "R2 : " << R2 << std::endl;
	std::cout << "P1 : " << P1 << std::endl;
	std::cout << "P2 : " << P2 << std::endl;
	cv::Mat M1l, M2l, M1r, M2r;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, P1.rowRange(0, 3).colRange(0, 3), cv::Size(width, height), CV_32F, M1l, M2l);    //左目のremap用のマップを生成
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R2, P2.rowRange(0, 3).colRange(0, 3), cv::Size(width, height), CV_32F, M1r, M2r);    //右目のremap用のマップを生成


	auto device = pipeline.get_active_profile().get_device();
	auto depth_sensor = device.query_sensors()[0];
	depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);	//emitterを切る
	while (1) // Application still alive?
	{
		// wait for frames and get frameset
		rs2::frameset frameset = pipeline.wait_for_frames();

		// get single infrared frame from frameset
		//rs2::video_frame ir_frame = frameset.get_infrared_frame();

		// get left and right infrared frames from frameset
		rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

		cv::Mat dMat_left = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
		cv::Mat dMat_right = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

		cv::remap(dMat_left, dMat_left, M1l, M2l, cv::INTER_LINEAR);
		cv::remap(dMat_right, dMat_right, M1r, M2r, cv::INTER_LINEAR);


		cv::imshow("img_l", dMat_left);
		cv::imshow("img_r", dMat_right);
		char c = cv::waitKey(1);

		detect_marker(dMat_left);

		if (c == 's')
		{
			std::cout << "saveImg" << std::endl;
			cv::imwrite("marker2.png", dMat_left);
			cv::waitKey(1);
		}
		else if (c == 'q')
			break;
	}

	return EXIT_SUCCESS;
}

//int main() {
//
//    rs2::colorizer color_map;
//    rs2::pipeline pipe;
//    rs2::config cfg;
//    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
//    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
//    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
//    pipe.start(cfg);
//
//    rs2::pipeline_profile pipeProfile = pipe.get_active_profile();
//    rs2_intrinsics intrinsics =
//        pipeProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
//    rs2_intrinsics intrinsics_depth =
//        pipeProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
//    //rs2_intrinsics intrinsics_infrared = 
//    //    pipeProfile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>().get_intrinsics();
//    while (true)
//    {
//        rs2::frameset frames = pipe.wait_for_frames();
//        //rs2::frame color_frame = frames.get_color_frame();
//        //cv::Mat color_bgr(cv::Size(intrinsics.width, intrinsics.height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
//        //cv::Mat color;
//        //cv::cvtColor(color_bgr, color, cv::COLOR_BGR2RGB);
//        //cv::imshow("color", color);
//        //cv::waitKey(1);
//
//        //rs2::frame depth_frame = frames.get_depth_frame();
//        //cv::Mat coloredDepth, depth;
//        //cv::Mat depthImg16U = cv::Mat(cv::Size(intrinsics_depth.width, intrinsics_depth.height), CV_16U, (char*)depth_frame.get_data());
//        //convertColoredDepthImage(depthImg16U, coloredDepth);
//        //cv::imshow("depth", coloredDepth);
//        //cv::waitKey(1);
//        rs2::frame infrared_frame = frames.first(RS2_STREAM_INFRARED);
//        cv::Mat infrared(cv::Size(640, 480), CV_8UC1, (void*)infrared_frame.get_data(), cv::Mat::AUTO_STEP);
//        cv::equalizeHist(infrared, infrared);
//        cv::applyColorMap(infrared, infrared, cv::COLORMAP_JET);
//
//        cv::imshow("infrared", infrared);
//        cv::waitKey(1);
//           
//    }
//
//    return 0;
//}