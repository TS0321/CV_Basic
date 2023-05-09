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


int main(void)
{

	cv::Mat img = cv::imread("marker2.png", cv::IMREAD_GRAYSCALE);
	cv::imshow("img", img);
	cv::waitKey(0);
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	clahe->apply(img, img);
	cv::imshow("clahe", img);
	cv::waitKey(0);
	cv::Mat dst_img;
	cv::threshold(img, dst_img, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	cv::imshow("binary_img", dst_img);
	cv::waitKey(0);
	
	cv::Mat src = dst_img.clone();
    //ラべリング処理
    cv::Mat LabelImg;
    cv::Mat stats;
    cv::Mat centroids;
    int nLab = cv::connectedComponentsWithStats(src, LabelImg, stats, centroids);

    // ラベリング結果の描画色を決定
    std::vector<cv::Vec3b> colors(nLab);
    colors[0] = cv::Vec3b(0, 0, 0);
    for (int i = 1; i < nLab; ++i) {
        colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
    }

    // ラベリング結果の描画
    cv::Mat Dst(src.size(), CV_8UC3);
    for (int i = 0; i < Dst.rows; ++i) {
        int* lb = LabelImg.ptr<int>(i);
        cv::Vec3b* pix = Dst.ptr<cv::Vec3b>(i);
        for (int j = 0; j < Dst.cols; ++j) {
            pix[j] = colors[lb[j]];
        }
    }

    //ROIの設定
    for (int i = 1; i < nLab; ++i) {
        int* param = stats.ptr<int>(i);

        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        if (i == 41) {
            cv::rectangle(Dst, cv::Rect(x-10, y-10, width+20, height+20), cv::Scalar(0, 255, 0), 2);
        }
    }

    //重心の出力
    for (int i = 1; i < nLab; ++i) {
        double* param = centroids.ptr<double>(i);
        int x = static_cast<int>(param[0]);
        int y = static_cast<int>(param[1]);

        cv::circle(Dst, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
    }

    //面積値の出力
    for (int i = 1; i < nLab; ++i) {
        int* param = stats.ptr<int>(i);
        std::cout << "area " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;

        //ROIの左上に番号を書き込む
        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        std::stringstream num;
        num << i;
        cv::putText(Dst, num.str(), cv::Point(x + 5, y + 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }

    cv::imshow("Src", src);
    cv::imshow("Labels", Dst);
    cv::waitKey();

    cv::Mat roi_marker;
    {
        int* param = stats.ptr<int>(41);
        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        roi_marker = cv::Mat(dst_img, cv::Rect(x, y, width, height));
        cv::imshow("roi_marker", roi_marker);
        cv::waitKey(0);
    }

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(roi_marker, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat contour_img;
    cv::cvtColor(roi_marker, contour_img, cv::COLOR_GRAY2BGR);
    cv::drawContours(contour_img, contours, -1, cv::Scalar(0, 0, 255), 2);

    // 画像を表示する
    cv::imshow("Contours", contour_img);
    cv::waitKey(0);

    return 0;
	
	//// 輪郭を格納するベクトル
	//std::vector<std::vector<cv::Point>> contours;

	//// 輪郭を検出する
	//findContours(dst_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	//// 輪郭を描画する
 //   cv::Mat dst;
	//cv::cvtColor(dst_img, dst, cv::COLOR_GRAY2BGR);
	//cv::drawContours(dst, contours, -1, cv::Scalar(0, 0, 255), 2);

	//// 画像を表示する
	//cv::imshow("Contours", dst);
	//cv::waitKey(0);


	return 0;
}