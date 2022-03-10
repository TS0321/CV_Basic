#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

class Camera
{
public:
	Camera() {};
	~Camera() {};
public:
	void init(int camId, int width, int height) {
		cap.open(camId);
		cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	}
	cv::Mat capture() 
	{
		cv::Mat frame;
		cap.read(frame);

		return frame;
	}

private:
	cv::VideoCapture cap;
	
};


int main(void)
{
	Camera cam;
	cam.init(1, 1280, 720);

	std::string save_path = "../../data/calib/";
	int img_num = 0;

	while (1)
	{
		cv::Mat img = cam.capture();
		cv::imshow("img", img);
		int key = cv::waitKey(1);
		if (key == 'c') {
			std::cout << "save image " << img_num << std::endl;
			cv::imwrite(save_path + "img_" + std::to_string(img_num++)+".png", img);
			cv::waitKey(1);
		}
		if (key == 'q')
		{
			break;
		}
	}

	return 0;
}