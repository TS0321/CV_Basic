#include <opencv2/opencv.hpp>
#include <iostream>

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
	while (1)
	{
		cv::Mat img = cam.capture();
		cv::imshow("img", img);
		cv::waitKey(1);
	}

	return 0;
}