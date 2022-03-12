#include "util.h"

 std::vector<std::string> Util::loadDirFiles(const std::string dir_path)
{
	 std::vector<std::string> file_names;
	 for (const auto& file : std::filesystem::directory_iterator(dir_path))
	 {
	 	std::string file_name = file.path().string();
	 	std::cout << "file_name : " << file_name << " is loaded. " << std::endl;
		if (file.exists()) 
		{
			file_names.push_back(file_name);
		}
	 }

	 return file_names;
}

 //calibrationボードのdotの三次元位置を算出する関数
 std::vector<cv::Point3f> Util::create_board3dPts(int width, int height, double margin)
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
