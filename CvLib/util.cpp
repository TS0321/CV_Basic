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