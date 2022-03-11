#include "util.h"

void Util::loadDirFiles(const std::string dir_path)
{
	for (const auto& file : std::filesystem::directory_iterator(dir_path))
	{
		std::string file_name = file.path().string();
		std::cout << "file_name : " << file_name << " is loaded. " << std::endl;
	}
}