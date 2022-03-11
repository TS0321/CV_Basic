#include <iostream>
#include <filesystem>
#include <string>

class Util
{
public:
	Util() {};
	~Util() {};
public:
	static void loadDirFiles(const std::string dir_path);
};