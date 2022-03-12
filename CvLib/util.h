#pragma once

#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

class Util
{
public:
	Util() {};
	~Util() {};
public:
	static std::vector<std::string> loadDirFiles(const std::string dir_path);
};