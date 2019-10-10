#pragma once
#include<opencv2/opencv.hpp>
#include<fstream>
#include<sstream>

bool read3dpoint(std::string file_name, std::vector<cv::Point3f>& object_points)
{
	//object_points.reserve(100);
	object_points.clear();
	object_points.resize(0);
	std::ifstream in(file_name);
	if (!in.is_open())
	{
		std::cerr << "Error: the file " << file_name << " open failed !" << std::endl;
		return false;
	}
	std::string str;
	float x, y, z;

	while (std::getline(in, str))
	{
		std::istringstream istr(str);
		istr >> x >> y >> z;
		object_points.push_back(cv::Point3f(x, y, z));
	}
	//std::cout << "object_points size : " << object_points.size() << std::endl;
	return true;
}
bool read2dpoint(std::string file_name, std::vector<cv::Point2f>& image_points)
{
	image_points.clear();
	image_points.resize(0);
	std::ifstream in(file_name);
	if (!in.is_open())
	{
		std::cerr << "Error: the file " << file_name << " open failed !" << std::endl;
		return false;
	}
	std::string str;
	float num, x, y;
	while (std::getline(in, str))
	{
		std::istringstream istr(str);
		istr >> num >> x >> y;
		image_points.push_back(cv::Point2f(x, y));
	}
	//std::cout << " image points size : " << image_points.size() << std::endl;
	return true;
}