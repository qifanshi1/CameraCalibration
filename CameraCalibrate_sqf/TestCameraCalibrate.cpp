//利用opencv实现相机标定
#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>
#include<fstream>
#include"io.h"
#include"Tools.h"
#include"Calibrate.h"

int main(int argc, char**argv)
{
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<std::vector<cv::Point2f>> image_points_left;
	std::vector<std::vector<cv::Point2f>> image_points_right;

	std::string file_name = "data_point3d.txt";

	//read 3d points in the target
	std::vector<cv::Point3f> object_point_single;
	bool read_flag = false;
	read_flag = read3dpoint(file_name, object_point_single);
	if (!read_flag)
		return 0;

	//read 2d points in the left images and the right images
	for (int i = 0; i < 5; i++)
	{
		
		std::vector<cv::Point2f> image_point_single1;
		std::vector<cv::Point2f> image_point_single2;
		//file name of the left image
		std::string file_name_image_left = "left_cam\\pos" + std::to_string(i + 1) + ".txt";
		//file name of the right image
		std::string file_name_image_right = "right_cam\\pos" + std::to_string(i + 1) + ".txt";
		bool read_flag_right = false;
		read_flag = read2dpoint(file_name_image_left, image_point_single1);
		read_flag_right = read2dpoint(file_name_image_right, image_point_single2);	

		if (!read_flag||!read_flag_right)
			return 0;
		if ((image_point_single1.size() == object_point_single.size())&&(image_point_single1.size()== image_point_single2.size()))
		{
			object_points.push_back(object_point_single);
			image_points_left.push_back(image_point_single1);
			image_points_right.push_back(image_point_single2);
		}			
	}

	////recover camera intrinsic and external parameter
	cv::Size image_size(2592, 2048);
	cv::Mat Camera_intrisic_left, Camera_intrisic_right, Dist_coeffs_left, Dist_coeffs_right;
	std::vector<cv::Mat> R_list_left, R_list_right, T_list_left, T_list_right;
	//cv::calibrateCamera(object_points, image_points, image_size, Camera_intrisic,
	//	Dist_coeffs, R_list, T_list, CV_CALIB_FIX_K3,
	//	cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-8));
	//std::cout << " Camera_intrisic : " << Camera_intrisic << std::endl;
	//std::cout << "Dist_coeffs : " << Dist_coeffs << std::endl;
	//double aver_error = 0.0;
	//for (int i = 0; i < object_points.size(); i++)
	//{
	//	double error=computeError(object_points[i], image_points[i],
	//		R_list[i], T_list[i], Camera_intrisic,
	//		Dist_coeffs);
	//	aver_error += error;
	//}
	//aver_error /= object_points.size();
	//std::cout << "aver_error : " << aver_error << std::endl;

	//recover camera intrinsic and external parameter after removing points with larger error
	CameraCalibrateOpti(object_points, image_points_left,
		image_size, Camera_intrisic_left, Dist_coeffs_left, R_list_left, T_list_left,3,0.06);

	CameraCalibrateOpti(object_points, image_points_right,
		image_size, Camera_intrisic_right, Dist_coeffs_right, R_list_right, T_list_right, 3, 0.06);

	cv::Mat R, T, E, F;
	cv::stereoCalibrate(object_points, image_points_left,
		image_points_right, Camera_intrisic_left, Dist_coeffs_left,
		Camera_intrisic_right, Dist_coeffs_right, image_size, R, T, E, F,
		cv::CALIB_FIX_INTRINSIC,
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, 1e-15));
	std::cout << " R : " << R << std::endl;
	std::cout << " T : " << T << std::endl;

	//cpmpute reproject error
	double aver_error = 0.0;
	for (int i = 0; i < image_points_left.size(); i++)
	{
		double error = computeReprojectError(image_points_left[i],
			image_points_right[i],
			Camera_intrisic_left,
			Camera_intrisic_right,
			Dist_coeffs_left,
			Dist_coeffs_right,
			R,
			T);
		aver_error += error;
	}
	aver_error /= image_points_left.size();
	std::cout << " Average reprojection error : " << aver_error << std::endl;
	system("pause");
	return 0;
}
