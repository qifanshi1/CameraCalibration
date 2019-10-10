#pragma once
#include<opencv2/opencv.hpp>
#include<vector>

void CameraCalibrateOpti(const std::vector<std::vector<cv::Point3f>>& object_points,
	const std::vector<std::vector<cv::Point2f>>& image_points, const cv::Size& image_size,
	cv::Mat& camera_intrisic, cv::Mat& Dist_coeffs,
	std::vector<cv::Mat>& R_list,
	std::vector<cv::Mat>& T_list,
	const int iter_num = 3,
	const float error_criterion = 0.08)
{
	std::vector<std::vector<cv::Point3f>> object_points_current(object_points.begin(), object_points.end());
	std::vector<std::vector<cv::Point2f>> image_points_current(image_points.begin(), image_points.end());

	std::vector<std::vector<cv::Point3f>> object_points_temp;
	std::vector<std::vector<cv::Point2f>> image_points_temp;
	object_points_temp.resize(object_points.size());
	image_points_temp.resize(object_points.size());
	int i = 0;
	int error_flags = true;
	double error_aver = 0.0;
	int num_point = 0;
	while (error_flags&&i<iter_num)
	{
		error_flags = false;
		//calibrate
		cv::calibrateCamera(object_points_current, image_points_current,
			image_size, camera_intrisic, Dist_coeffs,
			R_list, T_list, cv::CALIB_FIX_K3,
			cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,300,1e-8));
		//calculate error
		std::vector<cv::Point2f> project_points;
		for (int j = 0; j < object_points_current.size(); j++)
		{
			cv::projectPoints(
				object_points_current[j],
				R_list[j],
				T_list[j],
				camera_intrisic,
				Dist_coeffs,
				project_points);
			cv::Point2f error_point;
			double error_value = 0.0;
			error_aver = 0.0;
			num_point = 0;
			for (int k = 0; k < project_points.size(); k++)
			{
				error_point.x = project_points[k].x - image_points_current[j][k].x;
				error_point.y = project_points[k].y - image_points_current[j][k].y;
				error_value = std::sqrt(error_point.dot(error_point));
				error_aver += error_value;
				num_point++;
				if (error_value > error_criterion)
				{
					error_flags = true;
				}
				else
				{
					object_points_temp[j].push_back(object_points_current[j][k]);
					image_points_temp[j].push_back(image_points_current[j][k]);
					
				}
			}
			error_aver /= num_point;
			//std::cout << "error_aver : " << error_aver << std::endl;
			
		}
		//if the size is below 10,then break
		bool bfinished = false;
		for (int j = 0; j < object_points_temp.size(); j++)
		{
			if (object_points_temp[j].size() < 10)
				bfinished = true;
		}
		if (bfinished)
			break;
		object_points_current.clear();
		image_points_current.clear();
		object_points_current.assign(object_points_temp.begin(), object_points_temp.end());
		image_points_current.assign(image_points_temp.begin(), image_points_temp.end());

		object_points_temp.clear();
		image_points_temp.clear();
		object_points_temp.resize(object_points_current.size());
		image_points_temp.resize(image_points_current.size());
		i++;
	}

}
