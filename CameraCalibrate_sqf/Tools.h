#pragma once
#include<opencv2/opencv.hpp>
#include<vector>

cv::Point2f pixel2camera(const cv::Point2f& point, const cv::Mat& K)
{
	float fx = K.at<double>(0, 0);
	float fy = K.at<double>(1, 1);
	float u0 = K.at<double>(0, 2);
	float v0 = K.at<double>(1, 2);

	cv::Point2f output;
	output.x = (point.x - u0) / fx;
	output.y = (point.y - v0) / fy;
	return output;
}

double computeError(const std::vector<cv::Point3f>& object_point,
	const std::vector<cv::Point2f> image_point,
	cv::Mat R, cv::Mat T, cv::Mat K,
	cv::Mat Dist_coeff)
{
	double error = 0.0;
	std::vector<cv::Point2f> project_point;
	cv::projectPoints(
		object_point,
		R, T, K, 
		Dist_coeff, 
		project_point);
	for (int i = 0; i < object_point.size(); i++)
	{
		cv::Point2f error_point;
		error_point.x = project_point[i].x - image_point[i].x;
		error_point.y = project_point[i].y - image_point[i].y;
		error += double(std::sqrt(error_point.dot(error_point)));
	}
	error /= object_point.size();
	std::cout << "error : " << error << std::endl;
	return error;
}

double computeReprojectError(const std::vector<cv::Point2f>& image_left,
	const std::vector<cv::Point2f>& image_right,
	const cv::Mat& k1,
	const cv::Mat& k2,
	const cv::Mat& dis_coeff1,
	const cv::Mat& dis_coeff2,
	const cv::Mat& R,
	const cv::Mat& T)
{
	
	cv::Mat T1 = (cv::Mat_<double>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	//T1 = k1 * T1;
	
	cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), T.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), T.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), T.at<double>(2, 0));

	//T2 = k2 * T2;
	//convert to the image point to the camera point
	std::vector<cv::Point2f> camera_left, camera_right;
	for (int i = 0; i < image_left.size(); i++)
	{
		camera_left.push_back(pixel2camera(image_left[i], k1));
		camera_right.push_back(pixel2camera(image_right[i], k2));
	}
	cv::Mat points_4d;//4*N
	cv::triangulatePoints(T1, T2, camera_left, camera_right, points_4d);

	//convert 3d points
	std::vector<cv::Point3f> points_3d;
	for (int i = 0; i < points_4d.cols; i++)
	{
		cv::Mat x = points_4d.col(i);
		x /= x.at<float>(3, 0);
		cv::Point3f p(
			x.at<float>(0, 0),
			x.at<float>(1, 0),
			x.at<float>(2, 0)
			);
		points_3d.push_back(p);
	}

	//project to the left and right image
	std::vector<cv::Point2f> project_image_left;
	std::vector<cv::Point2f> project_image_right;
	
	cv::projectPoints(points_3d,
		R,
		T,
		k2,
		dis_coeff2,
		project_image_right);

	cv::projectPoints(points_3d,
		cv::Mat::eye(3, 3, CV_64FC1),
		cv::Mat::zeros(3, 1, CV_64FC1),
		k1,
		dis_coeff1,
		project_image_left);
	cv::Point2f error_point;
	double error_value = 0.0;
	for (int i = 0; i < image_left.size(); i++)
	{
		error_point.x = image_left[i].x - project_image_left[i].x;
		error_point.y = image_left[i].y - project_image_left[i].y;
		error_value += std::sqrt(error_point.dot(error_point));
		error_point.x = image_right[i].x - project_image_right[i].x;
		error_point.y = image_right[i].y - project_image_right[i].y;
		error_value += std::sqrt(error_point.dot(error_point));
	}
	error_value /= (image_left.size() * 2);
	std::cout << "reproject error value : " << error_value << std::endl;
	return error_value;

}