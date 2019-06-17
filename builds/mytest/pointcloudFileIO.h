#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class pointcloudFileIO
{
public:
	static const int OK = 0;
	static const int ERR = -1;

	/**************************
	2D x 3 format
	**************************/

	static int savePointCloud2DMatToRawTxt(std::string path, cv::Mat points, cv::Mat colors, cv::Mat isValids);
	static int savePointCloud2DMatToObj(std::string path, cv::Mat points, cv::Mat colors, cv::Mat isValids);


	/**************************
	N x 3 format
	**************************/
	static int LoadPointCloudFromRawTxt(const std::string _PCDataPath, cv::Mat &PCData_);
	static int LoadPointCloudFromObj(const std::string _PCDataPath, cv::Mat &PCData_);
	static int LoadPointCloudWNormalFromObj(const std::string _PCDataPath, cv::Mat &PCData_, cv::Mat &NormalData_);
	static int LoadPointCloudWNormalFromPly(const std::string _PCDataPath, cv::Mat &PCData_, cv::Mat &NormalData_);

	static int ScalePointCloud(cv::Mat &PCData, float scaling_factor);
	static int SavePointCloudToRawTxt(const std::string saveToPath, const cv::Mat &points, int iMatDataType);
	static int SavePointCloudToObj(const std::string saveToPath, const cv::Mat &points, int iMatDataType);
	static int SavePointCloudWNormalToPly(const std::string _PCDataPath, const cv::Mat &PCData_, const cv::Mat &NormalData_);

};


