#pragma once
#ifndef _VISIONTOOLS_H
#define _VISIONTOOLS_H
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
namespace mv
{
	using namespace cv;
	using namespace std;
	//通道分离
	//输入BGR，输出单通道图像，至于是什么取决于_mode
	void my_split(cv::InputArray _src, cv::OutputArray _dst, int _mode);
	bool lineFit_X(const vector<Point2f> &points, double &a, double &b, double &c, double &_x_Dxx);
	bool lineFit_Y(const vector<Point2f> &points, double &a, double &b, double &c, double &_y_Dxx);

	const int simpleCameraWidth = 640;
	const int simpleCameraHeight = 480;
	const int kinectColorWidth = 1920;
	const int kinectColorHeight = 1080;
	const int kinectIrWidth = 512;
	const int kinectIrHeight = 424;
	const float resizeHalf = 0.5f;
	const float duRadians = 180.f / CV_PI;

	enum OpenOrCloseCamera
	{
		MV_OPEN = 1,
		MV_CLOSE = 0
	};

	enum GetMatComponent
	{
		MV_GET_B = 0,
		MV_GET_G = 1,
		MV_GET_R = 2,
		MV_GET_H = 3,
		MV_GET_S = 4,
		MV_GET_V = 5
	};

	enum KinectDo
	{
		MV_KINECT_DOCOLOR = 0,
		MV_KINECT_DOIR = 1,
		MV_KINECT_DOCOLORANDIR = 2
	};
}

#endif