#pragma once

#ifndef _CLASSIS_MR_H
#define _CLASSIS_MR_H
#include <opencv2/core.hpp>
#include <iostream>
#include "CameraDS.h"
#include "visiontools.h"

namespace mv
{
	class GetLineIFM_MR
	{
		//这种方式不稳定的话，且增加rng次数严重影响程序速度的话
		//可以考略，两个方向上 仅找一根线的方式，结合二值图去处理
	public:
		GetLineIFM_MR();
		~GetLineIFM_MR();
		void IFM(float *_IFM);
		void calculateMiddleLine(const std::vector<cv::Point2f> &points_left, const std::vector<cv::Point2f> &points_right, int _mode);
		void solvePointsY(const std::vector<cv::Point2f> points_y, Mat& for_show, Mat& _Bin);
		void solvePointsX(const std::vector<cv::Point2f> points_x, Mat& for_show, Mat& _Bin);
		void CloseOrOpenCamera(OpenOrCloseCamera close_open, int Camera_num);
		void CoordinateConvert(float *_IFM, float Delta_X, float Delta_Y, float Angel);
		//void roteCoordinate(const std::vector<cv::Point2f> points_src,float _k);

		enum LINE_COORDINATE_CONVERT
		{
			Straight = 1,
			Counterclockwise90 = 2,
			Clockwise90 = 3,
			Counter = 4
		};

	private:
		//cv::VideoCapture cap;
		CCameraDS Camera0;
		cv::Point2f CenterDst;
		cv::Point2f CenterRobot;
		int FactorRange;
		int Factor30MM;
		bool Flag1;
		bool Flag2;
		float Line1K;
		float Line2K;
		float Line1B;
		float Line2B;
		bool LeftRightOutArea;
		bool UpDownOutArea;
		double DSP;
		//LINE_COORDINATE_CONVERT
		bool Line_X_Y_Swap;
		char Line_X_Contrary;
		char Line_Y_Contrary;
		LINE_COORDINATE_CONVERT LineCordinateConvert;
	};
}

#endif //_VISION_H