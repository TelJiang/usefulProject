#pragma once
#ifndef _FRONTPOSITION_MR_H
#define _FRONTPOSITION_MR_H

#include <opencv2/calib3d.hpp>
#include <iostream>
#include "visiontools.h"

namespace mv
{
	
	class FrontPosition_MR
	{
	public:
		FrontPosition_MR();
		~FrontPosition_MR();
		void GetPositionIFM(float* _IFM);
		void CloseOrOpenCamera(bool close_open, int Camera_num);
	private:
		cv::VideoCapture cap;
		int  PixelsK = 8;
		float AreaCutK = 0.4;
		float SCM_areaK = 0.6;
		float DSP = 0.0;
		std::vector<cv::Point3f> obj;
		cv::Mat camera_matrix;
		cv::Mat dist_coeffs;
		cv::Mat rv;
		cv::Mat tv;
	};
}


#endif