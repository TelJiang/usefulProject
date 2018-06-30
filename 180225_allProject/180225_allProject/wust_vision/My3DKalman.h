#pragma once
#ifndef _MYKALMAN_H
#define _MYKALMAN_H
#include <opencv2/core.hpp>
#include <iostream>
#include <opencv2/video/tracking.hpp>
namespace mv
{
#define airParameter_b 0.033216//0.02321//0.033216
#define  mOfBadminton 0.08
#define G_gravity 9.8
	const double ElevationTheta = 32.4865 * 3.1415926 / 180.0;
	class My3DKalman
	{
	public:
		My3DKalman();
		~My3DKalman();
		void DoKalman(const cv::Point3d& _postCoordinate, const cv::Point3d& _V, const double& intervalTime, cv::Point3d& _preCoordinate);
		//void prediction(const double& intervalTime, std::vector<cv::Point3d>& _preCoordinates);
		void KalmanInit(const cv::Point3d& firstPoint, const cv::Point3d& _initV);
		cv::KalmanFilter* KF;

	private:
		cv::Mat* control;
		cv::Mat* measurement;
		double kalmanT;
	};


}

#endif