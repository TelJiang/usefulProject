#pragma once

#ifndef _MYKINECT_H
#define _MYKINECT_H

#include <opencv2/core.hpp>
#include <Kinect.h>
#include <iostream>
#include "visiontools.h"
namespace mv
{

	//kinect内参外参
	const double color_k1 = 9.6867176773248270e-02;
	const double color_k2 = -4.7265407849005944e-02;
	const double color_k3 = 5.5153274022819610e-04;
	const double color_k4 = -5.3004198375921046e-04;
	const double color_k5 = -1.1401904414077289e-01;
	const double color_u0 = 9.4009660940182687e+02;
	const double color_v0 = 5.4068837772070754e+02;
	const double color_fx = 1.1011290098177997e+03;
	const double color_fy = 1.1030708026608088e+03;

	const double ir_k1 = 4.7334587464729024e-02;
	const double ir_k2 = 1.7037397883379560e-01;
	const double ir_p1 = -1.0815660556709908e-03;
	const double ir_p2 = 1.6064442895275682e-03;
	const double ir_k5 = -1.1361771969980621e+00;
	const double ir_u0 = 2.5830829036232512e+02;
	const double ir_v0 = 2.1157683260666076e+02;
	const double ir_fx = 3.8457602676208001e+02;
	const double ir_fy = 3.8605140835207527e+02;

	void get3D_ir(const double& depth, const Point2f& srcPoint, Point3d& _3DPoint);
	void posTo2D(Point3d& src_point, Point2f& dst_point);
	class MyKinectSensor
	{
	public:
		MyKinectSensor(void);
		~MyKinectSensor(void);

		void myKinectInit();
		void outPutCalibXML();
		void kinectDo(KinectDo _mode);
		Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight);
		//Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);
		Mat getBGR();
		Mat getDepth8U();
		Mat getDepth16U();
		Point2f kinect_IR2COLOR(Point2f _point, double z_1);
	private:
		HRESULT hr;
		Mat depthImg_show = cv::Mat::zeros(kinectIrHeight, kinectIrWidth, CV_8UC1);//U16
		Mat depthImg = cv::Mat::zeros(kinectIrHeight, kinectIrWidth, CV_16UC1);//the depth image  
		Mat colorImg = cv::Mat::zeros(kinectColorHeight, kinectColorWidth, CV_8UC3);//the color image  

		IKinectSensor* m_pKinectSensor = NULL;
		// Depth reader  
		IDepthFrameReader*  m_pDepthFrameReader = NULL;
		// Color reader  
		IColorFrameReader*  m_pColorFrameReader = NULL;

		UINT nBufferSize_depth = 0;
		UINT16 *pBuffer_depth = NULL;
		UINT nBufferSize_coloar = 0;
		RGBQUAD *pBuffer_color = NULL;

	};
}

#endif // !_MYKINECT_H