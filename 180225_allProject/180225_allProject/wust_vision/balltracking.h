#pragma once
#ifndef _BALLTRACKING_H
#define _BALLTRACKING_H
#include <opencv2/core.hpp>
#include <iostream>
#include "visiontools.h"
#include "mykinect.h"
#include "My3DKalman.h"
#include "glshow_cvAPI.h"
#include "../fit.h"
namespace mv
{
#define GL_SITEWINDOW "GL SHOW"

	const Point3d CameraCoordinates = Point3d(-1.8, -4.1, 0.988);
	const double transToGlobal[16] = {
		0.9992833003835793, -0.02830158390216655, -0.02513773902183631, 92.3734902225309,
		0.03640124797721943, 0.5362853529298088, 0.8432514271429674, -1196.631387021289,
		-0.01038434977272547, -0.8435621142403084, 0.5369312103968428, -364.697795950226,
		0, 0, 0, 1
	};
	const int CirclePixels[4] = { 284,337,117,180 };//…œœ¬◊Û”“
	void on_MouseSite(int event, int x, int y, int flags, void* param);

	/*struct BallPos
	{
		Point3f ball = Point3f(0.f, 0.f, 0.f);
		float time = 0.f;
	};*/

	class BallTrack
	{
	public:
		BallTrack();
		~BallTrack();
		void IFM(float* _IFM);
		vector<Mat> OutPutErodeBin;
		vector<Mat> OutPutBin;
		vector<Mat> EachBin;

		enum BallResult
		{
			Udeviation = 1,
			Ddeviation = 2,
			Ldeviation = 3,
			Rdeviation = 4,
			NOdeviation = 5
		};
	private:
		MyKinectSensor BallKinect;
		My3DKalman *BallKalman = new My3DKalman;
		GLSiteIFM mySiteIFM;
// 		Fit BallFit_x;
// 		Fit BallFit_y;
// 		Fit BallFit_z;
		int64 time0 = getTickCount();
		//vector<float> IntervalTime;
		bool OnceTrackFlag = false;
		bool TakeFlag = false;
		bool firstFlag = true;

		Point maxY_point = Point(1000, 1000);
		int MaY_depth = 0;
		int DisControl[2] = { 0, 0 };

		uchar InitFlag;
		Point3d BallsPosCalculate[3];
		double BallsTiemCalculate[2];
		double SolveIntervalTime = 0;
		Point3d Ball_V;

		//vector<Point3f> BallCameraPos;
		//vector<Point3f> BallPrePos;
		//vector<float> BallTime;


		//vector<BallPos> BallState;
	};
}

#endif