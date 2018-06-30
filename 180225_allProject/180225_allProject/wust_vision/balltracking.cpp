#include "balltracking.h"

namespace mv
{
#define Ff(x,y,z){\
	ball_vv = (x)*(x)+(y)*(y)+(z)*(z); \
	ball_v = sqrt(ball_vv); \
	temp = -abs((airParameter_b * ball_v) / mOfBadminton); \
	f[0] = temp*(x); \
	f[1] = temp*(y)+G_gravity*cos(ElevationTheta); \
	f[2] = temp*(z)-G_gravity*sin(ElevationTheta); \
}
	///////////////////////////////////////////////////

	FileStorage fs("globalpoint.xml", cv::FileStorage::WRITE);
	void on_MouseSite(int event, int x, int y, int flags, void* param)
	{
		mv::GLSiteIFM& _IFM = *(mv::GLSiteIFM*)param;
		static cv::Point point1, nowPoint;
		static bool moveSiteFlag = 0;
		switch (event)
		{
			//左键按下消息
		case cv::EVENT_LBUTTONDOWN:
		{
			point1 = cv::Point(x, y);
			moveSiteFlag = true;
		}
		break;

		//鼠标移动消息
		case cv::EVENT_MOUSEMOVE:
		{
			nowPoint = cv::Point(x, y);
			if (moveSiteFlag)
			{
				_IFM.z += 0.1*(nowPoint.y - point1.y);
				_IFM.angel += (nowPoint.x - point1.x);
				_IFM.x = _IFM.radius*sin(_IFM.angel*CV_PI / 180.0);
				_IFM.y = _IFM.radius*cos(_IFM.angel*CV_PI / 180.0);
				float k = 1.3;
				if (_IFM.z > _IFM.radius / k)
					_IFM.z = _IFM.radius / k;
				else if (_IFM.z < 1.5)
					_IFM.z = 1.5;
				point1 = nowPoint;
				cv::updateWindow(GL_SITEWINDOW);
			}
		}
		break;

		//左键抬起消息
		case cv::EVENT_LBUTTONUP:
			moveSiteFlag = false;
			break;
		default:
			break;
		}
	}
	//////////////////////////////////////////////////

	BallTrack::BallTrack()
	{
		//opengl部分
		cv::namedWindow(GL_SITEWINDOW, WINDOW_OPENGL);
		resizeWindow(GL_SITEWINDOW, 640, 480);
		setOpenGlContext(GL_SITEWINDOW);
		setOpenGlDrawCallback(GL_SITEWINDOW, mv::glOnDraw, (void*)&mySiteIFM);
		setMouseCallback(GL_SITEWINDOW, mv::on_MouseSite, (void*)&mySiteIFM);

		// 		mySiteIFM.BallPoints.push_back(Point3f(-0.269891, -7.60748, 4.13809));
		// 		mySiteIFM.BallPoints.push_back(Point3f(-0.269891, -0.249358, 4.13809));
		//		cv::updateWindow(GL_SITEWINDOW);
	}

	BallTrack::~BallTrack()
	{
	}

	void BallTrack::IFM(float* _IFM)
	{
		if (firstFlag)
		{
			Mat src_ir_u16, _Bin;
			_Bin = Mat::zeros(Size(kinectIrWidth, kinectIrHeight), CV_8UC1);
			BallKinect.kinectDo(MV_KINECT_DOIR);
			src_ir_u16 = BallKinect.getDepth16U();
			for (int i = 0; i < kinectIrHeight; i++)
			{
				for (int j = 0; j < kinectIrWidth; j++)
				{
					if (src_ir_u16.ptr<UINT16>(i)[j] > 3200
						&& src_ir_u16.ptr<UINT16>(i)[j] < 3900)
					{
						_Bin.ptr<UINT8>(i)[j] = 255;
					}
				}
			}

			imshow("123", _Bin);
			vector<vector<Point>> _points;
			int contourNum = -1;
			int maxNum = 0;
			findContours(_Bin, _points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			for (size_t i = 0; i < _points.size(); i++)
			{
				if (maxNum < _points[i].size())
				{
					maxNum = _points[i].size();
					contourNum = i;
				}
			}

			if (contourNum >= 0)
			{
				Point3d _point = Point3d(0.f, 0.f, 0.f);				
				for (size_t i = 0; i < _points[contourNum].size(); i++)
				{
					if (_points[contourNum][i].y < maxY_point.y)
					{
						maxY_point.x = _points[contourNum][i].x;
						maxY_point.y = _points[contourNum][i].y;
						MaY_depth = src_ir_u16.ptr<UINT16>(maxY_point.y)[maxY_point.x];

					}
				}
				get3D_ir(src_ir_u16.ptr<UINT16>(maxY_point.y)[maxY_point.x], Point2f(kinectIrWidth - maxY_point.x, maxY_point.y), _point);
				DisControl[0] = _point.y*sin(ElevationTheta) + _point.z*cos(ElevationTheta) - 50;
				DisControl[1] = _point.y*sin(ElevationTheta) + _point.z*cos(ElevationTheta) + 50;
				
			}

			firstFlag = false;
		}
		Mat src_ir_u16;
		BallKinect.kinectDo(MV_KINECT_DOIR);
		src_ir_u16 = BallKinect.getDepth16U();
		double take_intervalTime = (double)(getTickCount() - time0) / getTickFrequency();
		time0 = getTickCount();
		//threshold(src_ir, Bin, 5000, 65535, THRESH_TOZERO);
		//threshold(Bin, Bin, 450, 65535, THRESH_BINARY);
		Mat Bin(Size(kinectIrWidth, kinectIrHeight), CV_8UC1, Scalar(0));

		for (int i = 0; i < kinectIrHeight; i++)
		{
			for (int j = 0; j < kinectIrWidth; j++)
			{
				if (src_ir_u16.ptr<UINT16>(i)[j] > 450
					&& src_ir_u16.ptr<UINT16>(i)[j] < 3800)
				{
					Bin.ptr<UINT8>(i)[j] = 255;
				}
			}
		}
		//GaussianBlur(Bin, Bin, Size(3, 3), 3, 3);
		//Mat _element;
		//_element = getStructuringElement(MORPH_RECT, Size(5,5));
		//morphologyEx(Bin, Bin, MORPH_OPEN, _element);
		imshow("Bin", Bin);
		if (TakeFlag&&EachBin.size() < 30)
		{
			EachBin.push_back(Bin);
		}
		else if (EachBin.size() >= 30)
		{
			TakeFlag = false;
		}

		Mat BinErode;
		Mat _element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(Bin, BinErode, MORPH_ERODE, _element);
		Mat forContour;
		vector<vector<Point>> contourPoints;
		BinErode.copyTo(forContour);

		findContours(forContour, contourPoints, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		int contourSize = contourPoints.size();
		int contourNum = -1;
		float maxArea = 0;
		for (int i = 0; i < contourSize; i++)
		{
			if (contourPoints[i].size() > 20)
			{
				//drawContours(forShow, contourPoints, i, Scalar(130, 180, 231), -2);
				float _area = contourArea(contourPoints[i]);
				if (_area > maxArea)
				{
					maxArea = _area;
					contourNum = i;
					//cout << contourPoints[contourNum].size() << endl;
				}
			}
		}

		if (contourNum >= 0)
		{
			Point2f ballCenter;
			Point3d ballPosition;
			Point3d ballPrePosition;
			float ballDistance;
			Moments ballMu;
			ballMu = moments(contourPoints[contourNum], false);
			ballCenter = Point2f(static_cast<float>(ballMu.m10 / ballMu.m00), static_cast<float>(ballMu.m01 / ballMu.m00));
			ballDistance = src_ir_u16.ptr<UINT16>((int)ballCenter.y)[(int)ballCenter.x];
			get3D_ir(ballDistance / 1000.0, Point2f(kinectIrWidth - ballCenter.x, ballCenter.y), ballPosition);

			//此处连通域中心像素有可能为噪点，即值为0，当出现这种情况时
			if (ballPosition.z <= 0.3f)
			{
				for (int i = -10; i < 10; i++)
				{
					int _x = (int)ballCenter.x + i;
					if (_x >= 0 &&
						_x < kinectIrWidth)
					{
						for (int j = -10; j < 10; j++)
						{
							int _y = (int)ballCenter.y + j;
							if (_y >= 0 &&
								_y < kinectIrHeight)
							{
								get3D_ir(ballDistance / 1000.0, Point2f(kinectIrWidth - _x, _y), ballPosition);
							}
							if (ballPosition.z > 0.3f)
								break;
						}
					}
					if (ballPosition.z > 0.3f)
						break;
				}
			}

			//cout << ballPosition.z << endl;
			circle(Bin, ballCenter, 5, 150, -1);

			if (OutPutBin.size() <= 20)
				OutPutBin.push_back(Bin);
			// 			else
			// 				OutPutBin.clear();

			if (OutPutErodeBin.size() <= 20)
				OutPutErodeBin.push_back(BinErode);
			// 			else
			// 				OutPutErodeBin.clear();
			if (!OnceTrackFlag)
			{
				mySiteIFM.preBallPoints.clear();
				mySiteIFM.BallPoints.clear();
				TakeFlag = true;
				InitFlag++;
				switch (InitFlag)
				{
				case 1:
					BallsPosCalculate[0].x = ballPosition.x;
					BallsPosCalculate[0].y = ballPosition.y;
					BallsPosCalculate[0].z = ballPosition.z;
					cout << "first point " << ballPosition << endl;
					break;
				case 2:
					BallsPosCalculate[1].x = ballPosition.x;
					BallsPosCalculate[1].y = ballPosition.y;
					BallsPosCalculate[1].z = ballPosition.z;
					BallsTiemCalculate[0] = take_intervalTime;
					cout << "second point :" << ballPosition << endl;
					break;
				case 3:
					OnceTrackFlag = true;
					BallsPosCalculate[2].x = ballPosition.x;
					BallsPosCalculate[2].y = ballPosition.y;
					BallsPosCalculate[2].z = ballPosition.z;
					BallsTiemCalculate[1] = take_intervalTime;
					cout << "third point :" << ballPosition << endl;
					Ball_V.x = (BallsPosCalculate[2].x - BallsPosCalculate[0].x) / ((BallsTiemCalculate[0] + BallsTiemCalculate[1]) / 2.0);
					Ball_V.y = (BallsPosCalculate[2].y - BallsPosCalculate[0].y) / ((BallsTiemCalculate[0] + BallsTiemCalculate[1]) / 2.0);
					Ball_V.z = (BallsPosCalculate[2].z - BallsPosCalculate[0].z) / ((BallsTiemCalculate[0] + BallsTiemCalculate[1]) / 2.0);
					BallKalman->KalmanInit(BallsPosCalculate[1], Ball_V);
					break;
				default:
					break;
				}

				//BallKalman.KalmanInit(ballPosition);			
				//cout << ballPosition << endl;
				//Point3f _globalPoints(
				//	transToGlobal[0] * ballPosition.x + transToGlobal[1] * ballPosition.y + transToGlobal[2] * ballPosition.z - 3.f,
				//	transToGlobal[4] * ballPosition.x + transToGlobal[5] * ballPosition.y + transToGlobal[6] * ballPosition.z - 4.11f,
				//	transToGlobal[8] * ballPosition.x + transToGlobal[9] * ballPosition.y + transToGlobal[10] * ballPosition.z + 0.94f
				//);
				//mySiteIFM.BallPoints.push_back(_globalPoints);

//				BallCameraPos.push_back(ballPosition);
//				BallPos.push_back(ballPosition);
//				BallTime.push_back(0.f);
			}
			else
			{
				for (int i = 0; i < 2; i++)
				{
					BallsPosCalculate[i].x = BallsPosCalculate[i + 1].x;
					BallsPosCalculate[i].y = BallsPosCalculate[i + 1].y;
					BallsPosCalculate[i].z = BallsPosCalculate[i + 1].z;
				}
				BallsPosCalculate[2].x = ballPosition.x;
				BallsPosCalculate[2].y = ballPosition.y;
				BallsPosCalculate[2].z = ballPosition.z;

				BallsTiemCalculate[0] = BallsTiemCalculate[1];
				BallsTiemCalculate[1] = take_intervalTime;

				Ball_V.x = (BallsPosCalculate[2].x - BallsPosCalculate[0].x) / ((BallsTiemCalculate[0] + BallsTiemCalculate[1]) / 2.0);
				Ball_V.y = (BallsPosCalculate[2].y - BallsPosCalculate[0].y) / ((BallsTiemCalculate[0] + BallsTiemCalculate[1]) / 2.0);
				Ball_V.z = (BallsPosCalculate[2].z - BallsPosCalculate[0].z) / ((BallsTiemCalculate[0] + BallsTiemCalculate[1]) / 2.0);

				BallKalman->DoKalman(BallsPosCalculate[1], Ball_V, BallsTiemCalculate[0], ballPrePosition);

				Point3f _globalPoints(
					transToGlobal[0] * ballPosition.x + transToGlobal[1] * ballPosition.y + transToGlobal[2] * ballPosition.z + (float)CameraCoordinates.x,
					transToGlobal[4] * ballPosition.x + transToGlobal[5] * ballPosition.y + transToGlobal[6] * ballPosition.z + (float)CameraCoordinates.y,
					transToGlobal[8] * ballPosition.x + transToGlobal[9] * ballPosition.y + transToGlobal[10] * ballPosition.z + (float)CameraCoordinates.z
				);

				mySiteIFM.BallPoints.push_back(_globalPoints);
				//cout <<"z:"<< transToGlobal[8] * ballPosition.x + transToGlobal[9] * ballPosition.y + transToGlobal[10] * ballPosition.z + (float)CameraCoordinates.z << endl;

				mySiteIFM.preBallPoints.push_back(Point3f(
					transToGlobal[0] * ballPrePosition.x + transToGlobal[1] * ballPrePosition.y + transToGlobal[2] * ballPrePosition.z + (float)CameraCoordinates.x,
					transToGlobal[4] * ballPrePosition.x + transToGlobal[5] * ballPrePosition.y + transToGlobal[6] * ballPrePosition.z + (float)CameraCoordinates.y,
					transToGlobal[8] * ballPrePosition.x + transToGlobal[9] * ballPrePosition.y + transToGlobal[10] * ballPrePosition.z + (float)CameraCoordinates.z
				));

				fs << "globalPoints_x" << _globalPoints.x;
				fs << "globalPoints_y" << _globalPoints.y;
				fs << "globalPoints_z" << _globalPoints.z;

				fs << "preBallPoints_x" << transToGlobal[0] * ballPrePosition.x + transToGlobal[1] * ballPrePosition.y + transToGlobal[2] * ballPrePosition.z + (float)CameraCoordinates.x;
				fs << "preBallPoints_y" << transToGlobal[4] * ballPrePosition.x + transToGlobal[5] * ballPrePosition.y + transToGlobal[6] * ballPrePosition.z + (float)CameraCoordinates.y;
				fs << "preBallPoints_z" << transToGlobal[8] * ballPrePosition.x + transToGlobal[9] * ballPrePosition.y + transToGlobal[10] * ballPrePosition.z + (float)CameraCoordinates.z;


				//cout << "!!!!!!!!" << endl;
				cv::updateWindow(GL_SITEWINDOW);

				//IntervalTime.push_back(_intervalTime);

				//BallPos.push_back(ballPosition);
				//int ballNum = BallTime.size();
				//BallTime.push_back(_intervalTime + BallTime[ballNum - 1]);

				//cout << "vx:	" << (ballPosition.x - BallPos[ballNum - 1].x) / _intervalTime << endl;
				//cout << "vy:	" << (ballPosition.y - BallPos[ballNum - 1].y) / _intervalTime << endl;
				//cout << "vz:	" << (ballPosition.z - BallPos[ballNum - 1].z) / _intervalTime << endl;
			}
		}
		else if (OnceTrackFlag)
		{
			//完成一次跟踪
			OnceTrackFlag = false;
			//开始推算
			//double _time=0.05;
			//vector<Point3f> _BallPoints;
			//for (int i = 0; i < 30; i++)
			//{
			//	BallKalman.prediction(_time, _BallPoints);
			//	mySiteIFM.preBallPoints.push_back(Point3f(
			//		transToGlobal[0] * _BallPoints[i].x + transToGlobal[1] * _BallPoints[i].y + transToGlobal[2] * _BallPoints[i].z - 3.f,
			//		transToGlobal[4] * _BallPoints[i].x + transToGlobal[5] * _BallPoints[i].y + transToGlobal[6] * _BallPoints[i].z - 4.11f,
			//		transToGlobal[8] * _BallPoints[i].x + transToGlobal[9] * _BallPoints[i].y + transToGlobal[10] * _BallPoints[i].z + 0.94f
			//	));
			//	//if (_BallPoints[i].z*cos(Elevation) + _BallPoints[i].y*sin(Elevation)>4000)
			//	//{
			//	//	//准备进框
			//	//}
			//	cv::updateWindow(GL_SITEWINDOW);
			//}

			/*vector<float> _x;
			vector<float> _y;
			vector<float> _z;
			vector<float> _t;
			int ballNum = BallPos.size();
			for (int i = 1; i < ballNum; i++)
			{
				_x.push_back(BallPos[i].x);
				_y.push_back(BallPos[i].y);
				_z.push_back(BallPos[i].z);
				_t.push_back(BallTime[i]);
			}
			BallFit_x.polyfit(_t, _x, 3);
			BallFit_y.polyfit(_t, _y, 3);
			BallFit_z.polyfit(_t, _z, 3);

			float hist_x = 0,f;
			bool x0_flag = false;
			bool z0_flag = false;
			float hist_z = 0.f;*/
			/*for (int i = 0; i < 80; i++)
			{
				float _time = i*0.04f;
				Point3f prePoint;
				prePoint.y = BallFit_y.getY(_time);
				(BallFit_x.getY(_time), , BallFit_z.getY(_time));
				if ((BallFit_x.getY(0.f) - BallFit_x.getY(0.02))*(BallFit_x.getY(_time) - BallFit_x.getY(_time + 0.02)) > 0
					&& !x0_flag)
				{
					hist_x = BallFit_x.getY(_time);
					prePoint.x = hist_x;

				}
				else
				{
					x0_flag = true;
					prePoint.x = hist_x;
				}
				if ((BallFit_z.getY(0.f) - BallFit_z.getY(0.02))*(BallFit_z.getY(_time) - BallFit_z.getY(_time + 0.02)) > 0
					&& !z0_flag)
				{
					hist_z = BallFit_z.getY(_time);
					prePoint.z = hist_z;

				}
				else
				{
					z0_flag = true;
					prePoint.z = hist_z;
				}

				cout << prePoint << endl;

				Point3f _globalPoints(
					transToGlobal[0] * prePoint.x + transToGlobal[1] * prePoint.y + transToGlobal[2] * prePoint.z - 3.f,
					transToGlobal[4] * prePoint.x + transToGlobal[5] * prePoint.y + transToGlobal[6] * prePoint.z - 4.11f,
					transToGlobal[8] * prePoint.x + transToGlobal[9] * prePoint.y + transToGlobal[10] * prePoint.z + 0.94f
				);

				mySiteIFM.preBallPoints.push_back(prePoint);
			}*/

			/***********************************************************************************/
			bool goodshort = false;
			uchar LRdeviation = 0;
			uchar UDdeviation = 0;
			double x, y, z;
			double vx, vy, vz;
			double tvx, tvy, tvz;
			double k1[3], k2[3], k3[3], k4[3], f[3];
			double ball_vv, ball_v, temp;

			x = BallKalman->KF->statePost.ptr<double>(0)[0];
			y = BallKalman->KF->statePost.ptr<double>(0)[1];
			z = BallKalman->KF->statePost.ptr<double>(0)[2];
			vx = BallKalman->KF->statePost.ptr<double>(0)[3];
			vy = BallKalman->KF->statePost.ptr<double>(0)[4];
			vz = BallKalman->KF->statePost.ptr<double>(0)[5];

			double t = 0.001;
			while (t < 0.8)
			{
				int i = 0;
				//Ff(vx, vy, vz, f);
				Ff(vx, vy, vz);

				for (i = 0; i <= 2; i++)
				{
					k1[i] = f[i];
				}

				tvx = vx + k1[0] * t / 2;
				tvy = vy + k1[1] * t / 2;
				tvz = vz + k1[2] * t / 2;

				if (vx*tvx < 0) tvx = 0;
				if (vy*tvy < 0) tvy = 0;

				//Ff(tvx, tvy, tvz, f);
				Ff(tvx, tvy, tvz);

				for (i = 0; i <= 2; i++)
				{
					k2[i] = f[i];
				}

				tvx = vx + k2[0] * t / 2;
				tvy = vy + k2[1] * t / 2;
				tvz = vz + k2[2] * t / 2;

				if (vx*tvx < 0) tvx = 0;
				if (vy*tvy < 0) tvy = 0;

				//Ff(tvx, tvy, tvz, f);
				Ff(tvx, tvy, tvz);

				for (i = 0; i <= 2; i++)
				{
					k3[i] = f[i];
				}

				tvx = vx + k3[0] * t;
				tvy = vy + k3[1] * t;
				tvz = vz + k3[2] * t;

				if (vx*tvx < 0) tvx = 0;
				if (vy*tvy < 0) tvy = 0;

				//Ff(tvx, tvy, tvz, f);
				Ff(tvx, tvy, tvz);

				for (i = 0; i <= 2; i++)
				{
					k4[i] = f[i];
				}
				tvx = vx + (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])*t / 6;
				tvy = vy + (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])*t / 6;
				tvz = vz + (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])*t / 6;

				if (vx*tvx < 0) tvx = 0;
				if (vy*tvy < 0) tvy = 0;

				x = x + (vx + tvx)*t / 2;
				y = y + (vy + tvy)*t / 2;
				z = z + (vz + tvz)*t / 2;

				vx = tvx; vy = tvy; vz = tvz;
				t += 0.0003;

				//cout << x << endl;
				//cout << "ball_vv:" << ball_vv << endl;


				float _distance = (y*sin(ElevationTheta) + z*cos(ElevationTheta))*1000.0;
				if (_distance > DisControl[0] &&
					_distance < DisControl[1])
				{
					BallKinect.kinectDo(MV_KINECT_DOCOLOR);
					Mat kinect_color, color_resize, color_b, color_r;
					Mat _canny;
					kinect_color = BallKinect.getBGR();
					resize(kinect_color, color_resize, Size(1920 / 4, 1020 / 4));
					my_split(color_resize, color_b, MV_GET_B);
					my_split(color_resize, color_r, MV_GET_R);
					_canny = color_b - color_r;
					Canny(_canny, _canny, 47, 47 * 3, 3);
					Canny(_canny, _canny, 47, 47 * 3, 3);
					Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
					morphologyEx(_canny, _canny, MORPH_CLOSE, element);
					imshow("canny", _canny);
					vector<vector<Point>>_points;
					findContours(_canny, _points, RETR_LIST, CHAIN_APPROX_SIMPLE);


					Point3d _srcPoint = Point3d(x, y, z);
					Point2f _dstPoint = Point2f(0.f, 0.f);
					Point2f _colorPoint = Point2f(0.f, 0.f);
					Point2f _colorMaxY = Point2f(0.f, 0.f);
					posTo2D(_srcPoint, _dstPoint);
					_colorPoint = BallKinect.kinect_IR2COLOR(_dstPoint, _srcPoint.z);
					_colorMaxY = BallKinect.kinect_IR2COLOR(Point2f(kinectIrWidth - maxY_point.x, maxY_point.y), MaY_depth);
					_colorPoint.x = (kinectColorWidth - _colorPoint.x) / 4.f;
					_colorPoint.y = _colorPoint.y / 4.f;
					_colorMaxY.x = (kinectColorWidth - _colorMaxY.x) / 4.f;
					_colorMaxY.y = _colorMaxY.y / 4.f;
					circle(color_resize, _colorPoint, 5, Scalar(0, 255, 0), -1, 2);
					//circle(color_resize, _colorMaxY, 5, Scalar(0, 0, 255), -1, 2);
					//circle(color_resize, Point2f(_colorMaxY.x - 35, _colorMaxY.y), 5, Scalar(0, 0, 255), -1, 2);
					//circle(color_resize, Point2f(_colorMaxY.x, _colorMaxY.y - 85), 5, Scalar(0, 0, 255), -1, 2);
					imshow("color_resize", color_resize);
					int64 circlePointsNum = 0;
					int left_num = 0, right_num = 0, up_num = 0, down_num = 0;
					for (size_t i = 0; i < _points.size(); i++)
					{
						if (_points[i].size() > 20)
						{
							circlePointsNum += _points[i].size();
							for (size_t j = 0; j < _points[i].size(); j++)
							{
								if (_points[i][j].x > _colorMaxY.x - 35
									&& _points[i][j].x<_colorMaxY.x + 35
									&& _points[i][j].y>_colorMaxY.y - 85
									&& _points[i][j].y < _colorMaxY.y)
								{
									if (_colorPoint.x < _points[i][j].x)
									{
										left_num++;
									}
									else
									{
										right_num++;
									}

									if (_colorPoint.y > _points[i][j].y)
									{
										up_num++;
									}
									else
									{
										down_num++;
									}
								}
							}
						}
					}

					cout << "left:		" << left_num << "right:		" << right_num << "		";
					cout << "up:		" << up_num << "down:		" << down_num << "		" << endl;

					if (_dstPoint.y > CirclePixels[0]
						&& _dstPoint.y<CirclePixels[1]
						&& _dstPoint.x>CirclePixels[2]
						&& _dstPoint.x < CirclePixels[3])
					{
						goodshort = true;
					}
					else
					{
						//上下判断
						if (_dstPoint.y < CirclePixels[0])
						{
							UDdeviation = Udeviation;
						}
						else if (_dstPoint.y > CirclePixels[1])
						{
							UDdeviation = Ddeviation;
						}
						else
						{
							UDdeviation = NOdeviation;
						}

						if (_dstPoint.x < CirclePixels[2])
						{
							LRdeviation = Ldeviation;
						}
						else if (_dstPoint.x > CirclePixels[3])
						{
							LRdeviation = Rdeviation;
						}
						else
						{
							LRdeviation = NOdeviation;
						}
					}

					//cout << _dstPoint << "给定范围：" << CirclePixels[2] << "~" << CirclePixels[3] << "		" << CirclePixels[0] << "~" << CirclePixels[1] << endl;

				}


				Point3f _globalPoints(
					transToGlobal[0] * x + transToGlobal[1] * y + transToGlobal[2] * z + (float)CameraCoordinates.x,
					transToGlobal[4] * x + transToGlobal[5] * y + transToGlobal[6] * z + (float)CameraCoordinates.y,
					transToGlobal[8] * x + transToGlobal[9] * y + transToGlobal[10] * z + (float)CameraCoordinates.z
				);

				fs << "globalPoints_x" << _globalPoints.x;
				fs << "globalPoints_y" << _globalPoints.y;
				fs << "globalPoints_z" << _globalPoints.z;

				mySiteIFM.preBallPoints.push_back(_globalPoints);
				if (_globalPoints.z < 0)
					break;
				updateWindow(GL_SITEWINDOW);
			}

			delete BallKalman;
			BallKalman = new My3DKalman;
			OutPutBin.clear();
			OutPutErodeBin.clear();
			EachBin.clear();
			InitFlag = 0;

			cout << "maxDstY:" << (y*sin(ElevationTheta) + z*cos(ElevationTheta))*1000.0 << endl;
			cout <<"	asd:" <<DisControl[0]<<"	" << DisControl[1] << endl;

			//重新找框
			Mat src_ir_u16, _Bin;
			_Bin = Mat::zeros(Size(kinectIrWidth, kinectIrHeight), CV_8UC1);
			BallKinect.kinectDo(MV_KINECT_DOIR);
			src_ir_u16 = BallKinect.getDepth16U();
			for (int i = 0; i < kinectIrHeight; i++)
			{
				for (int j = 0; j < kinectIrWidth; j++)
				{
					if (src_ir_u16.ptr<UINT16>(i)[j] > 3200
						&& src_ir_u16.ptr<UINT16>(i)[j] < 3900)
					{
						_Bin.ptr<UINT8>(i)[j] = 255;
					}
				}
			}

			vector<vector<Point>> _points;
			int contourNum = -1;
			int maxNum = 0;
			findContours(_Bin, _points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			for (size_t i = 0; i < _points.size(); i++)
			{
				if (maxNum < _points[i].size())
				{
					maxNum = _points[i].size();
					contourNum = i;
				}
			}

			if (contourNum >= 0)
			{
				for (size_t i = 0; i < _points[contourNum].size(); i++)
				{
					Point3d _point = Point3d(0.f, 0.f, 0.f);
					if (_points[contourNum][i].y < maxY_point.y)
					{
						maxY_point.x = _points[contourNum][i].x;
						maxY_point.y = _points[contourNum][i].y;
						MaY_depth = src_ir_u16.ptr<UINT16>(maxY_point.y)[maxY_point.x];
					}
					get3D_ir(src_ir_u16.ptr<UINT16>(maxY_point.y)[maxY_point.x], Point2f(kinectIrWidth - maxY_point.x, maxY_point.y), _point);
					DisControl[0] = _point.y*sin(ElevationTheta) + _point.z*cos(ElevationTheta) - 50;
					DisControl[1] = _point.y*sin(ElevationTheta) + _point.z*cos(ElevationTheta) + 50;
				}
			}


			if (goodshort)
			{
				cout << "good shot!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
			}
			else
			{
				cout << "bad shot??????????????????????????????????????????" << endl;
				if (UDdeviation == Udeviation)
				{
					cout << "过高" << endl;
				}
				else if (UDdeviation == Ddeviation)
				{
					cout << "过低" << endl;
				}
				else if (UDdeviation == NOdeviation)
				{
					cout << "上下合适" << endl;
				}

				if (LRdeviation == Ldeviation)
				{
					cout << "过左" << endl;
				}
				else if (LRdeviation == Rdeviation)
				{
					cout << "过右" << endl;
				}
				else if (LRdeviation == NOdeviation)
				{
					cout << "左右合适" << endl;
				}
				else
				{
					cout << "预测的球没到达球框平面" << endl;
				}
			}
		}
		imshow("u16_ir", src_ir_u16);
	}

}