#include "frontposition_mr.h"

namespace mv
{
	FrontPosition_MR::FrontPosition_MR()
	{
		camera_matrix = Mat::zeros(Size(3, 3), CV_64FC1);
		dist_coeffs = Mat::zeros(Size(5, 1), CV_64FC1);
		obj.push_back(Point3f(0.0, 0.0, 0.0));
		obj.push_back(Point3f(57.0, 0.0, 0.0));
		obj.push_back(Point3f(0.0, 55.0, 0.0));
		obj.push_back(Point3f(57.0, 55.0, 0.0));
	
		camera_matrix.ptr<double>(0)[0] = 4.8036664410376443e+02;
		camera_matrix.ptr<double>(0)[2] = 3.1819829459137924e+02;
		camera_matrix.ptr<double>(1)[1] = 4.8008319528216572e+02;
		camera_matrix.ptr<double>(1)[2] = 2.7007152279279899e+02;
		camera_matrix.ptr<double>(2)[2] = 1.0;
	
		dist_coeffs.ptr<double>(0)[0] = -4.1054858804134586e-01;
		dist_coeffs.ptr<double>(0)[1] = 8.7480466272727320e-02;
		dist_coeffs.ptr<double>(0)[2] = -2.9574061020261057e-04;
		dist_coeffs.ptr<double>(0)[3] = 6.6559279152157702e-04;
		dist_coeffs.ptr<double>(0)[4] = 3.0375922177456693e-01;
	}
	
	FrontPosition_MR::~FrontPosition_MR()
	{
		if (cap.isOpened())
		{
			cap.release();
		}
	}

	void FrontPosition_MR::GetPositionIFM(float* _IFM)
	{
		if (!cap.isOpened())
		{
			cout << "相机未打开" << endl;
			return;
		}
		int64 time0 = getTickCount();
		Mat src, resrc, gray;
		cap >> src;
		resize(src, resrc, Size(320, 240));
		cvtColor(resrc, gray, CV_BGR2GRAY);//BGR
		Mat adaptiveCalib(gray.size(), gray.type(), Scalar(255));
		adaptiveThreshold(gray, adaptiveCalib, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 9, 0.0);

		Mat doContour;
		adaptiveCalib.copyTo(doContour);
		Mat element;
		element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(doContour, doContour, MORPH_OPEN, element);

		vector<vector<Point> > ROIcontour;
		vector<Point2f> usefullCenter;
		vector<float> usefullArea;
		vector<float> usefulRadius;
		findContours(doContour, ROIcontour, RETR_LIST, CHAIN_APPROX_SIMPLE);

		Mat _text = Mat::zeros(doContour.size(), CV_8UC1);
		bool contoursFlag = 0;
		int ROIcontourSize = (int)ROIcontour.size();
		for (int i = 0; i < ROIcontourSize; i++)
		{
			float _thisArea = contourArea(ROIcontour[i]);
			if (_thisArea > CV_PI * 8)
			{
				Point2f center;
				float radius;
				minEnclosingCircle(ROIcontour[i], center, radius);
				if (_thisArea / (CV_PI*radius*radius) > SCM_areaK)
				{
					usefulRadius.push_back(radius);
					usefullCenter.push_back(center);
					usefullArea.push_back(_thisArea);
					contoursFlag = 1;
					drawContours(_text, ROIcontour, i, Scalar(255), -1);
				}
			}
		}

		if (contoursFlag)
		{
			bool getTarget = false;
			int usefullAreaSize = usefullArea.size();
			Point2f targetPoints[4];
			int targetTime = 0;
			float lastRadius = 0.0;
			for (int i = 0; i < usefullAreaSize - 1; i++)
			{
				for (int j = i + 1; j < usefullAreaSize; j++)
				{
					if (abs(usefullArea[i] - usefullArea[j]) / usefullArea[i] < 0.3)
					{
						if (abs(usefullCenter[i].y - usefullCenter[j].y) < PixelsK)
						{
							float dis = sqrt((usefullCenter[i].x - usefullCenter[j].x)*(usefullCenter[i].x - usefullCenter[j].x) + (usefullCenter[i].y - usefullCenter[j].y)*(usefullCenter[i].y - usefullCenter[j].y));
							if (3.0*usefulRadius[i] < dis&&5.5*usefulRadius[i] > dis)
							{
								if (!targetTime)
								{
									lastRadius = usefulRadius[i];
									targetPoints[targetTime] = usefullCenter[i];
									targetTime++;
									targetPoints[targetTime] = usefullCenter[j];
									targetTime++;
								}
								else
								{
									targetPoints[targetTime] = usefullCenter[j];
									targetTime++;
								}

							}
						}
						else if (abs(usefullCenter[i].x - usefullCenter[j].x) < PixelsK)
						{
							float dis = sqrt((usefullCenter[i].x - usefullCenter[j].x)*(usefullCenter[i].x - usefullCenter[j].x) + (usefullCenter[i].y - usefullCenter[j].y)*(usefullCenter[i].y - usefullCenter[j].y));
							if (3.0*usefulRadius[i] < dis&&5.5*usefulRadius[i] > dis)
							{
								if (!targetTime)
								{
									lastRadius = usefulRadius[i];
									targetPoints[targetTime] = usefullCenter[i];
									targetTime++;
									targetPoints[targetTime] = usefullCenter[j];
									targetTime++;
								}
								else
								{
									targetPoints[targetTime] = usefullCenter[j];
									targetTime++;
								}

							}
						}

					}
					if (targetTime == 3)
					{
						if (abs(targetPoints[0].y - targetPoints[1].y) < PixelsK)
						{
							for (int m = 0; m < usefullAreaSize; m++)
							{
								float dst = sqrt((usefullCenter[m].x - targetPoints[1].x)*(usefullCenter[m].x - targetPoints[1].x) + (usefullCenter[m].y - targetPoints[2].y)*(usefullCenter[m].y - targetPoints[2].y));
								if (dst < lastRadius && abs(usefullArea[i] - usefullArea[m]) / usefullArea[i] < 0.3)
								{
									targetPoints[targetTime] = usefullCenter[m];
									targetTime++;
									break;
								}
							}
						}
						else if (abs(targetPoints[0].x - targetPoints[1].x) < PixelsK)
						{
							for (int m = 0; m < usefullAreaSize; m++)
							{
								float dst = sqrt((usefullCenter[m].x - targetPoints[2].x)*(usefullCenter[m].x - targetPoints[2].x) + (usefullCenter[m].y - targetPoints[1].y)*(usefullCenter[m].y - targetPoints[1].y));
								if (dst < lastRadius &&abs(usefullArea[i] - usefullArea[m]) / usefullArea[i] < 0.3)
								{
									targetPoints[targetTime] = usefullCenter[m];
									targetTime++;
									break;
								}
							}
						}
					}
					if (targetTime == 4)
						break;
				}
				if (targetTime == 4)
				{
					getTarget = true;
					break;
				}
				targetTime = 0;
				lastRadius = 0.0;
			}
			if (getTarget)
			{
				vector<Point2f> image(4);
				float minDst = targetPoints[0].x*targetPoints[0].x + targetPoints[0].y*targetPoints[0].y;
				image[0] = targetPoints[0];
				int firstPoint = 0;	//左上
				int secondPoint = 0; //右上
				int thirdPoint = 0;	 //左下
				for (int i = 1; i < 4; i++)
				{
					float dst = targetPoints[i].x*targetPoints[i].x + targetPoints[i].y*targetPoints[i].y;
					if (dst < minDst)
					{
						minDst = dst;
						firstPoint = i;
						image[0] = targetPoints[i];
					}
				}

				minDst = (320.0 - targetPoints[0].x)*(320.0 - targetPoints[0].x) + targetPoints[0].y*targetPoints[0].y;
				image[1] = targetPoints[0];
				for (int i = 1; i < 4; i++)
				{
					float dst = (320.0 - targetPoints[i].x)*(320.0 - targetPoints[i].x) + targetPoints[i].y*targetPoints[i].y;
					if (dst < minDst)
					{
						minDst = dst;
						secondPoint = i;
						image[1] = targetPoints[i];
					}
				}

				minDst = (targetPoints[0].x)*(targetPoints[0].x) + (240.0 - targetPoints[0].y)*(240 - targetPoints[0].y);
				thirdPoint = 0;
				image[2] = targetPoints[0];
				for (int i = 1; i < 4; i++)
				{
					float dst = (targetPoints[i].x)*(targetPoints[i].x) + (240.0 - targetPoints[i].y)*(240 - targetPoints[i].y);
					if (dst < minDst)
					{
						minDst = dst;
						thirdPoint = i;
						image[2] = targetPoints[i];
					}
				}

				for (int i = 0; i < 4; i++)
				{
					if (i != firstPoint&&i != secondPoint&&i != thirdPoint)
					{
						image[3] = targetPoints[i];
					}
				}

				Mat targetResult = Mat::zeros(Size(320, 240), CV_8UC1);
				for (int i = 0; i < 2; i++)
				{
					circle(targetResult, image[i], 5, Scalar(255), -1);
				}

				for (int i = 0; i < 4; i++)
				{
					image[i] = Point2f(image[i].x * 2, image[i].y * 2);
				}
				solvePnP(obj, image, camera_matrix, dist_coeffs, rv, tv, false, CV_ITERATIVE);

				double rm[9];
				Mat RoteM = cv::Mat(3, 3, CV_64FC1, rm);
				cv::Rodrigues(rv, RoteM);
				double r11 = RoteM.ptr<double>(0)[0];
				double r12 = RoteM.ptr<double>(0)[1];
				double r13 = RoteM.ptr<double>(0)[2];
				double r21 = RoteM.ptr<double>(1)[0];
				double r22 = RoteM.ptr<double>(1)[1];
				double r23 = RoteM.ptr<double>(1)[2];
				double r31 = RoteM.ptr<double>(2)[0];
				double r32 = RoteM.ptr<double>(2)[1];
				double r33 = RoteM.ptr<double>(2)[2];

				float thetaZ = atan2(r21, r11) / CV_PI * 180;
				float thetaY = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
				float thetaX = atan2(r32, r33) / CV_PI * 180;
				float _x = tv.ptr<double>(0)[0];
				float _y = tv.ptr<double>(0)[1];
				float _z = tv.ptr<double>(0)[2];
				_IFM[0] = _z*cos(thetaX);
				_IFM[1] = _x;
				_IFM[2] = thetaY;

			}
			else
			{
				_IFM[0] = 999;
			}
		}
		DSP = getTickFrequency() / (getTickCount() - time0);
		return;
	}

	void FrontPosition_MR::CloseOrOpenCamera(bool close_open, int Camera_num)
	{
		if (close_open == OpenOrCloseCamera::MV_CLOSE)
		{
			cap.release();
		}
		else
		{
			cap.open(Camera_num);
		}
		return;
	}
}