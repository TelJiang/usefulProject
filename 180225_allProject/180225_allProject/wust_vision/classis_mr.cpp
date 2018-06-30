#include "classis_mr.h"

namespace mv
{
	using namespace cv;
	using namespace std;
	GetLineIFM_MR::GetLineIFM_MR()
	{
		Flag1 = false;
		Flag2 = false;
		LeftRightOutArea = 0;
		UpDownOutArea = 0;
		Line1K = 0;
		Line2K = 0;
		Line1B = 0;
		Line2B = 0;
		Factor30MM = 54;
		FactorRange = 6;
		CenterDst = Point2f(0.f, 0.f);
		CenterRobot = Point2f(simpleCameraWidth*0.25f, simpleCameraHeight*0.25f);
		LineCordinateConvert = Straight;
		Line_X_Y_Swap = 0;
		Line_X_Contrary = 0;
		Line_Y_Contrary = 0;
		switch (LineCordinateConvert)
		{
		case Straight:Line_X_Y_Swap = 0;
			Line_X_Contrary = -1;
			Line_Y_Contrary = 1;
			break;
		case Counterclockwise90:Line_X_Y_Swap = 1;
			Line_X_Contrary = -1;
			Line_Y_Contrary = -1;
			break;
		case Clockwise90:Line_X_Y_Swap = 1;
			Line_X_Contrary = 1;
			Line_Y_Contrary = 1;
			break;
		case Counter:Line_X_Y_Swap = 0;
			Line_X_Contrary = 1;
			Line_Y_Contrary = -1;
			break;
		default:
			break;
		}
	}

	GetLineIFM_MR::~GetLineIFM_MR()
	{
		//if (cap.isOpened())
		//{
		//	cap.release();
		//}
	}

	//_IFM 1*3数组，分别为Δx Δy angel
	void GetLineIFM_MR::IFM(float *_IFM)
	{
		if (0/*!cap.isOpened()*/)
		{
			std::cout << "相机未打开。" << std::endl;
		}
		else
		{
			int64 time = cv::getTickCount();
			cv::Mat src, src_resize, dst, Bin;
			Mat for_show;
			IplImage *pFrame1 = Camera0.QueryFrame();
			src= cvarrToMat(pFrame1, true);
			src_resize = src;
			//cap >> src;
			//src=imread("wust_vision/image/7.bmp");
			//cv::resize(src, src_resize, cv::Size((int)simpleCameraWidth*resizeHalf, (int)simpleCameraHeight*resizeHalf));
			src_resize.copyTo(for_show);
			Mat image_R, image_G, image_B;
			/*my_split(src_resize, image_R, MV_GET_R);
			my_split(src_resize, image_G, MV_GET_G);
			dst = image_G - image_R;*/
			my_split(src_resize, image_R, MV_GET_R);
			my_split(src_resize, image_G, MV_GET_G);
			my_split(src_resize, image_B, MV_GET_B);
			dst = image_G - image_R;
			//my_split(src_resize, dst, MV_GET_S);
			blur(dst, dst, Size(5, 5));


			//用80填充非兴趣区域
			Mat _mask(Size(simpleCameraWidth*resizeHalf, simpleCameraHeight*resizeHalf), CV_8UC1, Scalar(80));
			_mask.copyTo(dst, dst > 20);
			_mask.copyTo(dst, image_B < 120);
			_mask.copyTo(dst, image_G < 120);
			_mask.copyTo(dst, image_R < 120);
			//for (int i = 0; i < simpleCameraHeight*resizeHalf; i++)
			//{
			//	Vec3b *p = src_resize.ptr<Vec3b>(i);
			//	for (int j = 0; j < simpleCameraWidth*resizeHalf; j++)
			//	{
			//		/*int _middle;
			//		_middle = (p[j][0] + p[j][1] + p[j][2]) / 3;
			//		int _BC;
			//		_BC = 0;
			//		for (int i = 0; i < 3; i++)
			//		{
			//			_BC += abs(p[j][i]-_middle);
			//		}*/
			//		
			//		if(p[j][1] - p[j][2]>20
			//			|| p[j][0] < 120
			//			|| p[j][1] < 120
			//			|| p[j][2] < 120)
			//		dst.ptr(i)[j] = 80;
			//		//if (p[j][0] > 120
			//		//	&& p[j][1] > 120
			//		//	&& p[j][2] > 120
			//		//	&& (abs(p[j][1] - p[j][2]) <= 15
			//		//		||abs(p[j][0]-p[j][2]<=20))
			//		//	/*&& _BC<30*/
			//		//	)
			//		//{

			//		//}
			//		//else
			//		//{
			//		//	dst.ptr(i)[j] = 80;
			//		//}
			//	}
			//}

			//threshold(dst, dst, 30, 255, THRESH_TRUNC);

			threshold(dst, Bin, 20, 255, CV_THRESH_BINARY_INV);//得二值图Bin
			Mat _element;
			_element = getStructuringElement(MORPH_RECT, Size(5, 5));
			morphologyEx(Bin, Bin, MORPH_OPEN, _element);
			Mat _forContour;
			vector<vector<Point>> _points;
			Bin.copyTo(_forContour);
			findContours(_forContour, _points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


			//搞到点数最多的轮廓，减少霍夫线的工作量。
			int contoursNum = _points.size();
			int maxPointsConNum = -1;
			int maxPoints = 300;
			for (size_t i = 0; i < contoursNum; i++)
			{
				if (_points[i].size() > maxPoints)
				{
					maxPoints = _points[i].size();
					maxPointsConNum = i;
				}
			}

			Mat lineContour(Size(simpleCameraWidth*resizeHalf, simpleCameraHeight*resizeHalf), CV_8UC1, Scalar(0));
			if (maxPointsConNum > -1)
			{
				drawContours(lineContour, _points, maxPointsConNum, Scalar(255), -1);
			}			
			/*Mat _BinForContour;
			Bin.copyTo(_BinForContour);
			vector<vector<Point>> _contourPoints;
			findContours(_BinForContour, _contourPoints, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			int _contourPoints_size = _contourPoints.size();
			float _maxArea = 0.0;
			int _contourNum = 0;
			for (int i = 0; i < _contourPoints_size; i++)
			{
				if (_contourPoints[i].size() > 500)
				{
					float area = contourArea(_contourPoints[i]);
					if (area > _maxArea)
					{
						_contourNum = i;
						_maxArea = area;
					}
				}
			}*/

			/*Mat _threshold(Size(simpleCameraWidth*resizeHalf, simpleCameraHeight*resizeHalf), CV_8UC1, Scalar(0));
			drawContours(_threshold, _contourPoints, _contourNum, Scalar(255), -1);
			imshow("???", _threshold);*/

			/*Mat _element;
			_element = getStructuringElement(MORPH_RECT, Size(5,5));
			morphologyEx(Bin, Bin, MORPH_OPEN, _element);*/
			//GaussianBlur(dst, dst, Size(5, 5), 0, 0);
			Mat canny_dst(Size(simpleCameraWidth*resizeHalf,simpleCameraHeight*resizeHalf),CV_8UC1,Scalar(0));
			//blur(dst, dst, Size(5, 5));

			Canny(lineContour, canny_dst, /*41, 121*/41, 41*3, 3);
			Canny(canny_dst, canny_dst, 41, 41*3, 3);	
			//线变化不能限制最大长度


			vector<Vec4i> lines;
			HoughLinesP(canny_dst, lines, 1, CV_PI / 180, 5, 5, 1);
			vector<Point2f> points_x;
			vector<Point2f> points_y;
			//将点分类


			int lineSize = lines.size();
			for (int i = 0; i < lineSize; i++)
			{
				Vec4i l = lines[i];

				if (l[0] != l[2])
				{
					if (abs(l[3] - l[1]) > abs(l[2] - l[0]))
					{
 						if (abs(l[3] - l[1])>35)
 						{
							char _flag_x = 0;
							char _flag_y = 0;
							if (l[0] > l[2])
								_flag_x = -1;
							else
								_flag_x = 1;
							if (l[1] > l[3])
								_flag_y = -1;
							else
								_flag_y = 1;
							float step = 15.f;
							float _x = step * abs(l[0] - l[2]) / abs(l[3] - l[1]);
							for (int i = 0,j=1; i < abs(l[3] - l[1]);)
							{
								points_x.push_back(Point2f(l[0] + _flag_x*_x*j, l[1] + _flag_y * step * j));
								j++;
								points_x.push_back(Point2f(l[0] + _flag_x*_x*j, l[1] + _flag_y * step * j));
								j++;
								i += step;
							}
 						}
						points_x.push_back(Point2f(l[0], l[1]));
						points_x.push_back(Point2f(l[2], l[3]));
					}
					else
					{
						if (abs(l[2] - l[0]) > 35)
						{
							char _flag_x = 0;
							char _flag_y = 0;
							if (l[0] > l[2])
								_flag_x = -1;
							else
								_flag_x = 1;
							if (l[1] > l[3])
								_flag_y = -1;
							else
								_flag_y = 1;
							float step = 25.f;
							float _y = step * abs(l[3] - l[1]) / abs(l[2] - l[0]);
							for (int i = 0, j = 1; i < abs(l[2] - l[0]);)
							{
								points_y.push_back(Point2f(l[0] + _flag_x*step*j, l[1] + _flag_y * _y * j));
								j++;
								points_y.push_back(Point2f(l[0] + _flag_x*step*j, l[1] + _flag_y * _y * j));
								j++;
								i += step;
							}
						}
						points_y.push_back(Point2f(l[0], l[1]));
						points_y.push_back(Point2f(l[2], l[3]));
					}
				}
				else
				{
					if (abs(l[3] - l[1]) > 35)
					{
						char _flag_x = 0;
						char _flag_y = 0;
						if (l[0] > l[2])
							_flag_x = -1;
						else
							_flag_x = 1;
						if (l[1] > l[3])
							_flag_y = -1;
						else
							_flag_y = 1;
						float step = 25.f;
						float _x = step * abs(l[0] - l[2]) / abs(l[3] - l[1]);
						for (int i = 0, j = 1; i < abs(l[3] - l[1]);)
						{
							points_x.push_back(Point2f(l[0] + _flag_x*_x*j, l[1] + _flag_y * step * j));
							j++;
							points_x.push_back(Point2f(l[0] + _flag_x*_x*j, l[1] + _flag_y * step * j));
							j++;
							i += step;
						}
					}
					points_x.push_back(Point2f(l[0], l[1]));
					points_x.push_back(Point2f(l[2], l[3]));
				}
			}

			
			//画点
			/*int point_ySize = points_y.size();
			for (int i = 0; i < point_ySize; i++)
			{
				circle(for_show, points_y[i], 4, Scalar(255, 0, 0), -1);

			}*/
			//RNG jc;
			//int point_xSize = points_x.size();
			//for (int i = 0; i < point_xSize; i++)
			//{
			//	//Scalar color = Scalar(jc.uniform(0, 255), jc.uniform(0, 255), jc.uniform(0, 255));
			//	//line(for_show, points_x[i], points_x[i + 1], color, 2);
			//	cv::circle(for_show, points_x[i], 4, Scalar(0, 0, 255), 2, -1);
			//	/*circle(for_show, points_x[i], 4, color, -1);
			//	circle(for_show, points_x[i+1], 4, color, -1);*/
			//	/*if(i%2)
			//		circle(for_show, points_x[i], 4, Scalar(0, 255, 255), -1);
			//	else
			//		circle(for_show, points_x[i], 4, Scalar(0, 0, 255), -1);*/
			//}


			solvePointsX(points_x, for_show, Bin);
			solvePointsY(points_y, for_show, Bin);


			
			Point2f _center = CenterDst;

			if (!Flag2)
			{
				if (UpDownOutArea == 0)
				{
					Line2K = 0;
					Line2B = 10;
				}
				else
				{
					Line2K = 0;
					Line2B = simpleCameraHeight*resizeHalf - 10;
				}
			}

			if (!Flag1)
			{
				if (LeftRightOutArea == 0)
				{
					CenterDst.x = 10;
					CenterDst.y = Line2K*CenterDst.x + Line2B;
				}
				else
				{
					CenterDst.x = simpleCameraWidth*resizeHalf - 10;
					CenterDst.y = Line2K*CenterDst.x + Line2B;
				}
			}
			else
			{
				CenterDst.x = (Line2B - Line1B) / (Line1K - Line2K);
				CenterDst.y = Line1K*CenterDst.x + Line1B;
			}
			Flag1 = false;
			Flag2 = false;

			if ((int)CenterDst.x < 10)
				CenterDst.x = 10;
			else if ((int)CenterDst.x > simpleCameraWidth*resizeHalf - 10)
				CenterDst.x = simpleCameraWidth*resizeHalf - 10;

			if ((int)CenterDst.y < 10)
				CenterDst.y = 10;
			else if ((int)CenterDst.y > simpleCameraHeight*resizeHalf - 10)
				CenterDst.y = simpleCameraHeight*resizeHalf - 10;

			float _deltaX, _deleteY, _angel;
			_deltaX = (CenterRobot.x - CenterDst.x)*30.f / 57.f;
			_deleteY = (CenterRobot.y - CenterDst.y)*30.f / 57.f;
			_angel = atan(Line2K)*180.f / 3.14159;

			if (_deltaX > 0)
				LeftRightOutArea = 0;
			else
				LeftRightOutArea = 1;
			if (_deleteY > 0)
				UpDownOutArea = 0;
			else
				UpDownOutArea = 1;

			CoordinateConvert(_IFM, _deltaX, _deleteY, _angel);


		/*	if (abs(_center.x - CenterDst.x) > 30 || abs(_center.y - CenterDst.y) > 30)
			{
				waitKey();
			}*/
			//cv::circle(for_show, CenterDst, 7, Scalar(255, 0, 0), -1);
			//cv::circle(for_show, CenterRobot, 7, Scalar(0, 190, 250), -1);
			cv::imshow("canny_dst", canny_dst);
			//cv::imshow("dst", dst);
			//cv::imshow("for_show", for_show);
			//cv::imshow("Bin", Bin);

			/*std::cout << "x.size():	" << points_x.size() << "   y.size():	" << points_y.size();*/
			std::cout << "	时间：" << (getTickCount() - time) / getTickFrequency() << endl;
		}
	}

	void GetLineIFM_MR::calculateMiddleLine(const vector<Point2f> &points_left, const vector<Point2f> &points_right, int _mode)
	{
		double a_left, b_left, c_left, q_left;
		double a_right, b_right, c_right, q_right;
		lineFit_X(points_left, a_left, b_left, c_left, q_left);
		lineFit_X(points_right, a_right, b_right, c_right, q_right);
		if (_mode == 1)
		{
			vector<Point2f> _points;
			for (int i = 0; i < simpleCameraHeight*resizeHalf; i += simpleCameraHeight / 6)
			{
				Point2f _point = Point2f(((-c_left - b_left*i) / a_left + (-c_right - b_right*i) / a_right) / 2.f, (float)i);
				_points.push_back(_point);
			}
			double a_last, b_last, c_last, q_last;
			lineFit_X(_points, a_last, b_last, c_last, q_last);
			Line1K = (float)-(a_last / b_last);
			Line1B = (float)-(c_last / b_last);
			Flag1 = true;
		}
		else if (_mode == 2)
		{
			vector<Point2f> _points;
			for (int i = 0; i < simpleCameraWidth*resizeHalf; i += simpleCameraWidth / 6)
			{
				Point2f _point = Point2f((float)i,((-c_left - a_left*i) / b_left + (-c_right - a_right*i) / b_right) / 2.f);
				_points.push_back(_point);
			}
			double a_last, b_last, c_last, q_last;
			lineFit_X(_points, a_last, b_last, c_last, q_last);
			Line2K = (float)-(a_last / b_last);
			Line2B = (float)-(c_last / b_last);
			Flag2 = true;
		}

	}

	void GetLineIFM_MR::solvePointsX(const std::vector<cv::Point2f> points_x, Mat& for_show, Mat& _Bin)
	{
		Mat Bin;
		_Bin.copyTo(Bin);
		vector<vector<Point2f>> points_ElsePart;
		vector<vector<Point2f>> points_UsePart;
		if (points_x.size() > 30)
		{
			points_ElsePart.push_back(points_x);
			RNG rng;
			vector<Vec4d> line_IFM;
			bool noLine_flag = false;
			int line_num = 0;
			int rng_circle_num = 0;
			int elsePartSize = points_ElsePart[line_num].size();
			int points_judge = 24;
			while (!noLine_flag&&
				elsePartSize >= points_judge)
			{

				if (!line_num)
				{
					rng_circle_num = 95;
				}
				else
				{
					rng_circle_num = 50;
				}
				for (int i = 0, j = line_num; i < rng_circle_num&&j == line_num; i++)//拟合rng_circle_num根线
				{
					vector<Point2f> point_1;
					int find_maxNum = 0;
					point_1.push_back(points_ElsePart[line_num][rng.uniform(0, elsePartSize)]);
					for (int i = 0; i < 2; )
					{
						int a = rng.uniform(0, elsePartSize);
						find_maxNum++;
						if (find_maxNum > 30)
						{
							break;
						}
						if (pow((point_1[0].x - points_ElsePart[line_num][a].x), 2) + pow(point_1[0].y - points_ElsePart[line_num][a].y, 2) < 255)
							continue;
						if (i == 1)
						{
							if (pow((point_1[1].x - points_ElsePart[line_num][a].x), 2) + pow(point_1[1].y - points_ElsePart[line_num][a].y, 2) < 255)
								continue;
						}
						point_1.push_back(points_ElsePart[line_num][a]);
						i++;
					}

					if (find_maxNum > 30)
					{
						noLine_flag = true;
						break;
					}

					double a = 0.0, b = 0.0, c = 0.0, p = 0.0;
					lineFit_X(point_1, a, b, c, p);

					if (p < 4.2)
					{
						//系数较小，是目标线的概率非常大
						//进行区域判断
						vector<Point2f> points_judgeLine;//judge line with this values' size
						vector<Point2f> points_else;
						for (int i = 0; i < elsePartSize; i+=2)
						{
							float _middle = (-c - b*points_ElsePart[line_num][i].y) / a;
							if (points_ElsePart[line_num][i].x > _middle - FactorRange && points_ElsePart[line_num][i].x < _middle + FactorRange)
							{
								points_judgeLine.push_back(points_ElsePart[line_num][i]);
								points_judgeLine.push_back(points_ElsePart[line_num][i+1]);
							}
							else
							{
								points_else.push_back(points_ElsePart[line_num][i]);
								points_else.push_back(points_ElsePart[line_num][i+1]);
							}
						}
						if (points_judgeLine.size() >= points_judge)//区域内有多余30个点存在,此处100%是目标线(至少得这样认为)
						{
							Vec4d line_x;
							line_x[0] = a;
							line_x[1] = b;
							line_x[2] = c;
							line_x[3] = p;
							line_IFM.push_back(line_x);
							points_ElsePart.push_back(points_else);
							points_UsePart.push_back(points_judgeLine);
							//cout << "i:	" << i << "		" << "find_maxNum:	" << find_maxNum << endl;
							line_num++;
							elsePartSize = points_ElsePart[line_num].size();
						}
					}
					if (i == rng_circle_num - 1)
					{
						//没线集合了
						noLine_flag = true;
					}
				}

			}

			int lineIFMSize = line_IFM.size();
			cout << "X---line_IFM.size():	" << lineIFMSize << endl;
			for (int i = 0; i < lineIFMSize; i++)
			{
				//line(for_show, Point2f((-line_IFM[i][2]) / line_IFM[i][0], 0.0), Point2f(((-line_IFM[i][2] - line_IFM[i][1] * 240) / line_IFM[i][0]), 240.0f), Scalar(0, 180, 250), 3);
				//line(for_show, Point2f((-line_IFM[i][2]) / line_IFM[i][0] + FactorRange, 0.0), Point2f(((-line_IFM[i][2] - line_IFM[i][1] * 240) / line_IFM[i][0]) + FactorRange, 240.0f), Scalar(200, 180, 250), 1);
				//line(for_show, Point2f((-line_IFM[i][2]) / line_IFM[i][0] - FactorRange, 0.0), Point2f(((-line_IFM[i][2] - line_IFM[i][1] * 240) / line_IFM[i][0]) - FactorRange, 240.0f), Scalar(200, 180, 250), 1);
			}

			//只要有线，就必须保证能检测出来，然后下一步		
			if (lineIFMSize >= 2)
			{
				float _minArea = Factor30MM * simpleCameraHeight * resizeHalf * 0.5f;
				for (int i = 0; i < lineIFMSize; i++)
				{
					float up_1 = 0.f, down_1 = 0.f;
					up_1 = (-line_IFM[i][2]) / line_IFM[i][0];
					down_1 = (-line_IFM[i][2] - line_IFM[i][1] * 240) / line_IFM[i][0];
					for (int j = i + 1; j < lineIFMSize; j++)
					{
						float up_2 = 0.f, down_2 = 0.f;
						up_2 = (-line_IFM[j][2]) / line_IFM[j][0];
						down_2 = (-line_IFM[j][2] - line_IFM[j][1] * 240) / line_IFM[j][0];
						//左右之分
						if (//up_1 - up_2<Factor30MM + FactorRange*4.f
							//&&down_1 - down_2<Factor30MM + FactorRange*4.f
							/*&&*/ up_1 - up_2>Factor30MM / 2.f
							&&down_1 - down_2>Factor30MM / 2.f)
						{
							//cout << "up_1: is right !" << endl;
							line(Bin, Point2f((-line_IFM[i][2]) / line_IFM[i][0], 0.0), Point2f(((-line_IFM[i][2] - line_IFM[i][1] * 240) / line_IFM[i][0]), 240.0f), Scalar(0), 1);
							line(Bin, Point2f((-line_IFM[j][2]) / line_IFM[j][0], 0.0), Point2f(((-line_IFM[j][2] - line_IFM[j][1] * 240) / line_IFM[j][0]), 240.0f), Scalar(0), 1);
							//腐蚀
							Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
							erode(Bin, Bin, element);

							Mat forContour;
							Bin.copyTo(forContour);
							vector<vector<Point>> contour_Points;
							findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
							int contourSize = contour_Points.size();
							for (int m = 0; m < contourSize; m++)
							{
								Moments mu;
								Point2f center;
								float area = contourArea(contour_Points[m]);
								if (area > _minArea)
								{
									mu = moments(contour_Points[m], false);
									center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
									if (center.x > (-line_IFM[j][2] - line_IFM[j][1] * center.y) / line_IFM[j][0]
										&& center.x < (-line_IFM[i][2] - line_IFM[i][1] * center.y) / line_IFM[i][0])
									{
										calculateMiddleLine(points_UsePart[j], points_UsePart[i], 1);
										//cout << "nice!" << endl;
									}
									cv::circle(Bin, center, 5, Scalar(150), -1);
								}

							}

						}
						else if (//fabs(up_1 - up_2) < Factor30MM + FactorRange*4.f
							//&&fabs(down_1 - down_2) < Factor30MM + FactorRange*4.f
							/*&&*/ fabs(up_1 - up_2) > Factor30MM / 2.f
							&&fabs(down_1 - down_2) > Factor30MM / 2.f)
						{
							//cout << "up_2: is right !" << endl;
							line(Bin, Point2f((-line_IFM[i][2]) / line_IFM[i][0], 0.0), Point2f(((-line_IFM[i][2] - line_IFM[i][1] * 240) / line_IFM[i][0]), 240.0f), Scalar(0), 1);
							line(Bin, Point2f((-line_IFM[j][2]) / line_IFM[j][0], 0.0), Point2f(((-line_IFM[j][2] - line_IFM[j][1] * 240) / line_IFM[j][0]), 240.0f), Scalar(0), 1);
							//腐蚀
							Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
							erode(Bin, Bin, element);

							Mat forContour;
							Bin.copyTo(forContour);
							vector<vector<Point>> contour_Points;
							findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
							int contourSize = contour_Points.size();
							for (int m = 0; m < contourSize; m++)
							{
								Moments mu;
								Point2f center;
								float area = contourArea(contour_Points[m]);
								if (area > _minArea)
								{
									mu = moments(contour_Points[m], false);
									center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
									if (center.x < (-line_IFM[j][2] - line_IFM[j][1] * center.y) / line_IFM[j][0]
										&& center.x >(-line_IFM[i][2] - line_IFM[i][1] * center.y) / line_IFM[i][0])
									{
										calculateMiddleLine(points_UsePart[i], points_UsePart[j], 1);
										//cout << "nice!" << endl;
									}
									cv::circle(Bin, center, 5, Scalar(150), -1);
								}

							}
						}

						//cout << "up_1 - up_2:	" << up_1 - up_2 << endl;
						//cout << "down_1 - down_2:	" << down_1 - down_2 << endl;

					}
				}
			}
			else if (lineIFMSize == 1)
			{
				float _minArea = 20 * simpleCameraHeight * resizeHalf * 0.4f;
				float up_1 = 0.f, down_1 = 0.f;
				up_1 = (-line_IFM[0][2]) / line_IFM[0][0];
				down_1 = (-line_IFM[0][2] - line_IFM[0][1] * 240) / line_IFM[0][0];
				//左右之分
				if (up_1 < simpleCameraWidth * resizeHalf*0.5)
				{
					//左半
					line(Bin, Point2f((-line_IFM[0][2]) / line_IFM[0][0], 0.0), Point2f(((-line_IFM[0][2] - line_IFM[0][1] * 240) / line_IFM[0][0]), 240.0f), Scalar(0), 1);
					line(Bin, Point2f((-line_IFM[0][2]) / line_IFM[0][0] - 20, 0.0), Point2f(((-line_IFM[0][2] - line_IFM[0][1] * 240) / line_IFM[0][0]) - 20, 240.0f), Scalar(0), 1);
					//腐蚀
					Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
					erode(Bin, Bin, element);

					Mat forContour;
					Bin.copyTo(forContour);
					vector<vector<Point>> contour_Points;
					findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
					int contourSize = contour_Points.size();
					for (int m = 0; m < contourSize; m++)
					{
						Moments mu;
						Point2f center;
						float area = contourArea(contour_Points[m]);
						if (area > _minArea)
						{
							mu = moments(contour_Points[m], false);
							center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
							if (center.x > (-line_IFM[0][2] - line_IFM[0][1] * center.y) / line_IFM[0][0] - 20
								&& center.x < (-line_IFM[0][2] - line_IFM[0][1] * center.y) / line_IFM[0][0])
							{
								//cout << "nice!" << endl;
								double a_last, b_last, c_last, q_last;
								lineFit_X(points_UsePart[0], a_last, b_last, c_last, q_last);
								Line1K = (float)-(a_last / b_last);
								Line1B = (float)-(c_last / b_last) + Line1K*Factor30MM*0.5;
								Flag1 = true;
								cv::circle(Bin, center, 5, Scalar(150), -1);
								//cout << "???????????????????????" << endl;
							}
							
						}

					}
				}
				else
				{
					//右半
					line(Bin, Point2f((-line_IFM[0][2]) / line_IFM[0][0], 0.0), Point2f(((-line_IFM[0][2] - line_IFM[0][1] * 240) / line_IFM[0][0]), 240.0f), Scalar(0), 1);
					line(Bin, Point2f((-line_IFM[0][2]) / line_IFM[0][0] + 20, 0.0), Point2f(((-line_IFM[0][2] - line_IFM[0][1] * 240) / line_IFM[0][0]) + 20, 240.0f), Scalar(0), 1);
					//腐蚀
					Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
					erode(Bin, Bin, element);

					Mat forContour;
					Bin.copyTo(forContour);
					vector<vector<Point>> contour_Points;
					findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
					int contourSize = contour_Points.size();
					for (int m = 0; m < contourSize; m++)
					{
						Moments mu;
						Point2f center;
						float area = contourArea(contour_Points[m]);
						if (area > _minArea)
						{
							mu = moments(contour_Points[m], false);
							center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
							if (center.x < (-line_IFM[0][2] - line_IFM[0][1] * center.y) / line_IFM[0][0] + 20
								&& center.x >(-line_IFM[0][2] - line_IFM[0][1] * center.y) / line_IFM[0][0])
							{
								double a_last, b_last, c_last, q_last;
								lineFit_X(points_UsePart[0], a_last, b_last, c_last, q_last);
								Line1K = (float)-(a_last / b_last);
								Line1B = (float)-(c_last / b_last) - Line1K*Factor30MM*0.5;
								Flag1 = true;
								//cout << "nice!" << endl;
							}
							//circle(Bin, center, 5, Scalar(150), -1);
						}

					}
				}
			}
			//imshow("__Bin", Bin);
		}
	}
 
 	void GetLineIFM_MR::solvePointsY(const std::vector<cv::Point2f> points_y, Mat& for_show, Mat& _Bin)
 	{
 		Mat Bin;
 		_Bin.copyTo(Bin);
 		vector<vector<Point2f>> points_ElsePart;
 		vector<vector<Point2f>> points_UsePart;
 		if ((int)points_y.size() > 40)
 		{
 			points_ElsePart.push_back(points_y);
 			RNG rng;
 			vector<Vec4d> line_IFM;
 			bool noLine_flag = false;
 			int line_num = 0;
 			int rng_circle_num = 0;
			int elsePartSize = points_ElsePart[line_num].size();
			int points_judge = 30;
 			while (!noLine_flag && 
				elsePartSize >= points_judge)
 			{
 				if (!line_num)
 				{
 					rng_circle_num = 95;
 				}
 				else
 				{
 					rng_circle_num = 50;
 				}
 				for (int i = 0, j = line_num; i < rng_circle_num&&j == line_num; i++)//拟合95根线}
 				{
 					vector<Point2f> point_1;
 					int find_maxNum = 0;
 					point_1.push_back(points_ElsePart[line_num][rng.uniform(0, elsePartSize)]);
 					for (int i = 0; i < 2; )
 					{
 						int a = rng.uniform(0, elsePartSize);
 						find_maxNum++;
 						if (find_maxNum > 30)
 						{
 							break;
 						}
 						if (pow((point_1[0].x - points_ElsePart[line_num][a].x), 2) + pow(point_1[0].y - points_ElsePart[line_num][a].y, 2) < 255)
 							continue;
 						if (i == 1)
 						{
 							if (pow((point_1[1].x - points_ElsePart[line_num][a].x), 2) + pow(point_1[1].y - points_ElsePart[line_num][a].y, 2) < 255)
 								continue;
 						}
 						point_1.push_back(points_ElsePart[line_num][a]);
 						i++;
 					}
 
 					if (find_maxNum > 30)
 					{
						noLine_flag = true;
 						break;
 					}
 
 					double a = 0.0, b = 0.0, c = 0.0, p = 0.0;
 					lineFit_Y(point_1, a, b, c, p);
 
 					if (p < 4.2)
 					{
 						//系数较小，是目标线的概率非常大
 						//进行区域判断
 						vector<Point2f> points_judgeLine;//judge line with this values' size
 						vector<Point2f> points_else;
 						for (int i = 0; i < elsePartSize; i+=2)
 						{
							float _middle = (-c - a*points_ElsePart[line_num][i].x) / b;
 							if (points_ElsePart[line_num][i].y > _middle - FactorRange && points_ElsePart[line_num][i].y < _middle + FactorRange)
 							{
 								points_judgeLine.push_back(points_ElsePart[line_num][i]);
								points_judgeLine.push_back(points_ElsePart[line_num][i + 1]);
 							}
 							else
 							{
 								points_else.push_back(points_ElsePart[line_num][i]);
								points_else.push_back(points_ElsePart[line_num][i + 1]);
 							}
 						}
 
 						if (points_judgeLine.size() > points_judge)//区域内有多余15个点存在,此处100%是目标线(至少得这样认为)
 						{
							//cout << "???????????????????" << endl;
 							Vec4d line_x;
 							line_x[0] = a;
 							line_x[1] = b;
 							line_x[2] = c;
 							line_x[3] = p;
 							line_IFM.push_back(line_x);
 							points_ElsePart.push_back(points_else);
 							points_UsePart.push_back(points_judgeLine);
 							line_num++;
							elsePartSize = points_ElsePart[line_num].size();
 						}
 					}
 					if (i == rng_circle_num - 1)
 					{
 						//没线集合了
 						noLine_flag = true;
 					}
 				}
 
 			}


			int lineIFMSize = line_IFM.size();
			cout << "Y--line_IFM.size():	" << lineIFMSize << endl;
 			/*for (int i = 0; i < lineIFMSize; i++)
 			{
 				line(for_show, Point2f(0.0, (-line_IFM[i][2]) / line_IFM[i][1]), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[i][0] * simpleCameraWidth*resizeHalf - line_IFM[i][2]) / line_IFM[i][1]), Scalar(186, 89, 201), 3);
 				line(for_show, Point2f(0.0, (-line_IFM[i][2]) / line_IFM[i][1] + FactorRange), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[i][0] * simpleCameraWidth*resizeHalf - line_IFM[i][2]) / line_IFM[i][1] + FactorRange), Scalar(186, 89, 201), 1);
 				line(for_show, Point2f(0.0, (-line_IFM[i][2]) / line_IFM[i][1] - FactorRange), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[i][0] * simpleCameraWidth*resizeHalf - line_IFM[i][2]) / line_IFM[i][1] - FactorRange), Scalar(186, 89, 201), 1);
 			}
 */
 			//只要有线，就必须保证能检测出来，然后下一步
 			if (lineIFMSize >= 2)
 			{
				float _minArea = Factor30MM * simpleCameraWidth * resizeHalf * 0.5f;
 				for (int i = 0; i < lineIFMSize; i++)
 				{
 					float left_1 = 0.f, right_1 = 0.f;
 					left_1 = (-line_IFM[i][2]) / line_IFM[i][1];
 					right_1 = (-line_IFM[i][2] - line_IFM[i][0] * simpleCameraWidth*resizeHalf) / line_IFM[i][1];
 					for (int j = i + 1; j < lineIFMSize; j++)
 					{
 						float left_2 = 0.f, right_2 = 0.f;
 						left_2 = (-line_IFM[j][2]) / line_IFM[j][1];
 						right_2 = (-line_IFM[j][2] - line_IFM[j][0] * simpleCameraWidth*resizeHalf) / line_IFM[j][1];
 						//上下之分
 						if (//left_1 - left_2<Factor30MM + FactorRange*4.f
 							//&&right_1 - right_2<Factor30MM + FactorRange*4.f
 							/*&&*/ left_1 - left_2>Factor30MM / 2.f
 							&&right_1 - right_2>Factor30MM / 2.f)
 						{
 							//cout << "left_1: is down !" << endl;
 							line(Bin, Point2f(0.0, (-line_IFM[i][2]) / line_IFM[i][1]), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[i][0] * simpleCameraWidth*resizeHalf - line_IFM[i][2]) / line_IFM[i][1]), Scalar(0), 1);
 							line(Bin, Point2f(0.0, (-line_IFM[j][2]) / line_IFM[j][1]), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[j][0] * simpleCameraWidth*resizeHalf - line_IFM[j][2]) / line_IFM[j][1]), Scalar(0), 1);
 							//腐蚀
 							Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
 							erode(Bin, Bin, element);
 
 							Mat forContour;
 							Bin.copyTo(forContour);
 							vector<vector<Point>> contour_Points;
 							findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
							int contourSize = contour_Points.size();
							for (int m = 0; m < contourSize; m++)
 							{
 								Moments mu;
 								Point2f center;
 								float area = contourArea(contour_Points[m]);
 								if (area > _minArea)
 								{
 									mu = moments(contour_Points[m], false);
 									center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
 									if (center.y > (-line_IFM[j][2] - line_IFM[j][0] * center.x) / line_IFM[j][1]
 										&& center.y < (-line_IFM[i][2] - line_IFM[i][0] * center.x) / line_IFM[i][1])
 									{
 										calculateMiddleLine(points_UsePart[j], points_UsePart[i], 2);
										//circle(Bin, center, 5, Scalar(150), -1);
										//cout << "nice!" << endl;
 									}
									
 								}
 
 							}
 
 						}
 						else if (//fabs(left_1 - left_2) < Factor30MM + FactorRange*4.f
 							//&&fabs(right_1 - right_2) < Factor30MM + FactorRange*4.f
 							/*&&*/ fabs(left_1 - left_2) > Factor30MM / 2.f
 							&&fabs(right_1 - right_2) > Factor30MM / 2.f)
 						{
 							//cout << "up_2: is right !" << endl;
 							line(Bin, Point2f(0.0, (-line_IFM[i][2]) / line_IFM[i][1]), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[i][0] * simpleCameraWidth*resizeHalf - line_IFM[i][2]) / line_IFM[i][1]), Scalar(0), 1);
 							line(Bin, Point2f(0.0, (-line_IFM[j][2]) / line_IFM[j][1]), Point2f(simpleCameraWidth*resizeHalf, (-line_IFM[j][0] * simpleCameraWidth*resizeHalf - line_IFM[j][2]) / line_IFM[j][1]), Scalar(0), 1);
 							//腐蚀
 							Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
 							erode(Bin, Bin, element);
 
 							Mat forContour;
 							Bin.copyTo(forContour);
 							vector<vector<Point>> contour_Points;
 							findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
							int contourSize = contour_Points.size();
							for (int m = 0; m < contourSize; m++)
 							{
 								Moments mu;
 								Point2f center;
 								float area = contourArea(contour_Points[m]);
 								if (area > _minArea)
 								{
 									mu = moments(contour_Points[m], false);
 									center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
 									if (center.y < (-line_IFM[j][2] - line_IFM[j][0] * center.x) / line_IFM[j][1]
 										&& center.y >(-line_IFM[i][2] - line_IFM[i][0] * center.x) / line_IFM[i][1])
 									{
										calculateMiddleLine(points_UsePart[i], points_UsePart[j], 2);
										//circle(Bin, center, 5, Scalar(150), -1);
										//cout << "nice!" << endl;
 									}	
 									
 								}
 
 							}
 						}
						//cout << "left1-left2:	" << left_1 - left_2 << endl;
						//cout << "right1-right2:	" << right_1 - right_2 << endl;
 					}
 				}
 			}
 			else if (lineIFMSize == 1)
 			{
				float _minArea = 20 * simpleCameraHeight * resizeHalf * 0.4f;
 				float left_1 = 0.f, right_1 = 0.f;
 				left_1 = (-line_IFM[0][2]) / line_IFM[0][1];
				//cout << "left_1:	" << left_1<<endl;
 				right_1 = (-line_IFM[0][2] - line_IFM[0][0] * simpleCameraWidth*resizeHalf) / line_IFM[0][1];
 				//上之分
 				if (left_1 < simpleCameraHeight*resizeHalf*0.5)
 				{
 					//上半
 					line(Bin, Point2f(0.f, (-line_IFM[0][2]) / line_IFM[0][1]), Point2f(simpleCameraWidth*resizeHalf, ((-line_IFM[0][2] - line_IFM[0][0] * simpleCameraWidth*resizeHalf) / line_IFM[0][1])), Scalar(0), 1);
 					line(Bin, Point2f(0.f, (-line_IFM[0][2]) / line_IFM[0][1] - 20), Point2f(simpleCameraWidth*resizeHalf, ((-line_IFM[0][2] - line_IFM[0][0] * simpleCameraWidth*resizeHalf) / line_IFM[0][1]) - 20), Scalar(0), 1);
 					//腐蚀
 					Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
 					erode(Bin, Bin, element);
 
 					Mat forContour;
 					Bin.copyTo(forContour);
 					vector<vector<Point>> contour_Points;
 					findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
					int contourSize = contour_Points.size();
 					for (int m = 0; m < contourSize; m++)
 					{
 						Moments mu;
 						Point2f center;
 						float area = contourArea(contour_Points[m]);
 						if (area > _minArea)
 						{
 							mu = moments(contour_Points[m], false);
 							center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
 							if (center.y > (-line_IFM[0][2] - line_IFM[0][0] * center.x) / line_IFM[0][1] - 20
 								&& center.y < (-line_IFM[0][2] - line_IFM[0][0] * center.x) / line_IFM[0][1])
 							{
 								double a_last, b_last, c_last, q_last;
 								lineFit_X(points_UsePart[0], a_last, b_last, c_last, q_last);
 								Line2K = (float)-(a_last / b_last);
 								Line2B = (float)-(c_last / b_last) - Factor30MM*0.5;
 								Flag2 = true;
								
 							}
 							circle(Bin, center, 5, Scalar(150), -1);
							//cout << "nice!" << endl;
 						}
 
 					}
 				}
 				else
 				{
 					//下半
 					line(Bin, Point2f(0.f, (-line_IFM[0][2]) / line_IFM[0][1]), Point2f(simpleCameraWidth*resizeHalf, ((-line_IFM[0][2] - line_IFM[0][0] * simpleCameraWidth*resizeHalf) / line_IFM[0][1])), Scalar(0), 1);
 					line(Bin, Point2f(0.f, (-line_IFM[0][2]) / line_IFM[0][1] + 20), Point2f(simpleCameraWidth*resizeHalf, ((-line_IFM[0][2] - line_IFM[0][0] * simpleCameraWidth*resizeHalf) / line_IFM[0][1]) + 20), Scalar(0), 1);
 					//腐蚀
 					Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
 					erode(Bin, Bin, element);
 
 					Mat forContour;
 					Bin.copyTo(forContour);
 					vector<vector<Point>> contour_Points;
 					findContours(forContour, contour_Points, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
					int contourSize = contour_Points.size();
 					for (int m = 0; m < contourSize; m++)
 					{
 						Moments mu;
 						Point2f center;
 						float area = contourArea(contour_Points[m]);
 						if (area > _minArea)
 						{
 							mu = moments(contour_Points[m], false);
 							center = Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
 							if (center.y < (-line_IFM[0][2] - line_IFM[0][0] * center.x) / line_IFM[0][1] + 20
 								&& center.y >(-line_IFM[0][2] - line_IFM[0][0] * center.x) / line_IFM[0][1])
 							{
 								double a_last, b_last, c_last, q_last;
 								lineFit_X(points_UsePart[0], a_last, b_last, c_last, q_last);
 								Line2K = (float)-(a_last / b_last);
 								Line2B = (float)-(c_last / b_last) + Factor30MM*0.5;
 								Flag2 = true;
 								//cout << "nice!" << endl;
 							}
 							circle(Bin, center, 5, Scalar(150), -1);
 						}
 
 					}
 				}
 
 			}
			//imshow("????Bin", Bin);
 		}
 	}

	/*void GetLineIFM_MR::roteCoordinate(const std::vector<cv::Point2f> input_points, float input_k)
	{
		vector<Point2f> points_rote;
		int *input_size;
		float *rote_sink;
		float *rote_cosk;
		float *atank;
		*atank = atan(input_k);
		if (abs(*atank)>45)
		{
			if (*atank > 0)
			{
				*rote_sink = sin(90.f - *atank);
				*rote_cosk = cos(90.f - *atank);
			}
			else
			{
				*rote_sink = sin(-(90.f - abs(*atank)));
				*rote_cosk = cos(-(90.f - abs(*atank)));
			}
			
		}
		else
		{
			*rote_sink = sin(*atank);
			*rote_cosk = cos(*atank);
		}
		*input_size = (int)input_points.size();
		for (int i=0;i<*input_size;i++)
		{
			Point2f _point;
			_point=Point2f()
		}
	}
*/

	//输入的 delta 表示RobotPoint-DstPoint
	void GetLineIFM_MR::CoordinateConvert(float *_IFM, float Delta_X, float Delta_Y, float Angel)
	{
		//更新一下LineCordinateConvert
		switch (LineCordinateConvert)
		{
		case Straight:Line_X_Y_Swap = 0;
			Line_X_Contrary = -1;
			Line_Y_Contrary = 1;
			break;
		case Counterclockwise90:Line_X_Y_Swap = 1;
			Line_X_Contrary = -1;
			Line_Y_Contrary = -1;
			break;
		case Clockwise90:Line_X_Y_Swap = 1;
			Line_X_Contrary = 1;
			Line_Y_Contrary = 1;
			break;
		case Counter:Line_X_Y_Swap = 0;
			Line_X_Contrary = 1;
			Line_Y_Contrary = -1;
			break;
		default:
			break;
		}

		if (Line_X_Y_Swap)//XY转换
		{
			_IFM[0] = Line_X_Contrary*Delta_Y;
			_IFM[1] = Line_Y_Contrary*Delta_X;
		}
		else
		{			
			_IFM[0] = Line_X_Contrary*Delta_X;
			_IFM[1] = Line_Y_Contrary*Delta_Y;
		}
		_IFM[2] = Angel;
		return;
	}

	//CO: 0 关，1 开。  变量Camera_num 代表打开哪个相机
	void GetLineIFM_MR::CloseOrOpenCamera(OpenOrCloseCamera close_open, int Camera_num)
	{
		if (close_open == OpenOrCloseCamera::MV_CLOSE)
		{
			Camera0.CloseCamera();
			//cap.release();
		}
		else
		{
			Camera0.OpenCamera(Camera_num, false);
			//cap.open(Camera_num);
		}
		return;
	}

}