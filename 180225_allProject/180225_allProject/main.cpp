#include <iostream>
#include "wust_vision/classis_mr.h"
#include "wust_vision/balltracking.h"
using namespace cv;
int main()
{
 #if 1 //¸ú×ÙÇò
float IFM[3];
	mv::BallTrack ballTrack1;
	char photoName[40];
	while (1)
	{
		char c = waitKey(1);
		ballTrack1.IFM(IFM);
		if (c=='s')
		{
			std::cout << "save:" << ballTrack1.OutPutErodeBin.size() <<std::endl;
			for (int i = 0; i < ballTrack1.OutPutErodeBin.size()&&i<30; i++)
			{
				sprintf_s(photoName, "wust_vision/image/Bin_erode%d.bmp", i);
				imwrite(photoName, ballTrack1.OutPutErodeBin[i]); //±£´æÍ¼Æ¬
			}

			for (int i = 0; i < ballTrack1.OutPutBin.size() && i < 30; i++)
			{
				//std::cout << "yes" << std::endl;
				sprintf_s(photoName, "wust_vision/image/Bin%d.bmp", i);
				imwrite(photoName, ballTrack1.OutPutBin[i]); //±£´æÍ¼Æ¬
			}

			for (int i = 0; i < ballTrack1.EachBin.size() && i < 30; i++)
			{
				sprintf_s(photoName, "wust_vision/image/EachBin%d.bmp", i);
				imwrite(photoName, ballTrack1.EachBin[i]); //±£´æÍ¼Æ¬
			}

		}

		if (c == 27)
		{
			break;
		}
		if (c == 'c')
		{
			std::cout << "clear" << std::endl;
			ballTrack1.OutPutErodeBin.clear();
		}

	}
#endif

	#if 0 //
	mv::MyKinectSensor kinect_v2;
	namedWindow("src", 0);
	namedWindow("dst", 1);
	namedWindow("dst_canny", 1);
	namedWindow("_contour", 1);
	Mat _contour = Mat::zeros(Size(mv::kinectColorWidth/4, mv::kinectColorHeight/4), CV_8UC1);
	while (true)
	{
		Mat src, src_1, src_2, dst, dst_canny;
		kinect_v2.kinectDo(mv::MV_KINECT_DOCOLOR);
		src = kinect_v2.getBGR();

		resize(src, src, Size(mv::kinectColorWidth / 4, mv::kinectColorHeight / 4));

		mv::my_split(src, src_2, mv::MV_GET_B);
		mv::my_split(src, src_1, mv::MV_GET_R);//b-rºÜºÃ
		//g-rÒ²ok
		dst = src_2 - src_1;

		Canny(dst, dst_canny, 47, 47*3, 3);
		Canny(dst_canny, dst_canny, 47, 47 * 3, 3);
		Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
		morphologyEx(dst_canny, dst_canny, MORPH_CLOSE, element);

		imshow("123", dst_canny);
		std::vector<std::vector<cv::Point>> _points;
		findContours(dst_canny, _points, RETR_LIST, CHAIN_APPROX_SIMPLE);
		_contour = Mat::zeros(Size(mv::kinectColorWidth, mv::kinectColorHeight), CV_8UC1);
		int j = 0;
		for (size_t i=0;i<_points.size();i++)
		{
			if (_points[i].size() > 20)
			{
				drawContours(_contour, _points, i, Scalar(255), -1, 1);
				j++;
			}
			
		}
		std::cout << j << std::endl;
		//imshow("src", src);
		imshow("src", src);
		imshow("dst", dst);
		imshow("dst_canny", dst_canny);
		imshow("_contour", _contour);

		char c = waitKey(1);

		if (c == 27)
		{
			break;
		}
	}
#endif

#if 0
	mv::GetLineIFM_MR line_1;
	line_1.CloseOrOpenCamera(mv::MV_OPEN, 0);
	float IFM[3] = { 0.f };
	while (true)
	{
		line_1.IFM(IFM);
		uchar key = waitKey(1);
		if (key == 27)
			break;

	}

#endif
	return 0;
}