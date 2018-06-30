// 180430_calibShow.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"


#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
	VideoCapture cap(0);
	FileStorage fs2("params/genius.xml", FileStorage::READ);

	Mat cameraMatrix2, distCoeffs2;
	fs2["cameraMatrix"] >> cameraMatrix2;
	fs2["cameraDistcoeff"] >> distCoeffs2;


	Mat map1, map2;
	/*initUndistortRectifyMap(cameraMatrix2, distCoeffs2, Mat(),
	getOptimalNewCameraMatrix(cameraMatrix2, distCoeffs2, Size(640, 480), 0, Size(640, 480), 0),
	Size(640, 480), CV_16SC2, map1, map2);*/
	fisheye::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, cv::Matx33d::eye(), cameraMatrix2, Size(640, 480), CV_16SC2, map1, map2);

	while (waitKey(1) != 27)
	{
		Mat image;
		cap >> image;

		Mat cImage;
		/*undistort(image, cImage, cameraMatrix2, distCoeffs2);*/

		remap(image, cImage, map1, map2, INTER_LINEAR);
		imshow("raw", image);
		imshow("corrected", cImage);

	}
	return 0;
}

