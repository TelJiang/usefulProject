#pragma once
#ifndef _GLSHOW_CVAPI_
#define _GLSHOW_CVAPI_
#include <opencv2/opencv.hpp>
#include <freeglut.h>

namespace mv
{
	const int site_n = 12;	//圆环圆盘类渲染参数
	const float siteH_pillars_side = 2.4;
	const float siteH_pillars_middle = 3.4;
	const float siteH_cup = 0.32;
	const float siteDis_pillars_center = 3.265;
	const float siteDis_cup_center = 3.7;
	const GLfloat ambient_red[] = { 5.0f,0.0f,0.0f,0.0f };
	const GLfloat ambient_green[] = { 0.0f,3.0f,0.0f,0.0f };
	const GLfloat ambient_blue[] = { 0.0f,0.0f,5.0f,0.0f };
	const GLfloat ambient_golden[] = { 5.0f,5.0f,0.0f,0.0f };
	const GLfloat ambient_white[] = { 5.0f,5.0f,5.0f,0.0f };
	const GLfloat ambient_preBall[] = { 2.0f,5.0f,5.0f,0.0f };

	/////////CV->GL的数据
	struct GLSiteIFM
	{
		std::vector<cv::Point3f> BallPoints;
		std::vector<cv::Point3f> preBallPoints;
		std::vector<double> intervalTime;
		float radius = 8;
		float x = radius;
		float y = 0;
		float z = 5;
		int angel = 90;
	};

	void glOnDraw(void *param);

	int listDisplaySite();

	int listDisplayBall();

	int listDisplayPreBall();
}


#endif