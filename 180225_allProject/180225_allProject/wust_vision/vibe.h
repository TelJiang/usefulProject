#pragma once
#ifndef _VIBE_H
#define  _VIBE_H

#include <iostream>  
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
namespace mv
{
	#define MV_NUM_SAMPLES 20      //每个像素点的样本个数  
	using namespace cv;

	class vibe_BGS
	{
	public:
		vibe_BGS(void);
		~vibe_BGS(void);

		void init(const Mat _image);   //初始化  
		void processFirstFrame(const Mat _image);
		void testAndUpdate(const Mat _image);  //更新  
		Mat getMask(void) { return m_mask; };

	private:
		const int MinMatches = 2;   //#min指数
		const int Radius = 20; //Sqthere半径 
		const int SubSampleFactor = 6; //子采样概率  
		Mat m_samples[MV_NUM_SAMPLES];
		Mat m_foregroundMatchCount;
		Mat m_mask;
	};
}
#endif