#pragma once
#ifndef _VIBE_H
#define  _VIBE_H

#include <iostream>  
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
namespace mv
{
	#define MV_NUM_SAMPLES 20      //ÿ�����ص����������  
	using namespace cv;

	class vibe_BGS
	{
	public:
		vibe_BGS(void);
		~vibe_BGS(void);

		void init(const Mat _image);   //��ʼ��  
		void processFirstFrame(const Mat _image);
		void testAndUpdate(const Mat _image);  //����  
		Mat getMask(void) { return m_mask; };

	private:
		const int MinMatches = 2;   //#minָ��
		const int Radius = 20; //Sqthere�뾶 
		const int SubSampleFactor = 6; //�Ӳ�������  
		Mat m_samples[MV_NUM_SAMPLES];
		Mat m_foregroundMatchCount;
		Mat m_mask;
	};
}
#endif