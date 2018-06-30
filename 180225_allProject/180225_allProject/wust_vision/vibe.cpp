#include "vibe.h"  

namespace mv
{
	using namespace std;
	using namespace cv;

	int c_xoff[9] = { -1,  0,  1, -1, 1, -1, 0, 1, 0 };  //x的邻居点  
	int c_yoff[9] = { -1,  0,  1, -1, 1, -1, 0, 1, 0 };  //y的邻居点  

	vibe_BGS::vibe_BGS(void)
	{

	}
	vibe_BGS::~vibe_BGS(void)
	{

	}

	/**************** Assign space and init ***************************/
	void vibe_BGS::init(const Mat _image)
	{
		for (int i = 0; i < MV_NUM_SAMPLES; i++)
		{
			/*m_samples[i] = Mat::zeros(_image.size(), CV_8UC1);*/
			_image.copyTo(m_samples[i]);
		}
		m_mask = Mat::zeros(_image.size(), CV_8UC1);
		m_foregroundMatchCount = Mat::zeros(_image.size(), CV_8UC1);
	}

	/**************** Init model from first frame ********************/
	void vibe_BGS::processFirstFrame(const Mat _image)
	{
		RNG rng;
		int row, col;

		for (int i = 0; i < _image.rows; i++)
		{
			for (int j = 0; j < _image.cols; j++)
			{
				for (int k = 0; k < MV_NUM_SAMPLES; k++)
				{
					// Random pick up NUM_SAMPLES pixel in neighbourhood to construct the model  
					int random = rng.uniform(0, 9);

					row = i + c_yoff[random];
					if (row < 0)
						row = 0;
					if (row >= _image.rows)
						row = _image.rows - 1;

					col = j + c_xoff[random];
					if (col < 0)
						col = 0;
					if (col >= _image.cols)
						col = _image.cols - 1;

					m_samples[k].at<uchar>(i, j) = _image.at<uchar>(row, col);
				}
			}
		}
	}

	/**************** Test a new frame and update model ********************/
	void vibe_BGS::testAndUpdate(const Mat _image)
	{
		RNG rng;
		for (int i = 0; i < _image.rows; i++)
		{
			for (int j = 0; j < _image.cols; j++)
			{
				int matches(0), count(0);
				float dist;
				//相当于遍历这20张图片，只要有两个背景历史与当前满足设置阈值，即认为是背景
				while (matches < MinMatches && count < MV_NUM_SAMPLES)
				{
					dist = abs(m_samples[count].at<uchar>(i, j) - _image.at<uchar>(i, j));
					if (dist < Radius)
						matches++;
					count++;
				}

				if (matches >= MinMatches)
				{
					// It is a background pixel  
					m_foregroundMatchCount.at<uchar>(i, j) = 0;

					// Set background pixel to 0  
					m_mask.at<uchar>(i, j) = 0;

					// 如果一个像素是背景点，那么它有 1 / defaultSubsamplingFactor 的概率去更新自己的模型样本值  
					int random = rng.uniform(0, SubSampleFactor);
					if (random == 0)
					{
						random = rng.uniform(0, MV_NUM_SAMPLES);
						m_samples[random].at<uchar>(i, j) = _image.at<uchar>(i, j);
					}

					// 同时也有 1 / defaultSubsamplingFactor 的概率去更新它的邻居点的模型样本值  
					random = rng.uniform(0, SubSampleFactor);
					if (random == 0)
					{
						int row, col;
						random = rng.uniform(0, 9);
						row = i + c_yoff[random];
						if (row < 0)
							row = 0;
						if (row >= _image.rows)
							row = _image.rows - 1;

						random = rng.uniform(0, 9);
						col = j + c_xoff[random];
						if (col < 0)
							col = 0;
						if (col >= _image.cols)
							col = _image.cols - 1;

						random = rng.uniform(0, MV_NUM_SAMPLES);
						m_samples[random].at<uchar>(row, col) = _image.at<uchar>(i, j);
					}
				}
				else
				{
					// It is a foreground pixel  
					m_foregroundMatchCount.at<uchar>(i, j)++;

					/*circle(m_mask, Point(300, 400), 10, Scalar(200), 2);
					imshow("m_mask", m_mask);*/
					// Set background pixel to 255  
					m_mask.at<uchar>(i, j) = 255;

					//如果某个像素点连续N次被检测为前景，则认为一块静止区域被误判为运动，将其更新为背景点  
					if (m_foregroundMatchCount.at<uchar>(i, j) > 2)
					{
						int random = rng.uniform(0, SubSampleFactor);
						if (random == 0)
						{
							random = rng.uniform(0, MV_NUM_SAMPLES);
							m_samples[random].at<uchar>(i, j) = _image.at<uchar>(i, j);
						}
					}
				}
			}
		}
	}
}
