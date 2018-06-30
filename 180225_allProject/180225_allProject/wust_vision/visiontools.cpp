#include "visiontools.h"
namespace mv
{
	using namespace cv;
	using namespace std;
	//通道分离
	//输入BGR，输出单通道图像，至于是什么取决于_mode
	void my_split(cv::InputArray _src, cv::OutputArray _dst, int _mode)
	{
		Mat src_HSV;
		vector<Mat> channels;
		switch (_mode)
		{
		case MV_GET_B:
			split(_src, channels);
			channels.at(0).copyTo(_dst);
			break;
		case MV_GET_G:
			split(_src, channels);
			channels.at(1).copyTo(_dst);
			break;
		case MV_GET_R:
			split(_src, channels);
			channels.at(2).copyTo(_dst);
			break;
		case MV_GET_H:
			cvtColor(_src, src_HSV, CV_BGR2HSV);
			split(src_HSV, channels);
			channels.at(0).copyTo(_dst);
			break;
		case MV_GET_S:
			cvtColor(_src, src_HSV, CV_BGR2HSV);
			split(src_HSV, channels);
			channels.at(1).copyTo(_dst);
			break;
		case MV_GET_V:
			cvtColor(_src, src_HSV, CV_BGR2HSV);
			split(src_HSV, channels);
			channels.at(2).copyTo(_dst);
			break;
		default:
			break;
		}
	}

	bool lineFit_X(const vector<Point2f> &points, double &a, double &b, double &c, double &_x_Dxx)
	{
		//ax+by+c=0;
		int size = points.size();
		_x_Dxx = 0.0;
		if (size < 2)
		{
			a = 0;
			b = 0;
			c = 0;
			return false;
		}
		double x_mean = 0;
		double y_mean = 0;
		/*double xy_mean = 0;*/
		for (int i = 0; i < size; i++)
		{
			x_mean += points[i].x;
			y_mean += points[i].y;
		}
		x_mean /= size;
		y_mean /= size; //至此，计算出了 x y 的均值
		double Dxx = 0, Dxy = 0, Dyy = 0;

		for (int i = 0; i < size; i++)
		{
			Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
			Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
			Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
		}

		double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
		double den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
		a = Dxy / den;
		b = (lambda - Dxx) / den;
		c = -a * x_mean - b * y_mean;


		vector<double> _x;
		double _x_mean = 0.0;
		for (int i = 0; i < size; i++)
		{
			_x_mean += points[i].x - (-c - b*points[i].y) / a;
			_x.push_back(points[i].x - (-c - b*points[i].y) / a);
		}
		_x_mean /= size;

		for (int i = 0; i < size; i++)
		{
			_x_Dxx += fabs(_x[i] - _x_mean);
		}
		/*_x_Dxx /= size;*/
		return true;
	}

	bool lineFit_Y(const vector<Point2f> &points, double &a, double &b, double &c, double &_y_Dxx)
	{
		//ax+by+c=0;
		int size = points.size();
		_y_Dxx = 0.0;
		if (size < 2)
		{
			a = 0;
			b = 0;
			c = 0;
			return false;
		}
		double x_mean = 0;
		double y_mean = 0;
		/*double xy_mean = 0;*/
		for (int i = 0; i < size; i++)
		{
			x_mean += points[i].x;
			y_mean += points[i].y;
		}
		x_mean /= size;
		y_mean /= size; //至此，计算出了 x y 的均值
		double Dxx = 0, Dxy = 0, Dyy = 0;

		for (int i = 0; i < size; i++)
		{
			Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
			Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
			Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
		}

		double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
		double den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
		a = Dxy / den;
		b = (lambda - Dxx) / den;
		c = -a * x_mean - b * y_mean;


		vector<double> _y;
		double _y_mean = 0.0;
		for (int i = 0; i < size; i++)
		{
			_y_mean += points[i].y - (-c - a*points[i].x) / b;
			_y.push_back(points[i].y - (-c - a*points[i].x) / b);
		}
		_y_mean /= size;

		//均方差
		for (int i = 0; i < size; i++)
		{
			_y_Dxx += fabs(_y[i] - _y_mean);
		}
		/*_y_Dxx /= size;*/
		return true;
	}

}