#include "My3DKalman.h"

namespace mv
{
	My3DKalman::My3DKalman()
	{
		KF = new cv::KalmanFilter(9, 9, 0, CV_64F); //kalman�˲���---״̬ά��4������ά��2��1��������
		//control = new cv::Mat(1, 1, CV_64F);//U,��֪���ⲿ����Ŀ���
		measurement = new cv::Mat(9, 1, CV_64F); //�۲���������
		kalmanT = 0.05;//����kalman������ʱ��
		//control->ptr(0)[1] = (double) 0.98;//��U���и�ֵ,�൱�ڼ��ٶ�
		KF->transitionMatrix = (cv::Mat_<double>(9, 9) << 1, 0., 0., kalmanT, 0., 0., 0.5*kalmanT*kalmanT, 0., 0.,
			0., 1., 0., 0., kalmanT, 0., 0., 0.5*kalmanT*kalmanT, 0.,
			0., 0., 1., 0., 0., kalmanT, 0., 0., 0.5*kalmanT*kalmanT,
			0., 0., 0., 1., 0., 0., kalmanT, 0., 0.,
			0., 0., 0., 0., 1., 0., 0., kalmanT, 0.,
			0, 0, 0, 0, 0, 1, 0., 0., kalmanT,
			0., 0., 0., 0., 0., 0., 1., 0., 0.,
			0., 0., 0., 0., 0., 0., 0., 1., 0.,
			0., 0., 0., 0., 0., 0., 0., 0., 1.);//��ʼ�� ״̬���ݾ���,�����󸳳�ֵ
		//KF->controlMatrix = (cv::Mat_<double>(6, 1) << 0, 0.5*kalmanT*kalmanT, 0, 0, kalmanT, 0);//���Ʒ���

		cv::setIdentity(KF->measurementMatrix); //��ʼ�� ��������
		cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(1e-1)); //ϵͳ����Э�������
		cv::setIdentity(KF->measurementNoiseCov, cv::Scalar::all(1e-1)); //��������Э�������
		cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));//��������Э�������

	}

	My3DKalman::~My3DKalman()
	{

	}

	void My3DKalman::KalmanInit(const cv::Point3d& firstPoint,const cv::Point3d& _initV)
	{
		KF->statePost.ptr<double>(0)[0] = firstPoint.x;
		KF->statePost.ptr<double>(1)[0] = firstPoint.y;
		KF->statePost.ptr<double>(2)[0] = firstPoint.z;
		KF->statePost.ptr<double>(3)[0] = _initV.x;
		KF->statePost.ptr<double>(4)[0] = _initV.y;
		KF->statePost.ptr<double>(5)[0] = _initV.z;

		double ball_vv, ball_v, temp;
		ball_vv = KF->statePost.ptr<double>(3)[0] * KF->statePost.ptr<double>(3)[0] +
			KF->statePost.ptr<double>(4)[0] * KF->statePost.ptr<double>(4)[0] +
			KF->statePost.ptr<double>(5)[0] * KF->statePost.ptr<double>(5)[0];
		ball_v = sqrt(ball_vv);
		temp = -abs((airParameter_b*ball_v) / mOfBadminton);


		KF->statePost.ptr<double>(6)[0] = KF->statePost.ptr<double>(3)[0] * temp;
		KF->statePost.ptr<double>(7)[0] = KF->statePost.ptr<double>(4)[0] * temp + G_gravity*cos(ElevationTheta);
		KF->statePost.ptr<double>(8)[0] = KF->statePost.ptr<double>(5)[0] * temp - G_gravity*sin(ElevationTheta);

	}

	void My3DKalman::DoKalman(const cv::Point3d& _postCoordinate,const cv::Point3d& _V,const double& intervalTime, cv::Point3d& _preCoordinate)
	{
// 		cv::Mat timeMatrix = (cv::Mat_<double>(6, 6) << 1, 1, 1, intervalTime, 1, 1,
// 			1, 1, 1, 1, intervalTime, 1,
// 			1, 1, 1, 1, 1, intervalTime,
// 			1, 1, 1, 1, 1, 1,
// 			1, 1, 1, 1, 1, 1,
// 			1, 1, 1, 1, 1, 1);
/*		KF->transitionMatrix = KF->transitionMatrix.mul(timeMatrix);//���*/
		double ball_vv, ball_v, temp;
		KF->transitionMatrix.ptr<double>(0)[3] = intervalTime;
		KF->transitionMatrix.ptr<double>(0)[6] = 1.0 / 2.0*intervalTime*intervalTime;
		KF->transitionMatrix.ptr<double>(1)[4] = intervalTime;
		KF->transitionMatrix.ptr<double>(1)[7] = 1.0 / 2.0*intervalTime*intervalTime;
		KF->transitionMatrix.ptr<double>(2)[5] = intervalTime;
		KF->transitionMatrix.ptr<double>(2)[8] = 1.0 / 2.0*intervalTime*intervalTime;
		KF->transitionMatrix.ptr<double>(3)[6] = intervalTime;
		KF->transitionMatrix.ptr<double>(4)[7] = intervalTime;
		KF->transitionMatrix.ptr<double>(5)[8] = intervalTime;
		KF->predict();

		measurement->ptr<double>(0)[0] = _postCoordinate.x;
		measurement->ptr<double>(1)[0] = _postCoordinate.y;
		measurement->ptr<double>(2)[0] = _postCoordinate.z;
		measurement->ptr<double>(3)[0] = _V.x;
		measurement->ptr<double>(4)[0] = _V.y;
		measurement->ptr<double>(5)[0] = _V.z;

		ball_vv = measurement->ptr<double>(3)[0] * measurement->ptr<double>(3)[0] +
			measurement->ptr<double>(4)[0] * measurement->ptr<double>(4)[0] +
			measurement->ptr<double>(5)[0] * measurement->ptr<double>(5)[0];
		ball_v = sqrt(ball_vv);
		temp = -abs((airParameter_b*ball_v) / mOfBadminton);

		measurement->ptr<double>(6)[0] = measurement->ptr<double>(3)[0] * temp;
		measurement->ptr<double>(7)[0] = measurement->ptr<double>(4)[0] * temp + G_gravity*cos(ElevationTheta);
		measurement->ptr<double>(8)[0] = measurement->ptr<double>(5)[0] * temp - G_gravity*sin(ElevationTheta);

		KF->correct(*measurement);

		_preCoordinate.x = KF->statePost.ptr<double>(0)[0];
		_preCoordinate.y = KF->statePost.ptr<double>(0)[1];
		_preCoordinate.z = KF->statePost.ptr<double>(0)[2];

// 		std::cout << "x��" << KF->statePost.ptr<double>(0)[0] << std::endl;
// 		std::cout << "y��" << KF->statePost.ptr<double>(0)[1] << std::endl;
// 		std::cout << "z��" << KF->statePost.ptr<double>(0)[2] << std::endl;
//		std::cout << "_t;	" << kalmanT << std::endl;
 		std::cout << "vx��" << KF->statePost.ptr<double>(0)[3] << "		";
 		std::cout << "vy��" << KF->statePost.ptr<double>(0)[4] << "		";
		std::cout << "vz��" << KF->statePost.ptr<double>(0)[5] << std::endl;
		return;
	}

//	void My3DKalman::prediction(const double& intervalTime, std::vector<cv::Point3d>& _preCoordinates)
//	{
//		KF->transitionMatrix.ptr<double>(0)[3] = intervalTime;
//		KF->transitionMatrix.ptr<double>(1)[4] = intervalTime;
//		KF->transitionMatrix.ptr<double>(2)[5] = intervalTime;
//		KF->predict();
//
//	/*	cv::Point3f _points = (cv::Point3f(KF->statePost.ptr<double>(0)[0],
//			KF->statePost.ptr<double>(0)[1],
//			KF->statePost.ptr<double>(0)[2]));*/
//
//		_preCoordinates.push_back(cv::Point3f(KF->statePost.ptr<double>(0)[0], KF->statePost.ptr<double>(0)[1], KF->statePost.ptr<double>(0)[2]));
//
//		measurement->ptr<double>(0)[0] = KF->statePost.ptr<double>(0)[0];
//		measurement->ptr<double>(0)[1] = KF->statePost.ptr<double>(0)[1];
//		measurement->ptr<double>(0)[2] = KF->statePost.ptr<double>(0)[2];
//		KF->correct(*measurement);
//
//		/*_preCoordinates.push_back(cv::Point3f(KF->statePost.ptr<double>(0)[0], KF->statePost.ptr<double>(0)[1], KF->statePost.ptr<double>(0)[2]));
//*/
//		//std::cout << cv::Point3f(KF->statePost.ptr<double>(0)[0],
//		//	KF->statePost.ptr<double>(0)[1],
//		//	KF->statePost.ptr<double>(0)[2]) << std::endl;
//	}
}