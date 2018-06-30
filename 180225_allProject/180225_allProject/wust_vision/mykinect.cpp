#include "mykinect.h"

namespace mv
{
	// Safe release for interfaces
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
	//输入单位为mm输出m
	void get3D_ir(const double& depth,const Point2f& srcPoint,Point3d& _dst3DPoint)
	{
		// 归一化坐标	
		double x_dist, y_dist;
		x_dist = (srcPoint.x - ir_u0) / ir_fx;
		y_dist = (srcPoint.y - ir_v0) / ir_fy;

		// 开始畸变补偿

		int k = 10;  // 迭代次数，实测值为3次就差不多了，可以多试凑几次

		double r2;   // x*x + y*y
		double x, y;	// 迭代变量
		double k_radial; // 径向
		double k_tang_x, k_tang_y;   // 切向

		double delta_;  // 残差
		double x0, y0;  // 每轮迭代开始时x,y的值，用于


		x = x_dist;  // 迭代初值
		y = y_dist;
		// 开始迭代
		for (int i = 0; i < k; i++)  // 迭代3次后,[x,y]的相对变化量为0.0003949
		{
			x0 = x; y0 = y;

			r2 = x*x + y*y;
			k_radial = 1 + ir_k1 * r2 + ir_k2 * r2*r2 + ir_p1 * r2*r2*r2;
			k_tang_x = 2 * ir_p1*x*y + ir_p2*(r2 + 2 * x*x);
			k_tang_y = 2 * ir_p2*x*y + ir_p1*(r2 + 2 * y*y);

			x = (x_dist - k_tang_x) / k_radial;
			y = (y_dist - k_tang_y) / k_radial;

			// 如果不使用残差决定是否结束迭代，则可以不运行下面if中的代码
			// 以提高运算速度
			if (0)
			{
				double delta = sqrt((x - x0)*(x - x0) + (y - y0)*(y - y0));
				double normal = sqrt((x0)*(x0)+(y0)*(y0));
				delta_ = delta / normal;
				if (delta_ < 0.0001)
					break;
			}

		}

		//u = x * ir_fx + ir_u0;
		//v = y * ir_fy + ir_v0;

		_dst3DPoint.z = depth;
		_dst3DPoint.x = x *depth;
		_dst3DPoint.y = y *depth;
		return;
	}

	void posTo2D(Point3d& src_point,Point2f& dst_point)
	{
		dst_point.x = (float)src_point.x*ir_fx / src_point.z + ir_u0;
		dst_point.y = (float)src_point.y*ir_fy / src_point.z + ir_v0;
	}

	MyKinectSensor::MyKinectSensor(void)
	{

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
		{
			return;
		}

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get the depth reader  
			IDepthFrameSource* pDepthFrameSource = NULL;

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			}

			SafeRelease(pDepthFrameSource);

			// for color  
			// Initialize the Kinect and get the color reader  
			IColorFrameSource* pColorFrameSource = NULL;
			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			}

			SafeRelease(pColorFrameSource);
		}

		//valify the depth reader  
		if (!m_pDepthFrameReader)
		{
			cout << "FUCK! Can not find the m_pDepthFrameReader!" << endl;
			waitKey(0);
			exit(0);
		}
		//valify the color reader  
		if (!m_pColorFrameReader)
		{
			cout << "FUCK! Can not find the m_pColorFrameReader!" << endl;
			waitKey(0);
			exit(0);
		}
	}

	MyKinectSensor::~MyKinectSensor(void)
	{

	}

	void MyKinectSensor::myKinectInit()
	{

	}

	void MyKinectSensor::outPutCalibXML()
	{
		Mat_<float> color_NTOD_X = Mat_<float>::zeros(Size(kinectColorWidth, kinectColorHeight));
		Mat_<float> color_NTOD_Y = Mat_<float>::zeros(Size(kinectColorWidth, kinectColorHeight));
		for (int i = 0; i < kinectColorHeight; i++)
		{
			for (int j = 0; j < kinectColorWidth; j++)
			{
				double x_1 = (j - color_u0) / color_fx;
				double y_1 = (i - color_v0) / color_fy;
				double r2 = x_1*x_1 + y_1*y_1;
				double x_2 = x_1*(1 + color_k1*r2 + color_k2*r2*r2 + color_k5*r2*r2*r2) + 2 * color_k3*x_1*y_1 + color_k4*(r2 + 2 * x_1*x_1);
				double y_2 = y_1*(1 + color_k1*r2 + color_k2*r2*r2 + color_k5*r2*r2*r2) + color_k3*(r2 + 2 * y_1*y_1) + 2 * color_k4*x_1*y_1;
				float x = (float)(x_2*color_fx + color_u0);
				float y = (float)(y_2*color_fy + color_v0);
				//double x_1 = (KINECT_DEPTH_WIDTH - j - ir_u0) / ir_fx;
				//double y_1 = (i - ir_v0) / ir_fy;
				//double r2 = x_1*x_1 + y_1*y_1;
				//double x_2 = x_1*(1 + ir_k1*r2 + ir_k2*r2*r2 + ir_k5*r2*r2*r2) + 2 * ir_k3*x_1*y_1 + ir_k4*(r2 + 2 * x_1*x_1);
				//double y_2 = y_1*(1 + ir_k1*r2 + ir_k2*r2*r2 + ir_k5*r2*r2*r2) + ir_k3*(r2 + 2 * y_1*y_1) + 2 * ir_k4*x_1*y_1;
				if ((int)x >= 0 && (int)x < kinectColorWidth && (int)y >= 0 && (int)y < kinectColorHeight)
				{
					color_NTOD_X(i, j) = x;
					color_NTOD_Y(i, j) = y;
				}
			}
		}
		FileStorage fs("color_NTOD.xml", cv::FileStorage::WRITE);
		fs << "color_NTOD_X" << color_NTOD_X;
		fs << "color_NTOD_Y" << color_NTOD_Y;
		cout << "finished(color_NTOD)" << endl;
	}


	Point2f MyKinectSensor::kinect_IR2COLOR(Point2f _point, double z_1)
	{
		double x_1 = (_point.x - ir_u0) *z_1 / ir_fx;
		double y_1 = (_point.y - ir_v0) *z_1 / ir_fy;
		//cout << x_1 << endl;
		/*double r2 = x_1*x_1 + y_1*y_1;
		double x_2 = x_1*(1 + ir_k1*r2 + ir_k2*r2*r2 + ir_k5*r2*r2*r2) + 2 * ir_k3*x_1*y_1 + ir_k4*(r2 + 2 * x_1*x_1);
		double y_2 = y_1*(1 + ir_k1*r2 + ir_k2*r2*r2 + ir_k5*r2*r2*r2) + ir_k3*(r2 + 2 * y_1*y_1) + 2 * ir_k4*x_1*y_1;
		*/
		//暂未进行畸变矫正
		//开始坐标转换
		double x_2 = x_1*9.9997034868676316e-01 - y_1*9.3487189338337660e-04 - z_1*7.6438054538507461e-03 + 5.1247529705834620e-02;//讲道理，此处的偏移量单位应该为mm但是效果不如单位为m的好所以此处就用的m
		double y_2 = x_1*(9.3570247207732792e-04) + y_1*9.9999955670892859e-01 - z_1*1.0508487062746666e-04 - 2.3074786414896873e-03;
		double z_2 = x_1*7.6437038245280682e-03 + y_1*1.1223408238228880e-04 + z_1*9.9997078017077767e-01 + 1.1939132799787798e-02;

		int x = (x_2*color_fx / z_2 + color_u0);
		int y = (y_2*color_fy / z_2 + color_v0);
		return Point2f(x, y);
	}

	void MyKinectSensor::kinectDo(KinectDo _mode)
	{
		IDepthFrame* pDepthFrame = NULL;
		IColorFrame* pColorFrame = NULL;

		switch (_mode)
		{
		case MV_KINECT_DOIR:
			do 
			{
				hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
				if (SUCCEEDED(hr))
				{
					USHORT nDepthMinReliableDistance = 0;//
					USHORT nDepthMaxReliableDistance = 0;//
					if (SUCCEEDED(hr))
					{
						hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
						/*cout << nDepthMinReliableDistance << endl;*/
					}
	
					if (SUCCEEDED(hr))
					{
						hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
						/*cout << nDepthMaxReliableDistance << endl;*/
					}
					if (SUCCEEDED(hr))
					{
						hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
						pDepthFrame->CopyFrameDataToArray(kinectIrWidth*kinectIrHeight, (UINT16 *)depthImg.data);
						//depthImg_show = ConvertMat(pBuffer_depth, kinectIrWidth, kinectIrHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);
						depthImg.convertTo(depthImg_show, CV_8UC1);
					}
				}
			} while (!SUCCEEDED(hr));
			break;
		case MV_KINECT_DOCOLOR:
			do 
			{
				hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
				if (SUCCEEDED(hr))
				{
					ColorImageFormat imageFormat = ColorImageFormat_None;
					if (SUCCEEDED(hr))
					{
						hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
					}
					RGBQUAD*  m_pColorRGBX = NULL;
					m_pColorRGBX = new RGBQUAD[kinectColorWidth * kinectColorHeight];
					if (SUCCEEDED(hr))
					{
						if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
						{
							hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
						}
						else if (m_pColorRGBX)
						{
							pBuffer_color = m_pColorRGBX;
							nBufferSize_coloar = kinectColorWidth * kinectColorHeight * sizeof(RGBQUAD);
							hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
						}
	
						else
						{
							hr = E_FAIL;
						}
						colorImg = ConvertMat(pBuffer_color, kinectColorWidth, kinectColorHeight);
					}
					delete[] m_pColorRGBX;
				}
			} while (!SUCCEEDED(hr));
			break;
		case MV_KINECT_DOCOLORANDIR:
			do
			{
				hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
				if (SUCCEEDED(hr))
				{
					USHORT nDepthMinReliableDistance = 0;//
					USHORT nDepthMaxReliableDistance = 0;//
					if (SUCCEEDED(hr))
					{
						hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
					}

					if (SUCCEEDED(hr))
					{
						hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
					}
					if (SUCCEEDED(hr))
					{
						hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
						pDepthFrame->CopyFrameDataToArray(kinectIrWidth*kinectIrHeight, (UINT16 *)depthImg.data);
						//depthImg_show = ConvertMat(pBuffer_depth, kinectIrWidth, kinectIrHeight, nDepthMinReliableDistance, nDepthMaxReliableDistance);
						depthImg.convertTo(depthImg_show, CV_8UC1);
					}
				}
			} while (SUCCEEDED(hr));
			do
			{
				hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
				if (SUCCEEDED(hr))
				{
					ColorImageFormat imageFormat = ColorImageFormat_None;
					if (SUCCEEDED(hr))
					{
						hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
					}
					RGBQUAD*  m_pColorRGBX = NULL;
					m_pColorRGBX = new RGBQUAD[kinectColorWidth * kinectColorHeight];
					if (SUCCEEDED(hr))
					{
						if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
						{
							hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
						}
						else if (m_pColorRGBX)
						{
							pBuffer_color = m_pColorRGBX;
							nBufferSize_coloar = kinectColorWidth * kinectColorHeight * sizeof(RGBQUAD);
							hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
						}
	
						else
						{
							hr = E_FAIL;
						}
						colorImg = ConvertMat(pBuffer_color, kinectColorWidth, kinectColorHeight);
					}
					delete[] m_pColorRGBX;
				}
			} while (!SUCCEEDED(hr));
			break;
		default:
			break;
		}
		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
	}

	Mat MyKinectSensor::getDepth16U()
	{
		return depthImg;
	}

	Mat MyKinectSensor::getBGR()
	{
		return colorImg;
	}

	Mat MyKinectSensor::getDepth8U()
	{
		return depthImg_show;
	}

	//彩图转换BGR
	Mat MyKinectSensor::ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
	{
		Mat img(nHeight, nWidth, CV_8UC3);
		uchar* p_mat = img.data;

		const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			*p_mat = pBuffer->rgbBlue;
			p_mat++;
			*p_mat = pBuffer->rgbGreen;
			p_mat++;
			*p_mat = pBuffer->rgbRed;
			p_mat++;

			++pBuffer;
		}
		return img;
	}

	//深度图，这张深度位8的图并不适合用来得到距离信息，只是显示比较指直观
	//易于分析观察
	//Mat MyKinectSensor::ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
	//{
	//	Mat img(nHeight, nWidth, CV_8UC1);
	//	uchar* p_mat = img.data;

	//	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	//	while (pBuffer < pBufferEnd)
	//	{
	//		USHORT depth = *pBuffer;

	//		BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth * 255 / nMaxDepth) : 0);

	//		*p_mat = intensity;
	//		p_mat++;
	//		//*p_mat = intensity;//之前把深度图弄成3通道的，这里保留历史是为了清楚，数据是连续的
	//		//p_mat++;
	//		//*p_mat = intensity;
	//		//p_mat++;

	//		++pBuffer;
	//	}
	//	return img;
	//}
}