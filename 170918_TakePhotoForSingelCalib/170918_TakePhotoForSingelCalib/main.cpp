#include <iostream>
#include "opencv2\opencv.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;
//�˳������ڻ�ȡ�궨�����ͼƬ����'q'���Զ����գ��������洢��
int main(void)
{
	VideoCapture cap(0);
	int i = 0;
	char a[40];
	char c = 0;
	while (c !=27)
	{
		Mat frame, frame_resize;
		cap >> frame;
		resize(frame, frame_resize, Size(640, 480));
		imshow("show", frame_resize);
		sprintf_s(a,"%d.bmp", i);
		c = waitKey(1);
		if (c == 'q')
		{
			imwrite(a, frame_resize); //����ͼƬ
			cout << i <<"\n";
			i++;
		}
		
	}
	return 0;
}