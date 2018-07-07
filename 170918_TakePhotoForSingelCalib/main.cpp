#include <iostream>
#include "opencv2\opencv.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;
//此程序用于获取标定所需的图片，按'q'，自动拍照，并命名存储。
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
			imwrite(a, frame_resize); //保存图片
			cout << i <<"\n";
			i++;
		}
		
	}
	return 0;
}