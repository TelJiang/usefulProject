#include "glshow_cvAPI.h"

namespace mv
{
	void glOnDraw(void *param)
	{
		//const std::vector<cv::Point3f>& ballPoints = *(std::vector<cv::Point3f>*)param;
		const mv::GLSiteIFM& mySiteIFM = *(mv::GLSiteIFM*)param;
		static int list_draw_site = 0;
		static int list_draw_ball = 0;
		static int list_draw_preBall = 0;
		//第一次调用时的初始化
		if (list_draw_site == 0)
		{
			glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_MULTISAMPLE);
			glMatrixMode(GL_PROJECTION);//透视变换
			glLoadIdentity();
			gluPerspective(90.0f, 1.0f, 1.0f, 20.0f);//90：视野广角
			glMatrixMode(GL_MODELVIEW);//模型视图
			glEnable(GL_DEPTH_TEST);
		}

		list_draw_site = listDisplaySite();
		list_draw_ball = listDisplayBall();
		list_draw_preBall = listDisplayPreBall();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();
		gluLookAt((GLdouble)mySiteIFM.x, (GLdouble)mySiteIFM.y, (GLdouble)mySiteIFM.z, 0.f, 0.f, 2.0f, 0.0f, 0.0f, 1.0f);//调整视角

																														   // 定义光源(白色的光源)
		GLfloat sun_light_position[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		GLfloat sun_light_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat sun_light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		GLfloat sun_light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		glLightfv(GL_LIGHT0, GL_POSITION, sun_light_position);
		glLightfv(GL_LIGHT0, GL_AMBIENT, sun_light_ambient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, sun_light_diffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, sun_light_specular);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		//调用显示列表,显示场地
		glCallList(list_draw_site);
		//使用输入参数(此处为球的坐标)
		int _ballPointsSize;
		_ballPointsSize = mySiteIFM.BallPoints.size();
		for (int i = 0; i < _ballPointsSize; i++)
		{
			glPushMatrix();//变换前存下此时坐标系中心
			glTranslatef((GLfloat)mySiteIFM.BallPoints[i].x, mySiteIFM.BallPoints[i].y, mySiteIFM.BallPoints[i].z);//正好kalman滤波里边的单位为m
			glCallList(list_draw_ball);
			glPopMatrix();//画完模型后再还原回去
		}
		int _preBallPointsSize;
		_preBallPointsSize = mySiteIFM.preBallPoints.size();
		for (int i = 0; i < _preBallPointsSize; i++)
		{
			glPushMatrix();//变换前存下此时坐标系中心
			glTranslatef((GLfloat)mySiteIFM.preBallPoints[i].x, mySiteIFM.preBallPoints[i].y, mySiteIFM.preBallPoints[i].z);//正好kalman滤波里边的单位为m
			glCallList(list_draw_preBall);
			glPopMatrix();//画完模型后再还原回去
		}
		glFlush();
		//glutSwapBuffers();，//cv接口下双缓冲问题还没弄清楚,注释掉

	}

	//球的显示列表
	int listDisplayBall()
	{
		static int _list = 0;
		if (_list == 0)
		{
			_list = glGenLists(1);
			glNewList(_list, GL_COMPILE_AND_EXECUTE);//开始设置显示列表
													 //定义球的材质
			GLfloat ball_diffuse[] = { 0.0f, 0.0f, 0.5f, 1.0f };
			GLfloat ball_specular[] = { 0.0f, 0.0f, 1.0f, 1.0f };
			GLfloat ball_emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
			GLfloat ball_shininess = 5.0f;
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_golden);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, ball_diffuse);
			glMaterialfv(GL_FRONT, GL_SPECULAR, ball_specular);
			glMaterialfv(GL_FRONT, GL_EMISSION, ball_emission);
			glMaterialf(GL_FRONT, GL_SHININESS, ball_shininess);


			const float PI = 3.141592f;
			GLfloat x, y, z, alpha, beta; // Storage for coordinates and angles        
			GLfloat radius = 0.06f;
			int gradation = 30;
			for (alpha = 0.0; alpha < CV_PI; alpha += PI / gradation)
			{
				glBegin(GL_TRIANGLE_STRIP);
				for (beta = 0.0; beta < 2.01*CV_PI; beta += PI / gradation)
				{
					x = radius*cos(beta)*sin(alpha);
					y = radius*sin(beta)*sin(alpha);
					z = radius*cos(alpha);
					glVertex3f(x, y, z);
					x = radius*cos(beta)*sin(alpha + PI / gradation);
					y = radius*sin(beta)*sin(alpha + PI / gradation);
					z = radius*cos(alpha + PI / gradation);
					glVertex3f(x, y, z);
				}
				glEnd();
			}

			glEndList();
		}
		return _list;
	}

	//预测的球的显示列表
	int listDisplayPreBall()
	{
		static int _list = 0;
		if (_list == 0)
		{
			_list = glGenLists(1);
			glNewList(_list, GL_COMPILE_AND_EXECUTE);//开始设置显示列表
													 //定义球的材质
			GLfloat ball_diffuse[] = { 0.0f, 0.0f, 0.5f, 1.0f };
			GLfloat ball_specular[] = { 0.0f, 0.0f, 1.0f, 1.0f };
			GLfloat ball_emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
			GLfloat ball_shininess = 5.0f;
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_preBall);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, ball_diffuse);
			glMaterialfv(GL_FRONT, GL_SPECULAR, ball_specular);
			glMaterialfv(GL_FRONT, GL_EMISSION, ball_emission);
			glMaterialf(GL_FRONT, GL_SHININESS, ball_shininess);


			const float PI = 3.141592f;
			GLfloat x, y, z, alpha, beta; // Storage for coordinates and angles        
			GLfloat radius = 0.06f;
			int gradation = 30;
			for (alpha = 0.0; alpha < CV_PI; alpha += PI / gradation)
			{
				glBegin(GL_TRIANGLE_STRIP);
				for (beta = 0.0; beta < 2.01*CV_PI; beta += PI / gradation)
				{
					x = radius*cos(beta)*sin(alpha);
					y = radius*sin(beta)*sin(alpha);
					z = radius*cos(alpha);
					glVertex3f(x, y, z);
					x = radius*cos(beta)*sin(alpha + PI / gradation);
					y = radius*sin(beta)*sin(alpha + PI / gradation);
					z = radius*cos(alpha + PI / gradation);
					glVertex3f(x, y, z);
				}
				glEnd();
			}

			glEndList();
		}
		return _list;
	}

	//场地显示列表
	int listDisplaySite()
	{
		static int _list = 0;//初始化一次
		if (_list == 0)
		{
			// 如果显示列表不存在，则创建
			_list = glGenLists(1);
			glNewList(_list, GL_COMPILE_AND_EXECUTE);//开始设置显示列表,只分配，不执行
			glLineWidth(2.0f);//设置线宽

							  ///////////////////////////// //底面，绿色//////////////////////////////////////////						
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_green);
			glBegin(GL_POLYGON);
			glVertex3f(7.05f, 7.05f, 0.0f);
			glVertex3f(-7.05f, 7.05f, 0.0f);
			glVertex3f(-7.05f, -7.05f, 0.0f);
			glVertex3f(7.05f, -7.05f, 0.0f);
			glEnd();

			//绘制三个立柱(环)
			//////////////////////////////金杯////////////////////////////////////////////
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_golden);
			glBegin(GL_LINES);//圆柱用粗线来代替(渲染比例比较小，就先用先来代替)
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, 0.0f, 3.0f);
			glEnd();
			glBegin(GL_LINE_LOOP);
			for (int i = 0; i < 2 * site_n; i++)
				glVertex3f(0.4*cos(i * CV_PI / site_n), 0.0f, siteH_pillars_middle + 0.4*sin(i * CV_PI / site_n));
			glEnd();

			/////////////////////////////红区红柱/////////////////////////////////////////////
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_red);
			glBegin(GL_LINES); //圆柱用粗线来代替(渲染比例比较小，就先用先来代替)
			glVertex3f(siteDis_pillars_center, 0.0f, 0.0f);
			glVertex3f(siteDis_pillars_center, 0.0f, 2.0f);//柱高两米
			glEnd();
			glBegin(GL_LINE_LOOP);
			for (int i = 0; i < 2 * site_n; i++)
				glVertex3f(siteDis_pillars_center + 0.4*cos(i * CV_PI / site_n), 0.0, siteH_pillars_side + 0.4*sin(i * CV_PI / site_n));
			glEnd();
			///////////////////////////////蓝区蓝柱///////////////////////////////////////////
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_blue);
			glBegin(GL_LINES);//圆柱用粗线来代替(渲染比例比较小，就先用先来代替)
			glVertex3f(-siteDis_pillars_center, 0.0, 0.0);
			glVertex3f(-siteDis_pillars_center, 0.0, 2.0);//柱高两米
			glEnd();
			glBegin(GL_LINE_LOOP);
			for (int i = 0; i < 2 * site_n; i++)
				glVertex3f(-siteDis_pillars_center + 0.4*cos(i * CV_PI / site_n), 0.0, siteH_pillars_side + 0.4*sin(i * CV_PI / site_n));
			glEnd();

			/////////////////////绘制金杯///////////////////////////////////////
			/////////////////////siteDis_cup_center负半轴金杯//////////////////////////////////
			glLineWidth(5.0);//设置线宽大一点
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_golden);
			glBegin(GL_LINES);
			glVertex3f(0.0, -siteDis_cup_center, 0.0);
			glVertex3f(0.0, -siteDis_cup_center, siteH_cup);
			glEnd();
			glLineWidth(2.0);//再小一点
			glBegin(GL_TRIANGLE_STRIP);//盘
			for (int i = 0; i < 2 * site_n + 1; i++)
			{
				glVertex3f(0.6*cos(i * CV_PI / site_n), -siteDis_cup_center + 0.6*sin(i * CV_PI / site_n), siteH_cup);
				glVertex3f(0.2*cos(i * CV_PI / site_n), -siteDis_cup_center + 0.2*sin(i * CV_PI / site_n), siteH_cup);
			}
			glEnd();

			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_red);
			glBegin(GL_POLYGON);//金杯中的红点
			for (int i = 0; i < 2 * site_n; i++)
				glVertex3f(0.2*cos(i * CV_PI / site_n), -siteDis_cup_center + 0.2*sin(i * CV_PI / site_n), siteH_cup);
			glEnd();

			////////////////////siteDis_cup_center正半轴金杯////////////////////////////////////////////////
			glLineWidth(5.0);//设置线宽大一点
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_golden);
			glBegin(GL_LINES);
			glVertex3f(0.0, siteDis_cup_center, 0.0);
			glVertex3f(0.0, siteDis_cup_center, siteH_cup);
			glEnd();
			glLineWidth(2.0);//线宽再小一点
			glBegin(GL_TRIANGLE_STRIP);//盘
			for (int i = 0; i < 2 * site_n + 1; i++)
			{
				glVertex3f(0.6*cos(i * CV_PI / site_n), siteDis_cup_center + 0.6*sin(i * CV_PI / site_n), siteH_cup);
				glVertex3f(0.2*cos(i * CV_PI / site_n), siteDis_cup_center + 0.2*sin(i * CV_PI / site_n), siteH_cup);
			}
			glEnd();

			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_blue);//金杯中的蓝点
			glBegin(GL_POLYGON);
			for (int i = 0; i < 2 * site_n; i++)
				glVertex3f(0.2*cos(i * CV_PI / site_n), siteDis_cup_center + 0.2*sin(i * CV_PI / site_n), siteH_cup);
			glEnd();
			/////////////////////////////////结束杯的绘制/////////////////////////////////

			//////////////////////////////////划地面白线////////////////////////////////////////
			glLineWidth(2.0);//设置线宽
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_blue);
			glBegin(GL_LINES);//空间坐标系的x轴
			glVertex3f(0.0, 0.0, 0.03);
			glVertex3f(7.05, 0.0, 0.03);
			glEnd();
			glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_white);
			/*glColor3f(1.0f, 0.0f, 0.0f);*/
			glBegin(GL_LINES);//空间坐标系的y轴
			glVertex3f(0.0, 0.0, 0.03);//4.98f
			glVertex3f(0.0, 7.05, 0.03);
			glEnd();
			glBegin(GL_LINES);//1米长的短白线——标识用
			glVertex3f(0.5, 6.015, 0.03);
			glVertex3f(-0.5, 6.015, 0.03);
			glEnd();
			glBegin(GL_LINES);//从柱子拉出来的白线(平行于y轴的)
			glVertex3f(siteDis_pillars_center, 0.0, 0.03);//4.98f
			glVertex3f(siteDis_pillars_center, 7.05, 0.03);
			glEnd();
			glBegin(GL_LINES);//1米长的短白线——标识用
			glVertex3f(siteDis_pillars_center + 0.5, 3.995, 0.03);//4.98f
			glVertex3f(siteDis_pillars_center - 0.5, 3.995, 0.03);
			glEnd();
			glBegin(GL_LINES);//1米长的短白线——标识用
			glVertex3f(siteDis_pillars_center + 0.5, 6.015, 0.03);//4.98f
			glVertex3f(siteDis_pillars_center - 0.5, 6.015, 0.03);
			glEnd();
			glEndList();
		}
		return _list;
	}

}