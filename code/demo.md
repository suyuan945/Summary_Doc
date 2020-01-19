```
// demo.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include "LibRSCam.h"
#include <opencv2/opencv.hpp>
#include <windows.h>
#include "FileHelper.h"

#define DEMO_PI 3.14159265358979323846

#define RGB_W 640
#define RGB_H 480

#define DEP_W 640
#define DEP_H 480

#define DECIMATE_DIV 1

LIBRSCam::RSCam MyCam;

typedef struct point_st {
	float x;
	float y;
	float z;

	uchar r;
	uchar g;
	uchar b;
};

void short_depth2PointCloud(cv::Mat imDepth, std::string filePath, int cx_offset, int cy_offset, float times, int algn2rgb)
{
	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);
	float fx, fy, cx, cy;

	std::cout << "cx=" << list_of_ints[0] << std::endl;
	std::cout << "cy=" << list_of_ints[1] << std::endl;
	std::cout << "fx=" << list_of_ints[2] << std::endl;
	std::cout << "fy=" << list_of_ints[3] << std::endl;

	std::cout << "cx=" << list_of_ints[4] << std::endl;
	std::cout << "cy=" << list_of_ints[5] << std::endl;
	std::cout << "fx=" << list_of_ints[6] << std::endl;
	std::cout << "fy=" << list_of_ints[7] << std::endl;

	if (algn2rgb == 0) {
		fx = list_of_ints[6] * times;
		fy = list_of_ints[7] * times;
		cx = list_of_ints[4] * times;
		cy = list_of_ints[5] * times;
	}
	else {
		fx = list_of_ints[2] * times;
		fy = list_of_ints[3] * times;
		cx = list_of_ints[0] * times;
		cy = list_of_ints[1] * times;
	}

	float factor = 10000.0f;

	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {
			unsigned short Zw = imDepth.at<unsigned short>(i, j);
			if (Zw != 0) {
				//std::cout << Zw << std::endl;
				point_st myp;

				myp.z = (float)(Zw / factor);
				myp.x = (j - cx + cx_offset) * myp.z / fx;
				myp.y = (i - cy + cy_offset) * myp.z / fy;

				points.push_back(myp);
			}
		}
	}

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
		//std::cout << points[i].x << std::endl;
		//std::cout << points[i].y << std::endl;
		//std::cout << points[i].z << std::endl;
		ply << points[i].x << "     " << points[i].y << "      " << points[i].z << "\n";
	}
	ply.close();
}

void float_depth2PointCloud(cv::Mat imDepth, std::string filePath, int cx_offset, int cy_offset, float times, int algn2rgb)
{
	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);
	float fx, fy, cx, cy;

	std::cout << "cx=" << list_of_ints[0] << std::endl;
	std::cout << "cy=" << list_of_ints[1] << std::endl;
	std::cout << "fx=" << list_of_ints[2] << std::endl;
	std::cout << "fy=" << list_of_ints[3] << std::endl;

	std::cout << "cx=" << list_of_ints[4] << std::endl;
	std::cout << "cy=" << list_of_ints[5] << std::endl;
	std::cout << "fx=" << list_of_ints[6] << std::endl;
	std::cout << "fy=" << list_of_ints[7] << std::endl;

	if (algn2rgb == 0) {
		fx = list_of_ints[6] * times;
		fy = list_of_ints[7] * times;
		cx = list_of_ints[4] * times;
		cy = list_of_ints[5] * times;
	}
	else {
		fx = list_of_ints[2] * times;
		fy = list_of_ints[3] * times;
		cx = list_of_ints[0] * times;
		cy = list_of_ints[1] * times;
	}

	float factor = 10000.0f;

	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {
			float Zw = imDepth.at<float>(i, j);
			if (Zw != 0) {
				//std::cout << Zw << std::endl;
				point_st myp;

				myp.z = (float)(Zw / factor);
				myp.x = (j - cx + cx_offset) * myp.z / fx;
				myp.y = (i - cy + cy_offset) * myp.z / fy;

				points.push_back(myp);
			}
		}
	}

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
		//std::cout << points[i].x << std::endl;
		//std::cout << points[i].y << std::endl;
		//std::cout << points[i].z << std::endl;
		ply << points[i].x << "     " << points[i].y << "      " << points[i].z << "\n";
	}
	ply.close();
}

//#define TEST_AVG //(x1-x)*(x1-x)+(x2-x)*(x2-x)+(x3-x)*(x3-x)+ ...(xn-x)*(xn-x),求和最小值时x的值是多少，验证下来就是x取算术平均值时求和最小
//#define TEST_LUCAS_KANADE_OPENCV //LK光流法demo
//#define TEST_FBACK_OPENCV //密集型光流法demo
//#define TEST_PNG_2_PLY //depth png to ply
//#define TEST_1 //保存深度图和RGB图片,包含单张保存(按键s)，清楚数据(按键r)
//#define TEST_2 //保存深度图和RGB图片,包含连续保存(按键s)，清楚数据(按键r)
//#define TEST_3 //深度图超分辨率
//#define TEST_4 //PCL_ICP
//#define TEST_4_1 //PCL_ICP 用反投影方式
//#define TEST_5 //颜色过渡1 蓝 红
//#define TEST_6 //颜色过渡2 蓝 红
//#define TEST_7 //三张图接缝过渡
//#define TEST_8 //GAMMA/白平衡
//#define TEST_9 //face morph
//#define TEST_10 //瑞芯微深度图和RGB图align验证
//#define TEST_11 //批量瑞芯微深度图和RGB图align验证
//#define TEST_12 //单张深度去背景
//#define TEST_13 //单张深度二值化
//#define TEST_14 //opencv 正脸检测/侧脸检测
//#define TEST_15 //dlib正脸检测1
//#define TEST_16 //dlib正脸检测2
//#define TEST_17 //dlib CNN人脸检测
//#define TEST_18 //cvMat 测试
//#define TEST_19 //张正友标定内参(rs415)
//#define TEST_19_1 //张正友标定内参（file）
//#define TEST_20 //标定外参
//#define TEST_20_1 //标定外参（file）
//#define TEST_20_2 //标定内外参（rs415 + AIO前摄12M）
//#define TEST_21 // TOF实验
//#define TEST_22 //TOF实验
//#define TEST_23
//#define TEST_24 //耳朵检测
//#define TEST_25 //yuvN21 to rgb, yuyv to gray
//#define TEST_26
//#define TEST_27 //三角剖分与仿射变换
//#define TEST_28
//#define TEST_29
//#define TEST_30 //图像仿射变换
//#define TEST_31
//#define TEST_32
#define TEST_33 //PCA

#if defined TEST_AVG
int main(int argc, char** argv)
{
	float a[10] = { 1,1,1,1,1,1,1,1,90,100 };

	float b = 0;
	float sum_b = 0;
	int loops = (a[9] - a[0]) * 10 + 1; //0.1

	float temp = 100000;
	int k = 0;

	for (int i = 0; i < loops; i++)
	{
		b = a[0] + 0.1*i;
		sum_b = 0;

		for (int j = 0; j < 10; j++)
		{
			sum_b += (a[j] - b)*(a[j] - b);
		}

		if (sum_b < temp)
		{
			temp = sum_b;
			k = i;
		}
	}

	std::cout << "b:" << a[0] + 0.1*k << std::endl;
	std::cout << "temp:" << temp << std::endl;

	float avg = 0;
	float sum_avg = 0;

	for (int i = 0; i < 10; i++)
	{
		avg += a[i];
	}

	avg = avg / 10;

	for (int i = 0; i < 10; i++)
	{
		sum_avg += (a[i] - avg)*(a[i] - avg);
	}

	std::cout << "avg:" << avg << std::endl;
	std::cout << "sum_avg:" << sum_avg << std::endl;

	return 0;
}
#elif defined(TEST_LUCAS_KANADE_OPENCV)

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
		"Using OpenCV version " << CV_VERSION << endl;
	cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tr - auto-initialize tracking\n"
		"\tc - delete all the points\n"
		"\tn - switch the \"night\" mode on/off\n"
		"To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		point = Point2f((float)x, (float)y);
		addRemovePt = true;
	}
}

int main(int argc, char** argv)
{
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned short* depth_image0 = new unsigned short[DEP_H * DEP_W];

	int align = 0;

	MyCam.StartAllCam();

	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	const int MAX_COUNT = 500;
	bool needToInit = false;
	bool nightMode = false;

	help();

	namedWindow("LK Demo", 1);
	setMouseCallback("LK Demo", onMouse, 0);

	Mat gray, prevGray, image;
	vector<Point2f> points[2];

	for (;;)
	{
		if (align) {
			//MyCam.grabAndFilterAlignedDeptFrame(0, rgb_image0, depth_image0);
		}
		else
		{
			MyCam.grabAndFilterDeptFrame(0, rgb_image0, depth_image0);
		}

		cv::Mat frame(DEP_H, DEP_W, CV_8UC3, rgb_image0);
		cv::cvtColor(frame, frame, CV_RGB2BGR);
		if (frame.empty())
			break;

		frame.copyTo(image);

		cvtColor(image, gray, COLOR_BGR2GRAY);

		if (nightMode)
			image = Scalar::all(0);

		if (needToInit)
		{
			// automatic initialization
			goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
			addRemovePt = false;
		}
		else if (!points[0].empty())
		{
			vector<uchar> status;
			vector<float> err;
			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);
			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++)
			{
				if (addRemovePt)
				{
					if (norm(point - points[1][i]) <= 5)
					{
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
			}
			points[1].resize(k);
		}

		if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
		{
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}

		needToInit = false;
		imshow("LK Demo", image);

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'r':
			needToInit = true;
			break;
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		case 'n':
			nightMode = !nightMode;
			break;
		}

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
}
#elif defined TEST_FBACK_OPENCV
using namespace cv;
using namespace std;

static void help()
{
	cout <<
		"\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
		"Mainly the function: calcOpticalFlowFarneback()\n"
		"Call:\n"
		"./fback\n"
		"This reads from video camera 0\n" << endl;
}
static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
	double, const Scalar& color)
{
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
			const Point2f& fxy = flow.at<Point2f>(y, x);
			line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
				color);
			circle(cflowmap, Point(x, y), 2, color, -1);
		}
}

int main(int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, "{help h||}");
	if (parser.has("help"))
	{
		help();
		return 0;
	}

	help();

	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned short* depth_image0 = new unsigned short[DEP_H * DEP_W];

	int align = 0;

	MyCam.StartAllCam();

	Mat flow, cflow, frame;
	UMat gray, prevgray, uflow;
	namedWindow("flow", 1);

	for (;;)
	{
		if (align) {
			//MyCam.grabAndFilterAlignedDeptFrame(0, rgb_image0, depth_image0);
		}
		else
		{
			MyCam.grabAndFilterDeptFrame(0, rgb_image0, depth_image0);
		}

		cv::Mat frame(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		cvtColor(frame, gray, COLOR_RGB2GRAY);

		if (!prevgray.empty())
		{
			calcOpticalFlowFarneback(prevgray, gray, uflow, 0.5, 3, 15, 3, 5, 1.2, 0);
			cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
			uflow.copyTo(flow);
			drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
			imshow("flow", cflow);
		}
		if (waitKey(30) >= 0)
			break;
		std::swap(prevgray, gray);
	}
	return 0;
}
#elif defined TEST_PNG_2_PLY
int main(int argc, char** argv)
{
	int align = 0;

	MyCam.StartAllCam();

	//读取深度图
	cv::Mat img = cv::imread("E:\\Project\\demo\\dataset\\TEST_PNG_2_PLY\\z.png", -1);

	//转成点云文件
	short_depth2PointCloud(img, "E:\\Project\\demo\\dataset\\TEST_PNG_2_PLY\\z.ply", 0, 0, 1, align);

	std::cout << "finished!!" << std::endl;

	return 0;
}
#elif defined TEST_1
int main()
{
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned short* depth_image0 = new unsigned short[(DEP_H * DEP_W) / (DECIMATE_DIV*DECIMATE_DIV)];

	int i = 0;
	int align = 0;

	MyCam.StartAllCam();

	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);
	std::cout << "cx=" << list_of_ints[0] << std::endl;
	std::cout << "cy=" << list_of_ints[1] << std::endl;
	std::cout << "fx=" << list_of_ints[2] << std::endl;
	std::cout << "fy=" << list_of_ints[3] << std::endl;

	std::cout << "cx=" << list_of_ints[4] << std::endl;
	std::cout << "cy=" << list_of_ints[5] << std::endl;
	std::cout << "fx=" << list_of_ints[6] << std::endl;
	std::cout << "fy=" << list_of_ints[7] << std::endl;

	for (;;)
	{
		if (align) {
			//MyCam.grabAndFilterAlignedDeptFrame(0, rgb_image0, depth_image0);
		}
		else
		{
			MyCam.grabAndFilterDeptFrame(0, rgb_image0, depth_image0);
		}

		//show rgb window
		cv::Mat rgb_frame0(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		cv::cvtColor(rgb_frame0, rgb_frame0, CV_RGB2BGR);
		if (!rgb_frame0.empty()) {
			cv::imshow("color", rgb_frame0);
		}

		//show depth window
		cv::Mat depth_frame0(DEP_H / DECIMATE_DIV, DEP_W / DECIMATE_DIV, CV_16UC1, depth_image0);
		if (!depth_frame0.empty()) {
			cv::imshow("depth", depth_frame0);
		}

		//key handle
		int keycode = cv::waitKey(1);
		if (keycode == 27)//Esc
			break;
		else if (keycode == 's')
		{
			std::stringstream filepath_i;
			filepath_i << "E:\\Project\\demo\\dataset\\TEST_1\\" << i;
			createDirByPath(filepath_i.str().c_str());

			//rgb
			std::stringstream filepath_rgb;
			filepath_rgb << "E:\\Project\\demo\\dataset\\TEST_1\\r_" << i << ".png";
			cv::imwrite(filepath_rgb.str(), rgb_frame0);
			CpFile(filepath_rgb.str(), filepath_i.str() + "\\r.png");

			//depth
			std::stringstream filepath_depth;
			filepath_depth << "E:\\Project\\demo\\dataset\\TEST_1\\z_" << i << ".png";
			cv::imwrite(filepath_depth.str(), depth_frame0);
			CpFile(filepath_depth.str(), filepath_i.str() + "\\z.png");

			//ply
			std::stringstream filepath_ply;
			filepath_ply << "E:\\Project\\demo\\dataset\\TEST_1\\c_" << i << ".ply";
			short_depth2PointCloud(depth_frame0, filepath_ply.str(), 0, 0, 1.0 / DECIMATE_DIV, align);
			CpFile(filepath_ply.str(), filepath_i.str() + "\\c.ply");

			//intrinsics
			std::stringstream filepath_intrinsics;
			filepath_intrinsics << "E:\\Project\\demo\\dataset\\TEST_1\\i_" << i << ".yml";
			std::ofstream intrinsics(filepath_intrinsics.str());
			if (align) {
				intrinsics << "fx:" << list_of_ints[2] << "\n";
				intrinsics << "fy:" << list_of_ints[3] << "\n";
				intrinsics << "cx:" << list_of_ints[0] << "\n";
				intrinsics << "cy:" << list_of_ints[1] << "\n";
			}
			else
			{
				intrinsics << "fx:" << list_of_ints[6] << "\n";
				intrinsics << "fy:" << list_of_ints[7] << "\n";
				intrinsics << "cx:" << list_of_ints[4] << "\n";
				intrinsics << "cy:" << list_of_ints[5] << "\n";
			}
			intrinsics.close();
			CpFile(filepath_intrinsics.str(), filepath_i.str() + "\\i.yml");

			i++;
		}
		else if (keycode == 'r')
		{
			i = 0;
			std::stringstream filepath_test1;
			filepath_test1 << "E:\\Project\\demo\\dataset\\TEST_1";
			deleteDirByPath(filepath_test1.str());
			createDirByPath(filepath_test1.str().c_str());
		}
	}

	std::cout << "finished!!" << std::endl;

	return 0;
}

#elif defined(TEST_2)

#define CONTINUE_CNT 30000

int main()
{
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned short* depth_image0 = new unsigned short[(DEP_H * DEP_W) / (DECIMATE_DIV*DECIMATE_DIV)];

	int align = 1;
	int continue_cnt = CONTINUE_CNT;

	MyCam.StartAllCam();

	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);
	std::cout << "rgb cx=" << list_of_ints[0] << std::endl;
	std::cout << "rgb cy=" << list_of_ints[1] << std::endl;
	std::cout << "rgb fx=" << list_of_ints[2] << std::endl;
	std::cout << "rgb fy=" << list_of_ints[3] << std::endl;

	std::cout << "depth cx=" << list_of_ints[4] << std::endl;
	std::cout << "depth cy=" << list_of_ints[5] << std::endl;
	std::cout << "depth fx=" << list_of_ints[6] << std::endl;
	std::cout << "depth fy=" << list_of_ints[7] << std::endl;

	for (;;)
	{
		if (align) {
			MyCam.grabAndDeptAlignFrame(0, rgb_image0, depth_image0);
		}
		else
		{
			MyCam.grabAndFilterDeptFrame(0, rgb_image0, depth_image0);
		}

		//show rgb window
		cv::Mat rgb_frame0(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		cv::cvtColor(rgb_frame0, rgb_frame0, CV_RGB2BGR);
		if (!rgb_frame0.empty()) {
			cv::imshow("color", rgb_frame0);
		}

		//show depth window
		cv::Mat depth_frame0(DEP_H / DECIMATE_DIV, DEP_W / DECIMATE_DIV, CV_16UC1, depth_image0);
		if (!depth_frame0.empty()) {
			cv::imshow("depth", depth_frame0);
		}

		if (continue_cnt < CONTINUE_CNT) {
			std::stringstream filepath_i;
			filepath_i << "E:\\Project\\demo\\dataset\\TEST_2\\" << continue_cnt;
			createDirByPath(filepath_i.str().c_str());

			//rgb
			std::stringstream filepath_rgb;
			filepath_rgb << "E:\\Project\\demo\\dataset\\TEST_2\\r_" << continue_cnt << ".png";
			cv::imwrite(filepath_rgb.str(), rgb_frame0);
			CpFile(filepath_rgb.str(), filepath_i.str() + "\\r.png");

			//depth
			std::stringstream filepath_depth;
			filepath_depth << "E:\\Project\\demo\\dataset\\TEST_2\\z_" << continue_cnt << ".png";
			cv::imwrite(filepath_depth.str(), depth_frame0);
			CpFile(filepath_depth.str(), filepath_i.str() + "\\z.png");

			//ply
			std::stringstream filepath_ply;
			filepath_ply << "E:\\Project\\demo\\dataset\\TEST_2\\c_" << continue_cnt << ".ply";
			short_depth2PointCloud(depth_frame0, filepath_ply.str(), 0, 0, 1.0 / DECIMATE_DIV, align);
			CpFile(filepath_ply.str(), filepath_i.str() + "\\c.ply");

			//intrinsics
			std::stringstream filepath_intrinsics;
			filepath_intrinsics << "E:\\Project\\demo\\dataset\\TEST_2\\i_" << continue_cnt << ".yml";
			std::ofstream intrinsics(filepath_intrinsics.str());
			if (align) {
				intrinsics << "fx:" << list_of_ints[2] << "\n";
				intrinsics << "fy:" << list_of_ints[3] << "\n";
				intrinsics << "cx:" << list_of_ints[0] << "\n";
				intrinsics << "cy:" << list_of_ints[1] << "\n";
			}
			else
			{
				intrinsics << "fx:" << list_of_ints[6] << "\n";
				intrinsics << "fy:" << list_of_ints[7] << "\n";
				intrinsics << "cx:" << list_of_ints[4] << "\n";
				intrinsics << "cy:" << list_of_ints[5] << "\n";
			}
			intrinsics.close();
			CpFile(filepath_intrinsics.str(), filepath_i.str() + "\\continue_cnt.yml");

			continue_cnt++;
		}

		//key handle
		int keycode = cv::waitKey(1);
		if (keycode == 27)//Esc
			break;
		else if (keycode == 's')
		{
			continue_cnt = 0;
		}
		else if (keycode == 'r')
		{
			continue_cnt = CONTINUE_CNT;
			std::stringstream filepath_test2;
			filepath_test2 << "E:\\Project\\demo\\dataset\\TEST_2";
			deleteDirByPath(filepath_test2.str());
			createDirByPath(filepath_test2.str().c_str());
		}
	}

	std::cout << "finished!!" << std::endl;

	return 0;
}

#elif defined TEST_3

using namespace cv;
using namespace std;

unsigned short getMax(unsigned short a[], int n)
{
	unsigned short buf = a[0];

	for (int i = 1; i < n; i++)
	{
		if (a[i] > buf)
			buf = a[i];
	}

	return buf;
}

unsigned short getMin(unsigned short a[], int n)
{
	unsigned short buf = a[0];

	for (int i = 1; i < n; i++)
	{
		if (a[i] < buf)
			buf = a[i];
	}

	return buf;
}

float getAvg(unsigned short a[], int n)
{
	float sum = 0;

	for (int i = 0; i < n; i++)
	{
		sum = sum + a[i];
	}

	return sum / n;
}

#define WINDOW_SIZE 9
typedef unsigned short element;
void _medianfilter(const element* signal, element* result, int N)
{
	//Move window through all elements of the signal	
	for (int i = (WINDOW_SIZE / 2); i < N - (WINDOW_SIZE / 2); ++i)
	{
		//Pick up window elements
		element window[WINDOW_SIZE];
		for (int j = 0; j < WINDOW_SIZE; ++j)
			window[j] = signal[i - (WINDOW_SIZE / 2) + j];
		//Order elements (only half of them)
		for (int j = 0; j < (WINDOW_SIZE / 2) + 1; ++j)
		{
			//Find position of minimum element
			int min = j;
			for (int k = j + 1; k < WINDOW_SIZE; ++k)
				if (window[k] < window[min])
					min = k;
			//Put found minimum element in its place
			const element temp = window[j];
			window[j] = window[min];
			window[min] = temp;
		}
		//Get result - the middle element
		result[i - (WINDOW_SIZE / 2)] = window[(WINDOW_SIZE / 2)];
	}
}

void medianfilter(element* signal, element* result, int N)
{
	//Check arguments
	if (!signal || N < 1)
		return;
	//Treat special case N = 1
	if (N == 1)
	{
		if (result)
			result[0] = signal[0];
		return;
	}
	//Allocate memory for signal extension
	element* extension = new element[N + (WINDOW_SIZE - 1)];
	//Check memory allocation
	if (!extension)
		return;
	//Create signal extension
	memcpy(extension + (WINDOW_SIZE / 2), signal, N * sizeof(element));
	for (int i = 0; i < (WINDOW_SIZE / 2); ++i)
	{
		extension[i] = signal[(WINDOW_SIZE / 2) - 1 - i];
		extension[N + (WINDOW_SIZE / 2) + i] = signal[N - 1 - i];
	}
	//Call median filter implementation
	_medianfilter(extension, result ? result : signal, N + (WINDOW_SIZE - 1));
	//Free memory
	delete[] extension;
}

typedef struct YK_ST {
	unsigned short* data = new unsigned short[(DEP_W * DEP_H) / (DECIMATE_DIV*DECIMATE_DIV)];
};

typedef struct YK_ST_F {
	float* data = new float[(DEP_W * DEP_H) / (DECIMATE_DIV*DECIMATE_DIV)];
};

#define Yk_CNT 30
int main(int argc, char** argv)
{
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned short* depth_image0 = new unsigned short[(DEP_W * DEP_H) / (DECIMATE_DIV*DECIMATE_DIV)];
	YK_ST Yk[Yk_CNT];
	YK_ST_F Dk;

	bool startRec = false;
	bool recFinished = false;
	int num = 0;

	int align = 0;

	MyCam.StartAllCam();

	for (;;)
	{
		if (align) {
			//MyCam.grabAndFilterAlignedDeptFrame(0, rgb_image0, depth_image0);
		}
		else
		{
			MyCam.grabAndFilterDeptFrame(0, rgb_image0, depth_image0);
		}

		//show rgb window
		cv::Mat frame_rgb(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		cv::cvtColor(frame_rgb, frame_rgb, CV_RGB2BGR);
		cv::imshow("demo", frame_rgb);

		if (startRec)
		{
			if (num >= Yk_CNT) {
				recFinished = true;
				startRec = false;
			}
			else {
				for (int i = 0; i < (DEP_W * DEP_H) / (DECIMATE_DIV*DECIMATE_DIV); i++)
				{
					Yk[num].data[i] = depth_image0[i];
				}
				num++;
			}
		}

		//key handle
		int keycode = waitKey(1);
		if (keycode == 27)//Esc
			break;
		else if (keycode == 's')
		{
			startRec = true;
		}

		if (recFinished)
		{
			std::cout << "start X!!" << std::endl;

			for (int i = 0; i < (DEP_W * DEP_H) / (DECIMATE_DIV*DECIMATE_DIV); i++)
			{
				unsigned short m_s[Yk_CNT] = { 0 };
				unsigned short m_d[Yk_CNT] = { 0 };

				for (int j = 0; j < Yk_CNT; j++)
				{
					m_s[j] = Yk[j].data[i];
				}
				medianfilter(m_s, m_d, Yk_CNT);

				Dk.data[i] = getAvg(m_d, Yk_CNT);
			}
			cv::Mat Dk_framef(DEP_H / DECIMATE_DIV, DEP_W / DECIMATE_DIV, CV_32FC1, Dk.data);
			cv::Mat Dk_frame;
			Dk_framef.convertTo(Dk_frame, CV_16U);
			imwrite("E:\\Project\\demo\\dataset\\TEST_3\\Dk_frame.png", Dk_frame);
			imwrite("E:\\Project\\demo\\dataset\\TEST_3\\frame_rgb.png", frame_rgb);
			float_depth2PointCloud(Dk_framef, "E:\\Project\\demo\\dataset\\TEST_3\\Dk_frame.ply", 0, 0, 1.0 / DECIMATE_DIV, align);

			num = 0;
			recFinished = false;

			std::cout << "finished X!!" << std::endl;

		}
	}

	std::cout << "finished!!" << std::endl;

	return 0;
}

#elif defined TEST_4

#include <pcl/io/pcd_io.h>         //I/O操作头文件
#include <pcl/point_types.h>        //点类型定义头文件
#include <pcl/registration/icp.h>   //ICP配准类相关头文件

using namespace cv;
using namespace std;

void depth2pcd(cv::Mat imDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cx_offset, int cy_offset, float times, int algn2rgb)
{
	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);
	float fx, fy, cx, cy;

	if (algn2rgb == 0) {
		fx = list_of_ints[6] * times;
		fy = list_of_ints[7] * times;
		cx = list_of_ints[4] * times;
		cy = list_of_ints[5] * times;
	}
	else {
		fx = list_of_ints[2] * times;
		fy = list_of_ints[3] * times;
		cx = list_of_ints[0] * times;
		cy = list_of_ints[1] * times;
	}

	float factor = 10000.0f;

	cloud->points.resize(imDepth.rows * imDepth.cols);

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {
			unsigned short Zw = imDepth.at<unsigned short>(i, j);
			if (Zw != 0) {
				//std::cout << Zw << std::endl;
				point_st myp;

				myp.z = (float)(Zw / factor);
				myp.x = (j - cx + cx_offset) * myp.z / fx;
				myp.y = (i - cy + cy_offset) * myp.z / fy;

				cloud->points[i*imDepth.cols + j].x = myp.x;
				cloud->points[i*imDepth.cols + j].y = myp.y;
				cloud->points[i*imDepth.cols + j].z = myp.z;
			}
		}
	}
}

void pcd2ply(std::string filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd)
{
	std::ofstream ply(filePath.c_str());

	int num_p = pcd->points.size();

	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < pcd->points.size(); i++) {
		//std::cout << points[i].x << std::endl;
		//std::cout << points[i].y << std::endl;
		//std::cout << points[i].z << std::endl;
		ply << pcd->points[i].x << "     " << pcd->points[i].y << "      " << pcd->points[i].z << "\n";
	}
	ply.close();
}

void pcd2ply(std::string filePath, pcl::PointCloud<pcl::PointXYZ> *pcd)
{
	std::ofstream ply(filePath.c_str());

	int num_p = pcd->points.size();

	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < pcd->points.size(); i++) {
		//std::cout << points[i].x << std::endl;
		//std::cout << points[i].y << std::endl;
		//std::cout << points[i].z << std::endl;
		ply << pcd->points[i].x << "     " << pcd->points[i].y << "      " << pcd->points[i].z << "\n";
	}
	ply.close();
}

//创建两个pcl::PointCloud<pcl::PointXYZ>共享指针，并初始化它们
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

void pcl_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);                 //cloud_in设置为点云的源点
	icp.setInputTarget(cloud_out);               //cloud_out设置为与cloud_in对应的匹配目标
	pcl::PointCloud<pcl::PointXYZ> Final;         //存储经过配准变换点云后的点云

	icp.setMaximumIterations(1000);  //最大迭代次数
	icp.setEuclideanFitnessEpsilon(1e-10);//前后两次迭代误差的差值
	icp.setTransformationEpsilon(1e-10); //上次转换与当前转换的差值；
	icp.setMaxCorrespondenceDistance(100); //忽略在此距离之外的点，对配准影响较大

	cout << "icp align start!!" << endl;
	icp.align(Final);
	pcd2ply("E:\\Project\\demo\\dataset\\TEST_4\\Final.ply", &Final);
	cout << "icp align end!!" << endl;

}

int main(int argc, char** argv)
{
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned short* depth_image0 = new unsigned short[(DEP_W * DEP_H) / (DECIMATE_DIV*DECIMATE_DIV)];

	int align = 0;

	MyCam.StartAllCam();

	for (;;)
	{
		if (align) {
			//MyCam.grabAndFilterAlignedDeptFrame(0, rgb_image0, depth_image0);
		}
		else
		{
			MyCam.grabAndFilterDeptFrame(0, rgb_image0, depth_image0);
		}

		//show rgb window
		cv::Mat frame_rgb(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		cv::cvtColor(frame_rgb, frame_rgb, CV_RGB2BGR);
		cv::imshow("rgb", frame_rgb);

		//show depth window
		cv::Mat frame_depth(DEP_H / DECIMATE_DIV, DEP_W / DECIMATE_DIV, CV_16UC1, depth_image0);
		cv::imshow("depth", frame_depth);

		//key handle
		int keycode = waitKey(1);
		if (keycode == 27)//Esc
			break;
		switch (keycode)
		{
		case 'i':
			depth2pcd(frame_depth, cloud_in, 0, 0, 1.0 / DECIMATE_DIV, align);
			pcd2ply("E:\\Project\\demo\\dataset\\TEST_4\\cloud_in.ply", cloud_in);
			break;
		case 'o':
			depth2pcd(frame_depth, cloud_out, 0, 0, 1.0 / DECIMATE_DIV, align);
			pcd2ply("E:\\Project\\demo\\dataset\\TEST_4\\cloud_out.ply", cloud_out);
			break;
		case 'c':
			pcl_icp(cloud_in, cloud_out);
			break;
		}
	}

	std::cout << "finished!!" << std::endl;

	return 0;
}
#elif defined TEST_4_1

#include "CloudMapper.h"

#pragma comment( lib,"winmm.lib" )

using namespace std;
using namespace cv;
using namespace hpe;

struct FileData {
	string ply;
	string format;
	int element;
	string propx;
	string propy;
	string propz;
	string end;

	std::vector<Point3f> vet;
};

float stringToFloat(const string& str)
{
	istringstream iss(str);
	float num;
	iss >> num;
	return num;
}

int stringToInt(const string& str)
{
	istringstream iss(str);
	int num;
	iss >> num;
	return num;
}

void getPlyData(string fileContent, struct FileData *ply) {

	if (fileContent.c_str() == nullptr)
	{
		return;
	}

	//强转字节流
	ifstream myfile(fileContent);
	string line;
	int count = 0;
	while (!myfile.eof())
	{
		getline(myfile, line);
		std::istringstream iss(line);
		if (strlen(line.c_str()) > 0) {
			if (line[0] == 'p' &&line[1] == 'l')
			{
				ply->ply = iss.str();
			}
			else if (line[0] == 'f' &&line[1] == 'o')
			{
				ply->format = iss.str();
			}
			else if (line[0] == 'e' && line[1] == 'l')
			{
				string element, vertex, num;
				iss >> element >> vertex >> num;
				ply->element = stringToInt(num);
			}
			else if (!line.compare("property float x"))
			{
				ply->propx = iss.str();
			}
			else if (!line.compare("property float y"))
			{
				ply->propy = iss.str();
			}
			else if (!line.compare("property float z"))
			{
				ply->propz = iss.str();
			}
			else if (line[0] == 'e' && line[1] == 'n')
			{
				ply->end = iss.str();
			}
			else
			{
				string x, y, z;
				Point3f vet;
				iss >> x >> y >> z;
				vet.x = stringToFloat(x);
				vet.y = stringToFloat(y);
				vet.z = stringToFloat(z);
				ply->vet.push_back(vet);
			}
		}
	}

	ply->vet.resize(ply->element);

	myfile.close();
}

void setPlyData(string filePath, struct FileData *plyData)
{
	std::ofstream ply(filePath.c_str());

	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << plyData->vet.size() << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < plyData->vet.size(); i++) {
		ply << plyData->vet[i].x << "     " << plyData->vet[i].y << "      " << plyData->vet[i].z << "\n";
	}
	ply.close();
}

void flip(short *src, short *dst, int w, int h, int mode)
{
	short *buf = new short[w * h];

	if (mode == 0) {//up/down
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				buf[w * (h - 1 - i) + j] = src[w * i + j];
			}
		}
	}
	else if (mode == 1) {//left/right
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				buf[w * i + (w - 1 - j)] = src[w * i + j];
			}
		}
	}
	else if (mode == 2) {//up/down/left/right
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				buf[w * (h - 1 - i) + (w - 1 - j)] = src[w * i + j];
			}
		}
	}

	memcpy(dst, buf, w * h * sizeof(short));

	delete[]buf;
}

void rotate(short *src, short *dst, int w, int h, int mode)
{
	short *buf = new short[w * h];

	if (mode == 0) //clockwise
	{
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				buf[(j + 1) * h - 1 - i] = src[w * i + j];
			}
		}
	}
	else if (mode == 1) //anticlockwise
	{
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				buf[(w - 1 - j) * h + i] = src[w * i + j];
			}
		}
	}

	memcpy(dst, buf, w * h * sizeof(short));

	delete[]buf;
}

void removeXYNoise(short *src, const short invalidvalue, const short minval, int w, int h)
{
	const int MAX_LENGTH = (int)(w * 0.03);

	short *s = new short[w];

	for (int j = 0; j < h; j++)
	{
		memcpy(s, src + j * w, w * sizeof(short));
		s[0] = minval;
		s[w - 1] = minval;
		for (int i = 1; i < w - 1; i++)
		{
			if (s[i] > invalidvalue)
			{
				int t = i;
				do
				{
					t++;
					if (t > w - 2)break;
				} while (s[t] > invalidvalue);

				if (t - i <= MAX_LENGTH)
				{
					for (; i < t; i++)
					{
						s[i] = invalidvalue;
					}
				}
				else
				{
					i = t;
				}
			}
		}
		s[0] = s[1];
		s[w - 1] = s[w - 2];

		memcpy(src + j * w, s, w * sizeof(short));
	}

	delete[]s;
}

void removeNoise_short(short* src, short* dst, int w, int h)
{
	short *tp = new short[w * h];
	short *fp = new short[w * h];
	short invalidvalue = 0;

	removeXYNoise(src, invalidvalue, 0, w, h);
	rotate(src, tp, w, h, 1);
	removeXYNoise(tp, invalidvalue, 0, h, w);
	rotate(tp, dst, h, w, 0);

	delete[]tp;
	delete[]fp;
}

void pcd2ply(std::string filePath, CloudMapper::Cloud::Ptr &pcd)
{
	std::ofstream ply(filePath.c_str());

	int num_p = pcd->points.size();

	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < pcd->points.size(); i++) {
		ply << pcd->points[i].x << "     " << pcd->points[i].y << "      " << pcd->points[i].z << "\n";
	}
	ply.close();
}

void depth2pcd(cv::Mat imDepth, CloudMapper::Cloud::Ptr &cloud)
{
	long cloud_size = 0;

	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);
	float fx, fy, cx, cy;

	fx = list_of_ints[6];
	fy = list_of_ints[7];
	cx = list_of_ints[4];
	cy = list_of_ints[5];

	float factor = 10000.0f;

	cloud->points.resize(imDepth.rows * imDepth.cols);

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {
			unsigned short Zw = imDepth.at<unsigned short>(i, j);
			if (Zw != 0) {
				cloud->points[cloud_size].z = (float)(Zw / factor);
				cloud->points[cloud_size].x = (j - cx) * cloud->points[cloud_size].z / fx;
				cloud->points[cloud_size].y = (i - cy) * cloud->points[cloud_size].z / fy;
				cloud_size++;
			}
		}
	}

	cloud->points.resize(cloud_size);

	cloud->width = cloud_size;
	cloud->height = 1;
	cloud->is_dense = false;
}

void ply2pcd(std::string plyFile, CloudMapper::Cloud::Ptr &cloud)
{
	struct FileData ply;

	getPlyData(plyFile, &ply);

	cloud->points.resize(ply.element);

	for (int i = 0; i<ply.element; i++)
	{
		cloud->points[i].x = ply.vet[i].x;
		cloud->points[i].y = ply.vet[i].y;
		cloud->points[i].z = ply.vet[i].z;
	}

	cloud->width = ply.element;
	cloud->height = 1;
	cloud->is_dense = false;
}

void pcd2csv(CloudMapper::Cloud::Ptr &cloud, string csvFile)
{
	std::ofstream csv(csvFile.c_str());

	for (int i = 0; i < cloud->points.size(); i++) {
		csv << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << "\n";
	}

	csv.close();
}

void voxel_filter(CloudMapper::Cloud::Ptr &input, CloudMapper::Cloud::Ptr &output)
{
	CloudMapper::Voxel vg;

	vg.setInputCloud(input);
	vg.setLeafSize(0.002f, 0.002f, 0.002f);
	vg.filter(*output);
}

void outlier_remove(CloudMapper::Cloud::Ptr &input, CloudMapper::Cloud::Ptr &output)
{
	pcl::RadiusOutlierRemoval<CloudMapper::PointType> outrem;
	outrem.setInputCloud(input);
	outrem.setRadiusSearch(0.04);
	outrem.setMinNeighborsInRadius(80);
	outrem.filter(*output);
}

Eigen::Matrix4f pcl_icp(CloudMapper::Cloud::Ptr &source, CloudMapper::Cloud::Ptr &target)
{
	//ICP初始化
	CloudMapper m_mapper;

	m_mapper.SetUseNormalShooting(false);

	return m_mapper.GetTransformForTwoCloudsDynamically(source, target);
}

int main(int argc, char** argv)
{
	//相机数据buffer
	char* rgb_image0 = new char[RGB_W * RGB_H * 3];
	short* depth_image0 = new short[DEP_W * DEP_H];

	//点云buffer
	CloudMapper::Cloud::Ptr cloud_in_outlier(new CloudMapper::Cloud);
	CloudMapper::Cloud::Ptr cloud_out_outlier(new CloudMapper::Cloud);
	CloudMapper::Cloud::Ptr cloud_in_vg(new CloudMapper::Cloud);
	CloudMapper::Cloud::Ptr cloud_out_vg(new CloudMapper::Cloud);
	CloudMapper::Cloud::Ptr cloud_in(new CloudMapper::Cloud);
	CloudMapper::Cloud::Ptr cloud_out(new CloudMapper::Cloud);

	//初始化RT矩阵
	Eigen::Matrix4f m_aggregatedTransform = Eigen::Matrix4f::Identity();

	//打开相机
	MyCam.StartAllCam();

	for (;;)
	{
		MyCam.grabAndDeptFrame(0, rgb_image0, depth_image0);

		//show rgb window
		//cv::Mat frame_rgb(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		//cv::cvtColor(frame_rgb, frame_rgb, CV_RGB2BGR);
		//cv::imshow("rgb", frame_rgb);

		//show depth window
		removeNoise_short(depth_image0, depth_image0, DEP_W, DEP_H);
		cv::Mat frame_depth(DEP_H, DEP_W, CV_16UC1, depth_image0);
		cv::imshow("depth", frame_depth);

		//key handle
		int keycode = waitKey(10);
		if (keycode == 27)//Esc
			break;
		switch (keycode)
		{
		case 'i':
			ply2pcd("C:\\Users\\Administrator\\Desktop\\ttt\\model.ply", cloud_in);

			//imwrite("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in.png", frame_depth);
			//depth2pcd(frame_depth, cloud_in);
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in.ply", cloud_in);
			voxel_filter(cloud_in, cloud_in_vg);

			//pcd2csv(cloud_in, "C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in.csv");
			pcd2ply("C:\\Users\\Administrator\\Desktop\\ttt\\model_1.ply", cloud_in_vg);
			//outlier_remove(cloud_in, cloud_in_outlier);
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in_outlier.ply", cloud_in_outlier);
			//voxel_filter(cloud_in, cloud_in_vg);
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in_vg.ply", cloud_in_vg);
			//voxel_filter(cloud_in, cloud_in_vg);
			break;
		case 'o':
			//imwrite("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_out.png", frame_depth);
			//depth2pcd(frame_depth, cloud_out);
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_out.ply", cloud_out);
			//voxel_filter(cloud_out, cloud_out_vg);
			//pcd2csv(cloud_out, "C:\\Users\\Administrator\\Desktop\\pcl\\cloud_out.csv");
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_out_vg.ply", cloud_out_vg);
			//outlier_remove(cloud_out, cloud_out_outlier);
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_out_outlier.ply", cloud_out_outlier);
			//voxel_filter(cloud_out, cloud_out_vg);
			//pcd2ply("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_out_vg.ply", cloud_out_vg);
			//voxel_filter(cloud_out, cloud_out_vg);
			break;
		case 'c':
			std::cout << "pcl_icp start:" << timeGetTime() << std::endl;
			m_aggregatedTransform = pcl_icp(cloud_in_vg, cloud_out_vg);
			std::cout << "pcl_icp end:" << timeGetTime() << std::endl;
			cout << "RT:" << m_aggregatedTransform << endl;
			break;
		case 'p':
			struct FileData ply;
			getPlyData("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in.ply", &ply);
			for (int i=0; i<ply.vet.size();i++)
			{
				Eigen::Vector4f vex(ply.vet[i].x, ply.vet[i].y, ply.vet[i].z, 1);
				Eigen::Vector4f pos = m_aggregatedTransform * vex;
				ply.vet[i].x = pos[0];
				ply.vet[i].y = pos[1];
				ply.vet[i].z = pos[2];
			}	
			setPlyData("C:\\Users\\Administrator\\Desktop\\pcl\\cloud_in1.ply", &ply);
			break;
		}
	}

	return 0;
}

#elif defined TEST_5

int main(int argc, char** argv)
{
	cv::Mat red(500, 500, CV_8UC3, cv::Scalar(255, 0, 0));
	cv::Mat blue(500, 500, CV_8UC3, cv::Scalar(0, 0, 255));
	cv::Mat blank(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));

	for (;;)
	{
		cv::imshow("red", red);
		cv::imshow("blue", blue);

		//cv::Mat blend(500,750, CV_8UC3, cv::Scalar(0, 0, 0));
		//red.copyTo(blend(cv::Rect(0, 0, red.cols, red.rows)));
		//blue.copyTo(blend(cv::Rect(250, 0, blue.cols, blue.rows)));
		//cv::imshow("blend", blend);

		double alpha = 1.0f;

		for (int i = 0; i < red.rows; i++)
		{
			uchar *p = red.ptr<uchar>(i);
			uchar *t = blue.ptr<uchar>(i);
			uchar *d = blank.ptr<uchar>(i);

			for (int j = 0; j < red.cols; j++)
			{
				if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
				{
					alpha = 1.0f;
				}
				else
				{
					alpha = double(red.cols - j) / red.cols;
				}

				d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
				d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
				d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

				//std::cout << "alhpa:" << alpha << std::endl;
				//std::cout << "d0:" << d[j * 3] <<"d1:"<< d[j * 3 + 1] <<"d2:"<< d[j * 3 + 2] << std::endl;
			}

		}

		cv::imshow("blank", blank);


		int keycode = cv::waitKey(10);
		if (keycode == 27)//Esc
			break;
	}

	std::cout << "finished!!" << std::endl;

	return 0;
}

#elif defined TEST_6
int main(int argc, char** argv)
{
	cv::Mat red(500, 500, CV_8UC3, cv::Scalar(0, 0, 255));
	cv::Mat blue(500, 500, CV_8UC3, cv::Scalar(255, 0, 0));
	cv::Mat blend(500, 750, CV_8UC3, cv::Scalar(0, 0, 0));

	//cv::Mat patch(500, 250, CV_8UC3, cv::Scalar(0, 255, 0));


	for (;;)
	{

		//patch.copyTo(blue(cv::Rect(0, 0, patch.cols, patch.rows)));

		cv::imshow("red", red);
		cv::imshow("blue", blue);

		red.copyTo(blend(cv::Rect(0, 0, red.cols, red.rows)));
		blue.copyTo(blend(cv::Rect(250, 0, blue.cols, blue.rows)));

		cv::imshow("blend", blend);

		int start = 250;
		double alpha = 1.0f;
		double processWidth = red.cols - start;

		for (int i = 0; i < red.rows; i++)
		{
			uchar *p = red.ptr<uchar>(i);
			uchar *t = blue.ptr<uchar>(i);
			uchar *d = blend.ptr<uchar>(i);

			for (int j = start; j < red.cols; j++)
			{
				if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
				{
					alpha = 1.0f;
				}
				else
				{
					alpha = double(red.cols - (j - start)) / processWidth;
				}

				d[j * 3] = p[j * 3] * alpha + t[(j - start) * 3] * (1 - alpha);
				d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[(j - start) * 3 + 1] * (1 - alpha);
				d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[(j - start) * 3 + 2] * (1 - alpha);

				//std::cout << "alhpa:" << alpha << std::endl;
				//std::cout << "d0:" << d[j * 3] <<"d1:"<< d[j * 3 + 1] <<"d2:"<< d[j * 3 + 2] << std::endl;
			}

		}

		cv::imshow("blend1", blend);

		int keycode = cv::waitKey(10);
		if (keycode == 27)//Esc
			break;
	}

	std::cout << "finished!!" << std::endl;

	return 0;
}
#elif defined TEST_7

int stringToNum(const string& str)
{
	istringstream iss(str);
	int num;
	iss >> num;
	return num;
}

typedef struct border_data {
	int left_l;
	int left_c;
	int left_r;

	int right_l;
	int right_c;
	int right_r;
};

int main(int argc, char** argv)
{
	std::vector<border_data> bData;

	cv::Mat left = cv::imread("C:\\Users\\Administrator\\Desktop\\output2\\left_new.jpg");
	cv::Mat right = cv::imread("C:\\Users\\Administrator\\Desktop\\output2\\right_new.jpg");
	cv::Mat center = cv::imread("C:\\Users\\Administrator\\Desktop\\output2\\center_new.jpg");

	cv::Mat blend(center.rows, center.cols, CV_8UC3, cv::Scalar(0, 0, 0));

	std::string fileContent = "C:\\Users\\Administrator\\Desktop\\output2\\range.txt";
	ifstream myfile(fileContent);
	string line;

	while (!myfile.eof())
	{
		getline(myfile, line);
		std::istringstream iss(line);
		if (strlen(line.c_str()) > 0) {
			string a, b, c, d, e, f;
			border_data border;
			iss >> a >> b >> c >> d >> e >> f;

			border.left_l = stringToNum(c);
			border.left_c = stringToNum(a);
			border.left_r = stringToNum(d);
			border.right_l = stringToNum(e);
			border.right_c = stringToNum(b);
			border.right_r = stringToNum(f);

			bData.push_back(border);
		}
	}

	for (int i = 0; i < blend.rows; i++)
	{
		uchar *l = left.ptr<uchar>(i);
		uchar *c = center.ptr<uchar>(i);
		uchar *r = right.ptr<uchar>(i);
		uchar *b = blend.ptr<uchar>(i);

		for (int j = 0; j < blend.cols; j++)
		{
			if (j <= bData[i].left_c)
			{
				b[j * 3] = l[j * 3];
				b[j * 3 + 1] = l[j * 3 + 1];
				b[j * 3 + 2] = l[j * 3 + 2];
			}
			else if (j > bData[i].left_c && j <= bData[i].right_c)
			{
				b[j * 3] = c[j * 3];
				b[j * 3 + 1] = c[j * 3 + 1];
				b[j * 3 + 2] = c[j * 3 + 2];
			}
			else if (j > bData[i].right_c)
			{
				b[j * 3] = r[j * 3];
				b[j * 3 + 1] = r[j * 3 + 1];
				b[j * 3 + 2] = r[j * 3 + 2];
			}
		}
	}

	cv::imwrite("C:\\Users\\Administrator\\Desktop\\output2\\blend_ori.jpg", blend);

	/* left to center */
	for (int i = 0; i < center.rows; i++)
	{
		int start = bData[i].left_l;
		int end = bData[i].left_r;
		double processWidth = end - start;
		double alpha = 1.0f;

		uchar *p = left.ptr<uchar>(i);
		uchar *t = center.ptr<uchar>(i);
		uchar *d = blend.ptr<uchar>(i);

		for (int j = start; j < end; j++)
		{
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1.0f;
			}
			else if (p[j * 3] == 0 && p[j * 3 + 1] == 0 && p[j * 3 + 2] == 0)
			{
				alpha = 0.0f;
			}
			else
			{
				alpha = (processWidth - (j - start)) / processWidth;
			}

			d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1.0f - alpha);
			d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1.0f - alpha);
			d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1.0f - alpha);
		}
	}

	/* right to center */
	for (int i = 0; i < center.rows; i++)
	{
		int start = bData[i].right_l;
		int end = bData[i].right_r;
		double processWidth = end - start;
		double alpha = 1.0f;

		uchar *p = right.ptr<uchar>(i);
		uchar *t = center.ptr<uchar>(i);
		uchar *d = blend.ptr<uchar>(i);

		for (int j = start; j < end; j++)
		{
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 0.0f;
			}
			else if (p[j * 3] == 0 && p[j * 3 + 1] == 0 && p[j * 3 + 2] == 0)
			{
				alpha = 1.0f;
			}
			else
			{
				alpha = (processWidth - (j - start)) / processWidth;
			}

			d[j * 3] = p[j * 3] * (1.0f - alpha) + t[j * 3] * alpha;
			d[j * 3 + 1] = p[j * 3 + 1] * (1.0f - alpha) + t[j * 3 + 1] * alpha;
			d[j * 3 + 2] = p[j * 3 + 2] * (1.0f - alpha) + t[j * 3 + 2] * alpha;
		}
	}

	cv::imwrite("C:\\Users\\Administrator\\Desktop\\output2\\blend.jpg", blend);

	std::cout << "finished!!" << std::endl;

	return 0;
}

#elif defined TEST_8

using namespace cv;

/*** gamma ***/
//void MyGammaCorrection(Mat& src, Mat& dst, float fGamma) {
//	CV_Assert(src.data); 	// accept only char type matrices	
//	CV_Assert(src.depth() != sizeof(uchar)); 	
//	
//	// build look up table	
//	unsigned char lut[256];	
//	for( int i = 0; i < 256; i++ )	
//	{		
//		lut[i] = saturate_cast<uchar>(pow((float)(i/255.0), fGamma) * 255.0f);	
//	} 	
//	dst = src.clone();	
//	const int channels = dst.channels();	
//	switch(channels)	
//	{		
//		case 1:			
//		{ 				
//			MatIterator_<uchar> it, end;				
//			for( it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++ )					
//				//*it = pow((float)(((*it))/255.0), fGamma) * 255.0;					
//				*it = lut[(*it)]; 				
//			break;			
//		}		
//		case 3: 			
//		{ 				
//			MatIterator_<Vec3b> it, end;				
//			for( it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++ )				
//			{					
//				//(*it)[0] = pow((float)(((*it)[0])/255.0), fGamma) * 255.0;					
//				//(*it)[1] = pow((float)(((*it)[1])/255.0), fGamma) * 255.0;					
//				//(*it)[2] = pow((float)(((*it)[2])/255.0), fGamma) * 255.0;					
//				(*it)[0] = lut[((*it)[0])];					
//				(*it)[1] = lut[((*it)[1])];					
//				(*it)[2] = lut[((*it)[2])];				
//			} 				
//			break; 			
//		}	
//	}
//}
//
//
//int main(int argc, char** argv)
//{
//
//	cv::Mat img = cv::imread("C:\\Users\\Administrator\\Desktop\\test\\final_obj_1.jpg");
//	
//	cv::Mat dst;
//
//	MyGammaCorrection(img,dst,1.5);
//
//	cv::imwrite("C:\\Users\\Administrator\\Desktop\\test\\final_obj.jpg", dst);
//
//	std::cout << "finished!!" << std::endl;
//
//	return 0;
//}

/*** white balance ***/
int main(int argc, char** argv)
{
	cv::Mat imageSource_center = cv::imread("C:\\Users\\Administrator\\Desktop\\output2\\center.jpg");
	cv::Mat imageSource_right = cv::imread("C:\\Users\\Administrator\\Desktop\\output2\\right.jpg");
	cv::Mat imageSource_left = cv::imread("C:\\Users\\Administrator\\Desktop\\output2\\left.jpg");
	vector<Mat> imageRGB_c;
	vector<Mat> imageRGB_r;
	vector<Mat> imageRGB_l;

	split(imageSource_center, imageRGB_c);
	split(imageSource_right, imageRGB_r);
	split(imageSource_left, imageRGB_l);

	double R, G, B;

	B = mean(imageRGB_c[0])[0];
	G = mean(imageRGB_c[1])[0];
	R = mean(imageRGB_c[2])[0];

	double KR, KG, KB;

	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);

	imageRGB_c[0] = imageRGB_c[0] * KB;
	imageRGB_c[1] = imageRGB_c[1] * KG;
	imageRGB_c[2] = imageRGB_c[2] * KR;
	merge(imageRGB_c, imageSource_center);

	imageRGB_r[0] = imageRGB_r[0] * KB;
	imageRGB_r[1] = imageRGB_r[1] * KG;
	imageRGB_r[2] = imageRGB_r[2] * KR;
	merge(imageRGB_r, imageSource_right);

	imageRGB_l[0] = imageRGB_l[0] * KB;
	imageRGB_l[1] = imageRGB_l[1] * KG;
	imageRGB_l[2] = imageRGB_l[2] * KR;
	merge(imageRGB_l, imageSource_left);

	cv::imwrite("C:\\Users\\Administrator\\Desktop\\output2\\center_new.jpg", imageSource_center);
	cv::imwrite("C:\\Users\\Administrator\\Desktop\\output2\\right_new.jpg", imageSource_right);
	cv::imwrite("C:\\Users\\Administrator\\Desktop\\output2\\left_new.jpg", imageSource_left);

	std::cout << "finished!!" << std::endl;

	return 0;
}

#elif defined TEST_9
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv/cv_image_abstract.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace dlib;
using namespace std;

// ----------------------------------------------------------------------------------------

/* detect 68 face landmarks on the input image by using the face landmark detector in dlib.
//
*/
void faceLandmarkDetection(dlib::array2d<unsigned char>& img, shape_predictor sp, std::vector<Point2f>& landmark)
{
	dlib::frontal_face_detector detector = get_frontal_face_detector();

	std::vector<dlib::rectangle> dets = detector(img);

	full_object_detection shape = sp(img, dets[0]);

	//for (int i = 0; i < shape.num_parts(); ++i)
	for (int i = 0; i < 60; ++i) // Removal of black spots on the mouth 
	{
		float x = shape.part(i).x();
		float y = shape.part(i).y();
		landmark.push_back(Point2f(x, y));
	}
}


/*
//add eight keypoints to the keypoints set of the input image.
//the added eight keypoints are the four corners points of the image, plus four median points of the four edges of the image.
*/

void addKeypoints(std::vector<Point2f>& points, Size imgSize)
{
	points.push_back(Point2f(1, 1));
	points.push_back(Point2f(1, imgSize.height - 1));
	points.push_back(Point2f(imgSize.width - 1, imgSize.height - 1));
	points.push_back(Point2f(imgSize.width - 1, 1));
	points.push_back(Point2f(1, imgSize.height / 2));
	points.push_back(Point2f(imgSize.width / 2, imgSize.height - 1));
	points.push_back(Point2f(imgSize.width - 1, imgSize.height / 2));
	points.push_back(Point2f(imgSize.width / 2, 1));
}



/*
// calculate the keypoints on the morph image.
*/

void morpKeypoints(const std::vector<Point2f>& points1, const std::vector<Point2f>& points2, std::vector<Point2f>& pointsMorph, double alpha)
{
	for (int i = 0; i < points1.size(); i++)
	{
		float x, y;
		x = (1 - alpha) * points1[i].x + alpha * points2[i].x;
		y = (1 - alpha) * points1[i].y + alpha * points2[i].y;

		pointsMorph.push_back(Point2f(x, y));
	}
}


/*
//perform Delaunay Triangulation on the keypoints of the morph image.
*/
typedef struct correspondens {
	std::vector<int> index;
};

void delaunayTriangulation(const std::vector<Point2f>& points1, const std::vector<Point2f>& points2,
	std::vector<Point2f>& pointsMorph, double alpha, std::vector<correspondens>& delaunayTri, Size imgSize)
{
	morpKeypoints(points1, points2, pointsMorph, 0);

	Rect rect(0, 0, imgSize.width, imgSize.height);

	cv::Subdiv2D subdiv(rect);
	for (std::vector<Point2f>::iterator it = pointsMorph.begin(); it != pointsMorph.end(); it++)
		subdiv.insert(*it);

	std::vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);

	//cout << triangleList.size() << endl;
	for (size_t i = 0; i < triangleList.size(); ++i)
	{
		std::vector<Point2f> pt;
		correspondens ind;
		Vec6f t = triangleList[i];
		pt.push_back(Point2f(t[0], t[1]));
		pt.push_back(Point2f(t[2], t[3]));
		pt.push_back(Point2f(t[4], t[5]));

		if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
		{
			int count = 0;
			for (int j = 0; j < 3; ++j)
				for (size_t k = 0; k < pointsMorph.size(); k++) {
					//cout << "pt[j].x:" << pt[j].x << "---x:" << pointsMorph[k].x << endl;
					//cout << "pt[j].y:" << pt[j].y << "---y:" << pointsMorph[k].y << endl;
					if (abs(pt[j].x - pointsMorph[k].x) < 1.0   &&  abs(pt[j].y - pointsMorph[k].y) < 1.0)
					{
						ind.index.push_back(k);
						count++;
					}
				}
			if (count == 3) {
				//cout << "index is " << ind.index[0] << " " << ind.index[1] << " " << ind.index[2] << endl;
				delaunayTri.push_back(ind);
			}
		}
	}

	//cout << delaunayTri.size() << endl;
}


/*
// apply affine transform on one triangle.
*/
void applyAffineTransform(Mat &warpImage, Mat &src, std::vector<Point2f> & srcTri, std::vector<Point2f> & dstTri)
{
	Mat warpMat = getAffineTransform(srcTri, dstTri);

	warpAffine(src, warpImage, warpMat, warpImage.size(), cv::INTER_LINEAR, BORDER_REFLECT_101);
}


/*
//the core function of face morph.
//morph the two input image to the morph image by transacting the set of triangles in the two input image to the morph image.
*/
void morphTriangle(Mat &img1, Mat &img2, Mat &img, std::vector<Point2f> &t1, std::vector<Point2f> &t2, std::vector<Point2f> &t, double alpha)
{
	Rect r = cv::boundingRect(t);
	Rect r1 = cv::boundingRect(t1);
	Rect r2 = cv::boundingRect(t2);

	std::vector<Point2f> t1Rect, t2Rect, tRect;
	std::vector<Point> tRectInt;
	for (int i = 0; i < 3; ++i)
	{
		tRect.push_back(Point2f(t[i].x - r.x, t[i].y - r.y));
		tRectInt.push_back(Point(t[i].x - r.x, t[i].y - r.y));

		t1Rect.push_back(Point2f(t1[i].x - r1.x, t1[i].y - r1.y));
		t2Rect.push_back(Point2f(t2[i].x - r2.x, t2[i].y - r2.y));
	}

	Mat mask = Mat::zeros(r.height, r.width, CV_32FC3);
	fillConvexPoly(mask, tRectInt, Scalar(1.0, 1.0, 1.0), 16, 0);

	Mat img1Rect, img2Rect;
	img1(r1).copyTo(img1Rect);
	img2(r2).copyTo(img2Rect);

	Mat warpImage1 = Mat::zeros(r.height, r.width, img1Rect.type());
	Mat warpImage2 = Mat::zeros(r.height, r.width, img2Rect.type());

	applyAffineTransform(warpImage1, img1Rect, t1Rect, tRect);
	applyAffineTransform(warpImage2, img2Rect, t2Rect, tRect);

	Mat imgRect = (1.0 - alpha)*warpImage1 + alpha*warpImage2;

	multiply(imgRect, mask, imgRect);
	multiply(img(r), Scalar(1.0, 1.0, 1.0) - mask, img(r));
	img(r) = img(r) + imgRect;
}




/*
//morp the two input images into the morph image.
//first get the keypoints correspondents of the set of  triangles, then call the core function.
*/
void morp(Mat &img1, Mat &img2, Mat& imgMorph, double alpha, const std::vector<Point2f> &points1, const std::vector<Point2f> &points2, const std::vector<correspondens> &triangle)
{
	img1.convertTo(img1, CV_32F);
	img2.convertTo(img2, CV_32F);

	std::vector<Point2f> points;
	morpKeypoints(points1, points2, points, 0);

	int x, y, z;
	int count = 0;
	for (int i = 0; i < triangle.size(); ++i)
	{
		correspondens corpd = triangle[i];
		x = corpd.index[0];
		y = corpd.index[1];
		z = corpd.index[2];
		std::vector<Point2f> t1, t2, t;
		t1.push_back(points1[x]);
		t1.push_back(points1[y]);
		t1.push_back(points1[z]);

		t2.push_back(points2[x]);
		t2.push_back(points2[y]);
		t2.push_back(points2[z]);

		t.push_back(points[x]);
		t.push_back(points[y]);
		t.push_back(points[z]);
		morphTriangle(img1, img2, imgMorph, t1, t2, t, alpha);
	}

}

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		cout << "Give some image files as arguments to this program." << endl;
		return 0;
	}

	argv[1] = "E:\\Project\\demo\\x64\\Release\\ym.jpg";
	argv[2] = "E:\\Project\\demo\\x64\\Release\\zjq.jpg";
	//-------------- step 1. load the input two images --------------------------------------------       
	shape_predictor sp;
	deserialize("E:\\3rd_lib\\dlib-19.9\\shape_predictor_68_face_landmarks.dat") >> sp;
	dlib::array2d<unsigned char> img1, img2;
	dlib::load_image(img1, argv[1]);
	dlib::load_image(img2, argv[2]);
	std::vector<Point2f> landmarks1, landmarks2;

	Mat img1CV = imread(argv[1]);
	Mat img2CV = imread(argv[2]);
	if (!img1CV.data || !img2CV.data)
	{
		printf("No image data \n");
		return -1;
	}
	else
		cout << "image readed by opencv" << endl;


	//----------------- step 2. detect face landmarks ---------------------------------------------
	faceLandmarkDetection(img1, sp, landmarks1);
	faceLandmarkDetection(img2, sp, landmarks2);
	cout << "landmark2 number is " << landmarks2.size() << endl;

	//add some land marks in the edges to get better performance.
	addKeypoints(landmarks1, img1CV.size());
	addKeypoints(landmarks2, img2CV.size());

	cout << "landmark number after added is " << landmarks1.size() << endl;

	//for (int i=0;i<landmarks1.size();++i)
	//{
	//circle(img1CV, landmarks1[i], 2, CV_RGB(255, 255, 255), 1);
	//}
	//imshow("landmark",img1CV);
	//cv::waitKey(10);



	//--------------- step 3. face morp ----------------------------------------------
	std::vector<Mat> resultImage;
	resultImage.push_back(img1CV);
	cout << "add the first image" << endl;
	for (double alpha = 0.1; alpha < 1; alpha += 0.1)
	{
		Mat imgMorph = Mat::zeros(img1CV.size(), CV_32FC3);
		std::vector<Point2f> pointsMorph;

		std::vector<correspondens> delaunayTri;
		delaunayTriangulation(landmarks1, landmarks2, pointsMorph, alpha, delaunayTri, img1CV.size());

		morp(img1CV, img2CV, imgMorph, alpha, landmarks1, landmarks2, delaunayTri);

		resultImage.push_back(imgMorph);
	}
	resultImage.push_back(img2CV);

	for (int i = 0; i < resultImage.size(); ++i)
	{

		//output_src<<resultImage[i];
		string st = argv[1];
		char t[20];
		sprintf(t, "%d", i);
		st = st + t;
		st = st + ".jpg";
		imwrite(st, resultImage[i]);
	}

	return 0;
}

#elif defined TEST_10
#include <fstream>
#include <iostream>

#include <Eigen\Eigen>
#include <opencv2/opencv.hpp>

#define COLOR_PLY

using namespace std;
using namespace cv;

void depth2PointCloud(cv::Mat imDepth, cv::Mat imRGB, std::string filePath)
{
	float fx = 8.4479804659324691e+02;//RK intrix depth
	float fy = 8.4257091093148335e+02;//RK intrix depth
	float cx = 3.4277563623854769e+02;//RK intrix depth
	float cy = 6.6073738924054089e+02;//RK intrix depth

	float fx_color = 1.5203292707447745e+03;//RK intrix rgb
	float fy_color = 1.5169691588804969e+03;//RK intrix rgb
	float cx_color = 4.9872383236506693e+02;//RK intrix rgb
	float cy_color = 9.5411039730726077e+02;//RK intrix rgb

	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {

			unsigned short disp = imDepth.at<unsigned short>(i, j);
			if (disp != 0) {

				float f = (fx + fy) / 2;
				point_st myp;

				float u = 0.003;//pixel size, unit:mm
				float s = 40.0;//基线距离, unit:mm
				float d = 600.0;//参考帧距离, unit:mm
				float f_mm = f * u;//焦距转成毫米单位

				myp.z = (f_mm * s * d) / (f_mm * s + (disp - 1024) * u * d / 16);
				myp.x = -(cx / 2 - j) * (2.0 / f) * myp.z;
				myp.y = -(cy / 2 - i) * (2.0 / f) * myp.z;


#ifdef COLOR_PLY /* 生成color点云 */
				Eigen::Vector4f vet(myp.x, myp.y, myp.z, 1);
				Eigen::MatrixXf calMat(4, 4);

				//rgb2ir
				/*calMat << 0.9984, 0.0372, -0.0432, -1.37960,
					-0.0363, 0.9991, 0.0199, -8.65535,
					0.0439, -0.0183, 0.9989, 0.91501,
					0, 0, 0, 1;*/

					//ir2rgb
				calMat << 0.99912514997018408991, -0.03243837631654813980, 0.02639481841201186257, 1.86960536417045175561,
					0.03247398698323455957, 0.99947215618113360858, -0.00092151399789793692, -0.42521844477544912255,
					-0.02635099365241819896, 0.00177785280088593452, 0.99965117134575931512, 0.15115274364020359066,
					0, 0, 0, 1;

				//单位矩阵
				/*calMat << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;*/

				Eigen::Vector4f pos = calMat * vet;
				myp.x = pos[0];
				myp.y = pos[1];
				myp.z = pos[2];

				/* for color */
				float ux = fx_color * myp.x / myp.z + cx_color;
				float uy = fy_color * myp.y / myp.z + cy_color;

				int x = static_cast<int>(ux + 0.5f);
				int y = static_cast<int>(uy + 0.5f);

				if (x < 0 || x >= 1080 || y < 0 || y >= 1920)
				{
					//cout << "x < 0 || x >= 1280 || y < 0 || y >= 720" << endl;
					myp.r = 0; myp.g = 0; myp.b = 0;
				}
				else
				{
					myp.b = imRGB.at<Vec3b>(y, x)[0];
					myp.g = imRGB.at<Vec3b>(y, x)[1];
					myp.r = imRGB.at<Vec3b>(y, x)[2];
				}
#endif
				points.push_back(myp);
			}
		}
	}

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
#ifdef COLOR_PLY /* 生成color点云 */
	ply << "property uchar red" << "\n";
	ply << "property uchar green" << "\n";
	ply << "property uchar blue" << "\n";
#endif
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
#ifdef COLOR_PLY /* 生成color点云 */
		ply << points[i].x << " " << points[i].y << " " << points[i].z << " " <<
			(int)points[i].r << " " << (int)points[i].g << " " << (int)points[i].b << "\n";
#else
		ply << points[i].x << "     " << points[i].y << "      " << points[i].z << "\n";
#endif
	}
	ply.close();
}

void outputDepthImg(unsigned short *inputImg)
{
	//int dep_height = 360;
	//int dep_width = 640;

	//Mat depth_frame(dep_height, dep_width, CV_16UC1, inputImg);
	/*Mat depth_frame_medianBlur(depth_frame.size(), CV_16UC1, cv::Scalar(0, 0, 0));
	cv::medianBlur(depth_frame, depth_frame_medianBlur, 15);

	Mat filledDepth = depth_frame_medianBlur.clone();
	Mat filledDepthf; filledDepth.convertTo(filledDepthf, CV_32F);
	Mat filteredDepthf = Mat::ones(depth_frame_medianBlur.size(), CV_32F);
	bilateralFilter(filledDepthf, filteredDepthf, 10, 10, 10);
	filteredDepthf.convertTo(filledDepth, CV_16U);*/

	//imwrite("C:\\Users\\Administrator\\Desktop\\D11-good\\1.png", depth_frame);

	cv::Mat rgb_frame = cv::imread("C:\\Users\\Administrator\\Desktop\\02\\1\\rgb_1.jpg");
	cv::Mat dep_frame = cv::imread("C:\\Users\\Administrator\\Desktop\\02\\1\\dep_1.png",-1);

	depth2PointCloud(dep_frame, rgb_frame, "C:\\Users\\Administrator\\Desktop\\02\\1\\1.ply");
}

int main(int argc, char** argv)
{
	//const int ARR_SIZE = 8;
	//const int SIZE_PER_CHANNEL = 2;

	//unsigned short zd_ori;
	std::vector<unsigned short> zd_table;
	//const char* pFileName_zd = "C:\\Users\\Administrator\\Desktop\\D11-good\\depth_1.raw";

	//ifstream infile_zd(pFileName_zd, ios::binary);
	//if (!infile_zd)
	//{
	//	cout << "Read file error" << endl;
	//	return -1;
	//}

	//zd_table.clear();

	//while (!infile_zd.eof())
	//{
	//	for (int i = 0; i < ARR_SIZE; ++i)
	//	{
	//		infile_zd.read((char*)&zd_ori, SIZE_PER_CHANNEL);
	//		//unsigned short temp = (zd_ori >> 8) + (zd_ori << 8);
	//		//zd_table.push_back(temp);
	//		zd_table.push_back(zd_ori);
	//		//cout << temp << endl;
	//	}
	//}
	//infile_zd.close();
	//zd_table.resize(640 * 360);

	/*for (int i = zd_table.size() - 16; i < zd_table.size(); i++)
	{
		cout << zd_table[i] << endl;
	}*/


	//unsigned short yuv_buf;
	//std::vector<unsigned short> depth_buf;
	//const char* pFileName_yuvImg = "C:\\Users\\Administrator\\Desktop\\D11-good\\Depth_sn19032_52054750.yuv";

	//ifstream infile_yuvImg(pFileName_yuvImg, ios::binary);
	//if (!infile_yuvImg)
	//{
	//	cout << "Read file error" << endl;
	//	return -1;
	//}

	//depth_buf.clear();

	//while (!infile_yuvImg.eof())
	//{
	//	for (int i = 0; i < ARR_SIZE; ++i)
	//	{
	//		infile_yuvImg.read((char*)&yuv_buf, SIZE_PER_CHANNEL);
	//		yuv_buf &= 0x07FF;
	//		depth_buf.push_back(zd_table[yuv_buf]);
	//		//cout << yuv_buf << endl;
	//	}
	//}

	//infile_yuvImg.close();
	//depth_buf.resize(1920*1080);

	/*for (int i = depth_buf.size() - 16; i < depth_buf.size(); i++)
	{
		cout << depth_buf[i] << endl;
	}*/

	outputDepthImg(zd_table.data());

	return 0;
}

#elif defined TEST_11
#pragma comment( lib,"winmm.lib" )

#include <Eigen\Eigen>
#include <opencv2/opencv.hpp>

#define COLOR_PLY

using namespace std;
using namespace cv;

void depth2PointCloud(cv::Mat imDepth, cv::Mat imRGB, std::string filePath, int count)
{
	cv::Mat depth = imDepth.clone();

	//a07
	//float fx = 855.90017;//RK intrix depth
	//float fy = 855.83248;//RK intrix depth
	//float cx = 644.66965;//RK intrix depth
	//float cy = 361.97021;//RK intrix depth

	//float fx_color = 1230.72673;//RK intrix rgb
	//float fy_color = 1230.31252;//RK intrix rgb
	//float cx_color = 647.31065;//RK intrix rgb
	//float cy_color = 361.17036;//RK intrix rgb

	//a08
	float fx = 850.02772;//RK intrix depth
	float fy = 849.11788;//RK intrix depth
	float cx = 646.30737;//RK intrix depth
	float cy = 361.94682;//RK intrix depth

	float fx_color = 1208.74278;//RK intrix rgb
	float fy_color = 1208.26830;//RK intrix rgb
	float cx_color = 638.86837;//RK intrix rgb
	float cy_color = 361.86958;//RK intrix rgb

	//a12
	//float fx = 849.03956;//RK intrix depth
	//float fy = 848.52405;//RK intrix depth
	//float cx = 648.30611;//RK intrix depth
	//float cy = 352.75012;//RK intrix depth

	//float fx_color = 1219.44408;//RK intrix rgb
	//float fy_color = 1219.24295;//RK intrix rgb
	//float cx_color = 655.53307;//RK intrix rgb
	//float cy_color = 361.97611;//RK intrix rgb

	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {

			unsigned short disp = imDepth.at<unsigned short>(i, j);
			if (disp != 0) {

				float f = (fx + fy) / 2;
				point_st myp;

				myp.z = (90 * f) / (0.15 * f + (disp - 1024) * 0.1125);
				myp.x = -(cx / 2 - j) * (2.0 / f) * myp.z;
				myp.y = -(cy / 2 - i) * (2.0 / f) * myp.z;

				depth.at<unsigned short>(i, j) = (unsigned short)myp.z;

#ifdef COLOR_PLY /* 生成color点云 */
				Eigen::Vector4f vet(myp.x, myp.y, myp.z, 1);
				Eigen::MatrixXf calMat(4, 4);

				//ir2rgb a07
				/*calMat << 0.9997, -0.0102, 0.0210, -1.86569,
					0.0103, 0.9999, -0.0038, 9.99175,
					-0.0209, 0.0040, 0.9998, 0.73901,
					0, 0, 0, 1;*/

					//ir2rgb a08
				calMat << 0.9945, 0.0042, 0.1042, 0.32603,
					-0.0044, 1.0000, 0.0016, 10.31960,
					-0.1042, -0.0021, 0.9946, 0.73626,
					0, 0, 0, 1;

				//ir2rgb a12
				/*calMat << 0.9999, 0.0073, -0.0084, -0.05556,
					-0.0075, 0.9998, -0.0182, 8.92511,
					0.0082, 0.0182, 0.9998, 0.87518,
					0, 0, 0, 1;*/

					//单位矩阵
					/*calMat << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;*/

				Eigen::Vector4f pos = calMat * vet;
				myp.x = pos[0];
				myp.y = pos[1];
				myp.z = pos[2];

				/* for color */
				float ux = fx_color * myp.x / myp.z + cx_color;
				float uy = fy_color * myp.y / myp.z + cy_color;

				int x = static_cast<int>(ux + 0.5f);
				int y = static_cast<int>(uy + 0.5f);

				if (x < 0 || x >= 1280 || y < 0 || y >= 720)
				{
					//cout << "x < 0 || x >= 1280 || y < 0 || y >= 720" << endl;
					myp.r = 0; myp.g = 0; myp.b = 0;
				}
				else
				{
					myp.b = imRGB.at<Vec3b>(y, x)[0];
					myp.g = imRGB.at<Vec3b>(y, x)[1];
					myp.r = imRGB.at<Vec3b>(y, x)[2];
				}
#endif
				points.push_back(myp);
			}
		}
	}

	std::stringstream file_depth;
	file_depth << "C:\\Users\\Administrator\\Desktop\\D11-good\\A08\\depth\\depth_" << count << ".png";
	cv::imwrite(file_depth.str(), depth);

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
#ifdef COLOR_PLY /* 生成color点云 */
	ply << "property uchar red" << "\n";
	ply << "property uchar green" << "\n";
	ply << "property uchar blue" << "\n";
#endif
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
#ifdef COLOR_PLY /* 生成color点云 */
		ply << points[i].x << " " << points[i].y << " " << points[i].z << " " <<
			(int)points[i].r << " " << (int)points[i].g << " " << (int)points[i].b << "\n";
#else
		ply << points[i].x << "     " << points[i].y << "      " << points[i].z << "\n";
#endif
	}
	ply.close();
}

void outputDepthImg(unsigned short *inputImg, int count)
{
	int dep_height = 360;
	int dep_width = 640;

	Mat depth_frame(dep_height, dep_width, CV_16UC1, inputImg);

	std::stringstream file_rgb;
	file_rgb << "C:\\Users\\Administrator\\Desktop\\D11-good\\A08\\rgb\\rgb_" << count << ".jpg";
	cv::Mat rgb_frame = cv::imread(file_rgb.str());

	std::stringstream file_ply;
	file_ply << "C:\\Users\\Administrator\\Desktop\\D11-good\\A08\\ply\\" << count << ".ply";
	depth2PointCloud(depth_frame, rgb_frame, file_ply.str(), count);
}


int main(int argc, char** argv)
{
	const int ARR_SIZE = 8;
	const int SIZE_PER_CHANNEL = 2;

	unsigned short depth_value;
	std::vector<unsigned short> depth_value_table;

	std::cout << "start:" << timeGetTime() << std::endl;

	for (int i = 0; i < 256; i++)
	{
		std::stringstream file_raw;
		file_raw << "C:\\Users\\Administrator\\Desktop\\D11-good\\A08\\raw\\raw_" << i << ".raw";

		std::ifstream depth_file(file_raw.str().c_str(), ios::binary);
		if (!depth_file)
		{
			cout << "Read file error" << endl;
			return -1;
		}

		depth_value_table.clear();
		depth_value = 0;

		while (!depth_file.eof())
		{
			for (int i = 0; i < ARR_SIZE; ++i)
			{
				depth_file.read((char*)&depth_value, SIZE_PER_CHANNEL);
				depth_value_table.push_back(depth_value);
			}
		}
		depth_file.close();
		depth_value_table.resize(640 * 360);

		outputDepthImg(depth_value_table.data(), i);
	}

	std::cout << "end:" << timeGetTime() << std::endl;

	return 0;
}
#elif defined TEST_12

using namespace cv;

int main(int argc, char** argv)
{
	Mat depth_frame = imread("C:\\Users\\Administrator\\Desktop\\depth\\23.png", -1);

	memset(depth_frame.data + 480 * 500 * 2, 0x0, 480 * 140 * 2);

	for (int i = 0; i < depth_frame.rows; i++)
	{
		for (int j = 0; j < depth_frame.cols; j++)
		{
			unsigned short z = depth_frame.at<unsigned short>(i, j);
			if (z > 50000)
				depth_frame.at<unsigned short>(i, j) = 0;
		}
	}

	imwrite("C:\\Users\\Administrator\\Desktop\\depth\\23_1.png", depth_frame);

	return 0;
}
#elif defined TEST_13

using namespace cv;

int main(int argv, char **args)

{
	Mat img = imread("C:\\Users\\Administrator\\Desktop\\1\\11.png", -1);

	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (img.at<unsigned short>(i, j) > 0)
				img.at<unsigned short>(i, j) = 50000;
		}
	}

	imwrite("C:\\Users\\Administrator\\Desktop\\1\\12.png", img);

	return 0;

}
#elif defined TEST_14
#include<opencv2/objdetect/objdetect.hpp>  
#include<opencv2/highgui/highgui.hpp>  
#include<opencv2/imgproc/imgproc.hpp>  
#include <iostream> 

using namespace cv;
using namespace std;
CascadeClassifier faceCascade;
CascadeClassifier faceCascade_profileface;
int main()
{
	faceCascade.load("E:/3rd_lib/opencv3.3.0/build/etc/haarcascades/haarcascade_frontalface_alt2.xml");
	faceCascade_profileface.load("E:/3rd_lib/opencv3.3.0/build/etc/haarcascades/haarcascade_profileface.xml");
	VideoCapture capture;
	capture.open(1);    // 打开摄像头
	//capture.open("video.avi");    // 打开视频	
	if (!capture.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	Mat img, imgGray;
	vector<Rect> faces;
	while (1)
	{
		capture >> img;    // 读取图像至img		

		cv::resize(img, img, cv::Size(img.cols / 2, img.rows / 2));
		cout << img.rows << endl;
		cout << img.cols << endl;
		if (img.empty())
		{
			continue;
		}
		if (img.channels() == 3)
		{
			cvtColor(img, imgGray, CV_RGB2GRAY);
		}
		else
		{
			imgGray = img;
		}
		faceCascade.detectMultiScale(imgGray, faces, 1.2, 6, 0, Size(0, 0));    // 检测正脸				
		if (faces.size() > 0)
		{
			for (int i = 0; i < faces.size(); i++)
			{
				rectangle(img, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0, 255, 0), 1, 8);
			}
		}
		else
		{
			faceCascade_profileface.detectMultiScale(imgGray, faces, 1.2, 6, 0, Size(0, 0));    // 检测侧脸	
			if (faces.size() > 0)
			{
				for (int i = 0; i < faces.size(); i++)
				{
					rectangle(img, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0, 255, 0), 1, 8);
				}
			}
		}
		imshow("CamerFace", img);      // 显示				
		if (waitKey(1) > 0)		// delay ms 等待按键退出		
		{
			break;
		}
	}
	return 0;
}

#elif defined TEST_15
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace dlib;
using namespace std;

int main(int argc, char** argv)
{
	VideoCapture capture;
	capture.open(1);

	if (!capture.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	dlib::frontal_face_detector detector = get_frontal_face_detector();

	while (1)
	{
		Mat opencvImage, opencvImageGray;

		capture >> opencvImage;

		cv::resize(opencvImage, opencvImage, cv::Size(opencvImage.cols / 2, opencvImage.rows / 2));

		if (!opencvImage.data)
		{
			printf("No image data \n");
			return -1;
		}
		else if (opencvImage.channels() == 3)
		{
			cvtColor(opencvImage, opencvImageGray, CV_RGB2GRAY);
		}
		else
		{
			opencvImageGray = opencvImage;
		}

		//-------------- step 1. load the input two images --------------------------------------------       
		//shape_predictor sp;
		//deserialize("E:\\3rd_lib\\dlib-19.9\\shape_predictor_68_face_landmarks.dat") >> sp;
		dlib::array2d<unsigned char> dlibImage;
		dlib::assign_image(dlibImage, dlib::cv_image<unsigned char>(opencvImageGray));
		std::vector<dlib::rectangle> dets = detector(dlibImage);

		if (dets.size() > 0)
		{
			for (int i = 0; i < dets.size(); i++)
			{
				cv::rectangle(opencvImage, Point(dets[i].left(), dets[i].top()), Point(dets[i].right(), dets[i].bottom()), Scalar(0, 255, 0), 1, 8);
			}
		}

		imshow("CamerFace", opencvImage);      // 显示	

		if (waitKey(1) > 0)		// delay ms 等待按键退出		
		{
			break;
		}
	}
	return 0;
}
#elif defined TEST_16
#include <iostream>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h> 
using namespace std;
using namespace cv;

//Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
//Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };
//double D[5] = {0.0, 0.0, 0.0, 0.0, 0.0 };
//double K[9] = { 5.9460101318359375e+02 * 2, 0.0, 2.3929042053222656e+02, 0.0, 5.9460101318359375e+02 * 2, 3.1977670288085938e+02, 0.0, 0.0, 1.0 };

//double K[9] = { 116.5308391993466671e+002, 0.0, 113.1950000000000000e+002, 0.0, 116.5308391993466671e+002, 112.3950000000000000e+002, 0.0, 0.0, 1.0 };

int main()
{
	//open cam
	cv::VideoCapture cap(1);
	//VideoCapture cap;
	//cap.open(1);
	if (!cap.isOpened())
	{
		std::cout << "Unable to connect to camera" << std::endl;
		return EXIT_FAILURE;
	}
	//Load face detection and pose estimation models (dlib).
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor predictor;
	dlib::deserialize("E:\\3rd_lib\\dlib-19.9\\shape_predictor_68_face_landmarks.dat") >> predictor;

	//fill in cam intrinsics and distortion coefficients
	cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
	cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

	//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	std::vector<cv::Point3d> object_pts;
	object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
	object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
	object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
	object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
	object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
	object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
	object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
	object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
	object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
	object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
	object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
	object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
	object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
	object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner

																		 //2D ref points(image coordinates), referenced from detected facial feature
	std::vector<cv::Point2d> image_pts;

	//result
	cv::Mat rotation_vec;                           //3 x 1
	cv::Mat rotation_mat;                           //3 x 3 R
	cv::Mat translation_vec;                        //3 x 1 T
	cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
	cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

	//reproject 3D points world coordinate axis to verify result pose
	std::vector<cv::Point3d> reprojectsrc;
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

	//reprojected 2D points
	std::vector<cv::Point2d> reprojectdst;
	reprojectdst.resize(8);

	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

	//text on screen
	std::ostringstream outtext;

	//main loop
	while (1)
	{
		// Grab a frame
		cv::Mat temp;
		cap >> temp;
		dlib::cv_image<dlib::bgr_pixel> cimg(temp);


		// Detect faces
		long t1 = GetTickCount();
		std::vector<dlib::rectangle> faces = detector(cimg);

		// Find the pose of each face 
		if (faces.size() > 0)
		{
			//track features
			dlib::full_object_detection shape = predictor(cimg, faces[0]);
			long t2 = GetTickCount();
			cout << " detected time: " << t2 - t1 << endl;
			//draw features
			for (unsigned int i = 0; i < 68; ++i)
			{
				cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
			}


			image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
			image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
			image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
			image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
			image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
			image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
			image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
			image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
			image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
			image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
			image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
			image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
			image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
			image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner

																					  //calc pose
			cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);
			//cv::solvePnPRansac(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);
			//reproject
			cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);

			//draw axis
			cv::line(temp, reprojectdst[0], reprojectdst[1], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[1], reprojectdst[2], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[2], reprojectdst[3], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[3], reprojectdst[0], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[4], reprojectdst[5], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[5], reprojectdst[6], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[6], reprojectdst[7], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[7], reprojectdst[4], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[0], reprojectdst[4], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[1], reprojectdst[5], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[2], reprojectdst[6], cv::Scalar(0, 0, 255));
			cv::line(temp, reprojectdst[3], reprojectdst[7], cv::Scalar(0, 0, 255));

			//calc euler angle
			cv::Rodrigues(rotation_vec, rotation_mat);
			cv::hconcat(rotation_mat, translation_vec, pose_mat);
			cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

			std::cout << "xyz" << euler_angle << std::endl;
			//show angle result
			outtext << "X: " << std::setprecision(3) << euler_angle.at<double>(0);
			cv::putText(temp, outtext.str(), cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255));
			outtext.str("");
			outtext << "Y: " << std::setprecision(3) << euler_angle.at<double>(1);
			cv::putText(temp, outtext.str(), cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255));
			outtext.str("");
			outtext << "Z: " << std::setprecision(3) << euler_angle.at<double>(2);
			cv::putText(temp, outtext.str(), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255));
			outtext.str("");

			image_pts.clear();
		}

		//press esc to end
		cv::imshow("demo", temp);
		unsigned char key = cv::waitKey(1);
		if (key == 27)
		{
			break;
		}
	}

	return 0;
}

bool blurDetect(cv::Mat srcImage)
{

	cv::Mat gray1;
	if (srcImage.channels() != 1)
	{
		//进行灰度化
		cvtColor(srcImage, gray1, CV_RGB2GRAY);
	}
	else
	{
		gray1 = srcImage.clone();
	}
	cv::Mat tmp_m1, tmp_sd1;	//用来存储均值和方差
	double m1 = 0, sd1 = 0;
	//使用3x3的Laplacian算子卷积滤波
	Laplacian(gray1, gray1, CV_16S, 3);
	//归到0~255
	convertScaleAbs(gray1, gray1);
	//计算均值和方差
	meanStdDev(gray1, tmp_m1, tmp_sd1);
	m1 = tmp_m1.at<double>(0, 0);		//均值
	sd1 = tmp_sd1.at<double>(0, 0);		//标准差


	if (sd1*sd1 < 400)
	{
		cout << "原图像是模糊图像" << endl;
		return 0;
	}
	else
	{
		cout << "原图像是清晰图像" << endl;
		return 1;
	}
}
#elif defined TEST_17
#include <iostream>
#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/opencv.h>

using namespace std;
using namespace dlib;
using namespace cv;
// ----------------------------------------------------------------------------------------

template <long num_filters, typename SUBNET> using con5d = con<num_filters, 5, 5, 2, 2, SUBNET>;
template <long num_filters, typename SUBNET> using con5 = con<num_filters, 5, 5, 1, 1, SUBNET>;

template <typename SUBNET> using downsampler = relu<affine<con5d<32, relu<affine<con5d<32, relu<affine<con5d<16, SUBNET>>>>>>>>>;
template <typename SUBNET> using rcon5 = relu<affine<con5<45, SUBNET>>>;

using net_type = loss_mmod<con<1, 9, 9, 1, 1, rcon5<rcon5<rcon5<downsampler<input_rgb_image_pyramid<pyramid_down<6>>>>>>>>;

// ----------------------------------------------------------------------------------------


int main() try
{
	VideoCapture capture;
	capture.open(1);

	if (!capture.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	net_type net;
	deserialize("E:\\3rd_lib\\dlib-19.9\\mmod_human_face_detector.dat") >> net;

	while (1) {
		Mat opencvImage;
		capture >> opencvImage;

		matrix<rgb_pixel> img1;
		dlib::assign_image(img1, dlib::cv_image<rgb_pixel>(opencvImage));

		auto dets = net(img1);

		if (dets.size() > 0)
		{
			for (int i = 0; i < dets.size(); i++)
			{
				cv::rectangle(opencvImage, Point(dets[i].rect.left(), dets[i].rect.top()), Point(dets[i].rect.right(), dets[i].rect.bottom()), Scalar(0, 255, 0), 1, 8);
			}
		}

		imshow("CamerFace", opencvImage);      // 显示

		if (waitKey(1) > 0)
			break;
	}
}
catch (std::exception& e)
{
	cout << e.what() << endl;
}
#elif defined TEST_18
using namespace std;
using namespace cv;

int main()
{
	VideoCapture capture;
	capture.open(1);

	if (!capture.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	while (1)
	{
		Mat a;
		capture >> a;
		imshow("a", a);/*(height:480, width:640)*/

		/*将a放置到b中的b_roi位置*/
		Mat b = Mat::zeros(720, 1280, CV_8UC3);
		Mat b_roi = b(Rect(0/*x*/, 50/*y*/, a.cols/*x_offset*/, a.rows/*y_offset*/));
		a.copyTo(b_roi);
		imshow("b", b);

		/*将c中(y0,y1,x0,x1)位置的图截取出来*/
		Mat c = a.clone();
		Mat c_roi = c(Range(0, a.rows)/*(y_start,y_end)*/, Range(0, a.cols)/*(x_start,x_end)*/);
		imshow("c_roi", c_roi);

		/*初始化一张纯蓝色的480x640的图片*/
		Mat d(480, 640, CV_8UC3, Scalar(255, 0, 0)/*b,g,r*/);
		imshow("d", d);

		/*初始化一张纯黑色的480x640的图片*/
		Mat e(480, 640, CV_8UC3);
		imshow("e", e);

		/*初始化一张纯蓝色的640x480的图片*/
		Size f_size(480, 640)/*width,height*/;
		Mat f(f_size, CV_8UC3, Scalar(255, 0, 0));
		imshow("f", f);

		waitKey(1);
	}
}
#elif defined TEST_19

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

void distCoeffs_fixed(Mat intrinsic_matrix, Mat distortion_coeffs, Mat image)
{
	Size image_size = image.size();

	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);

	Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));

	newCameraMatrix = getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0);

	cout << "矫正后的新相机内参:" << newCameraMatrix << endl;

	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, newCameraMatrix, image_size, CV_32FC1, mapx, mapy);

	Mat t = image.clone();

	cv::remap(image, t, mapx, mapy, INTER_LINEAR);

	imwrite("E:\\Project\\demo\\dataset\\TEST_19\\rgb_0_jiaozheng.jpg", t);
}

void startCalibration(int w, int h, int img_count, Size board_size, Size2f square_size, vector<vector<Point2f>> image_points_seq)
{
	if (img_count < 3)
	{
		cout << "Error：标定图片少于3张" << endl;
		return;
	}

	cout << "标定初始化。。。" << endl;

	/* 图像尺寸 */
	Size image_size;
	/* 标定板上角点的三维坐标 */
	vector<vector<Point3f>> object_points;
	/* 摄像机内参数矩阵 */
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	/* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	/* 每幅图像的旋转向量 */
	vector<Mat> tvecsMat;
	/* 每幅图像的平移向量 */
	vector<Mat> rvecsMat;

	image_size.width = w;
	image_size.height = h;

	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t < img_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	
	cout << "开始标定………………" << endl;
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	
	bool ok = checkRange(cameraMatrix);
	if (ok)
	{
		cout << "标定完成！！" << endl;
	}
	else
	{
		cout << "标定失败！！" << endl;
		return;
	}

	cout << "cameraMatrix:" << cameraMatrix << endl;
	cout << "distCoeffs:" << distCoeffs << endl;

	/* 将内参写入文件 */
	cout << "将内参写入文件………………" << endl;
	std::stringstream cameraMatrixFile;
	cameraMatrixFile << "E:\\Project\\demo\\dataset\\TEST_19\\intrinsics.txt";
	std::ofstream intrinsics(cameraMatrixFile.str());

	intrinsics << "fx:" << setprecision(16) << cameraMatrix.at<double>(0, 0) << "\n";
	intrinsics << "fy:" << setprecision(16) << cameraMatrix.at<double>(1, 1) << "\n";
	intrinsics << "cx:" << setprecision(16) << cameraMatrix.at<double>(0, 2) << "\n";
	intrinsics << "cy:" << setprecision(16) << cameraMatrix.at<double>(1, 2) << "\n";

	intrinsics << "distCoeffs[0]:" << setprecision(16) << distCoeffs.at<double>(0) << "\n";
	intrinsics << "distCoeffs[1]:" << setprecision(16) << distCoeffs.at<double>(1) << "\n";
	intrinsics << "distCoeffs[2]:" << setprecision(16) << distCoeffs.at<double>(2) << "\n";
	intrinsics << "distCoeffs[3]:" << setprecision(16) << distCoeffs.at<double>(3) << "\n";
	intrinsics << "distCoeffs[4]:" << setprecision(16) << distCoeffs.at<double>(4) << "\n";

	intrinsics.close();

	cout << "开始评价标定结果………………\n";	
	/* 所有图像的平均误差的总和 */
	double total_err = 0.0;
	/* 每幅图像的平均误差 */
	double err = 0.0;
	/* 保存重新计算得到的投影点 */
	vector<Point2f> image_points2;	
	cout << "\t每幅图像的标定误差：\n";	
	
	for (i = 0; i<img_count; i++)
	{ 
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		vector<Point3f> tempPointSet = object_points[i];		
		/* 计算新的投影点和旧的投影点之间的误差*/
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);		
				
		vector<Point2f> tempImagePoint = image_points_seq[i];		
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);		
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);		
		for (int j = 0; j < tempImagePoint.size(); j++) 
		{ 
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);			
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}		
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);		
		total_err += (err /= (board_size.height * board_size.width));
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;   		
	}   	
	std::cout << "总体平均误差：" << total_err / img_count << "像素" << endl;   	
	std::cout << "评价完成！" << endl;


	std::cout << "开始矫正！" << endl;
	Mat distCoeffs_img = imread("E:\\Project\\demo\\dataset\\TEST_19\\rgb_0.jpg");
	distCoeffs_fixed(cameraMatrix, distCoeffs, distCoeffs_img);
	std::cout << "矫正完成！" << endl;
}

#define RGB_CALI
//#define IR_CALI

void main()
{
	int inited = 0;
	int count = 0;

#ifdef RGB_CALI
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
#endif
#ifdef IR_CALI
	unsigned char* ir_image0 = new unsigned char[RGB_W * RGB_H];
#endif

#ifdef RGB_CALI
	/* 标定板上每行、列的角点数 */
	Size board_size_rgb = Size(9, 6);
	/* 标定板方格尺寸 */
	Size2f square_size(27.0, 27.0);
	/* 缓存每幅图像上检测到的角点 */
	vector<Point2f> corner_point_rgb;
	/* 总角点 */
	vector<vector<Point2f>> corner_point_rgb_total;
#endif

#ifdef IR_CALI
	/* 标定板上每行、列的角点数 */
	Size board_size_ir = Size(9, 6);
	/* 标定板方格尺寸 */
	Size2f square_size(27.0, 27.0);
	/* 缓存每幅图像上检测到的角点 */
	vector<Point2f> corner_point_ir;
	/* 总角点 */
	vector<vector<Point2f>> corner_point_ir_total;
#endif

	MyCam.StartAllCam();

	MyCam.setLaserOff(true);

	for (;;)
	{
#ifdef RGB_CALI
		MyCam.grabFrame(0, rgb_image0);
		Mat rgbMat(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		cvtColor(rgbMat, rgbMat, CV_RGB2BGR);
		imshow("rgbMat", rgbMat);
#endif

#ifdef IR_CALI
		MyCam.infraredFrame(0, ir_image0);
		Mat irMat(RGB_H, RGB_W, CV_8UC1, ir_image0);
		imshow("irMat", irMat);
#endif
		if (!inited) {
			cout << "拍照请输入c键; 退出请输入esc键;" << endl;
			inited = 1;
		}

		char c = waitKey(1);
		if (c == 27) {
			cout << "数据保存完成!!" << endl;
			break;
		}
		switch (c)
		{
		case 'c':
#ifdef RGB_CALI
			corner_point_rgb.clear();
			if (0 == findChessboardCorners(rgbMat, board_size_rgb, corner_point_rgb))
			{
				cout << "rgbMat 检测角点失败！！" << endl;
				break;
			}
			else
			{
				Mat rgbMatGray;
				cvtColor(rgbMat, rgbMatGray, CV_BGR2GRAY);
				/* 亚像素精确化 */
				find4QuadCornerSubpix(rgbMatGray, corner_point_rgb, Size(5, 5)); //对粗提取的角点进行精确化
				corner_point_rgb_total.push_back(corner_point_rgb);  //保存亚像素角点
				/* 在图像上显示角点位置 */
				//drawChessboardCorners(rgbMatGray, board_size_rgb, corner_point_rgb, true); //用于在图片中标记角点
				//imshow("RGB Camera Calibration",rgbMatGray);//显示图片
			}
#endif

#ifdef IR_CALI
			corner_point_ir.clear();
			if (0 == findChessboardCorners(irMat, board_size_ir, corner_point_ir))
			{
				cout << "irMat 检测角点失败！！" << endl;
				break;
			}
			else
			{
				Mat irMatGray = irMat.clone();
				/* 亚像素精确化 */
				find4QuadCornerSubpix(irMatGray, corner_point_ir, Size(5, 5)); //对粗提取的角点进行精确化
				corner_point_ir_total.push_back(corner_point_ir);  //保存亚像素角点
				/* 在图像上显示角点位置 */
				//drawChessboardCorners(irMatGray, board_size_ir, corner_point_ir, true); //用于在图片中标记角点
				//imshow("IR Camera Calibration", irMatGray);//显示图片
		}
#endif
#ifdef RGB_CALI
			/*rgb cali file*/
			std::stringstream rgbfile;
			rgbfile << "E:\\Project\\demo\\dataset\\TEST_19\\rgb_" << count << ".jpg";
			imwrite(rgbfile.str(), rgbMat);
#endif

#ifdef IR_CALI
			/*ir cali file*/
			std::stringstream irfile;
			irfile << "E:\\Project\\demo\\dataset\\TEST_19\\ir_" << count << ".jpg";
			imwrite(irfile.str(), irMat);
#endif
			count++;

			cout << "完成第 " << count << " 次拍照" << endl;

			break;
		}
	}

#ifdef RGB_CALI
	startCalibration(RGB_W, RGB_H, count, board_size_rgb, square_size, corner_point_rgb_total);
#endif

#ifdef IR_CALI
	startCalibration(RGB_W, RGB_H, count, board_size_ir, square_size, corner_point_ir_total);
#endif

#ifdef RGB_CALI
	delete[]rgb_image0;
#endif
#ifdef IR_CALI
	delete[]ir_image0;
#endif
	return;
}
#elif defined TEST_19_1

//#define TEST_19_1_RGB

using namespace std;
using namespace cv;

void startCalibration(int w, int h, int img_count, Size board_size, Size2f square_size, vector<vector<Point2f>> image_points_seq)
{
	if (img_count < 3)
	{
		cout << "Error：标定图片少于3张" << endl;
		return;
	}

	cout << "标定初始化。。。" << endl;

	/* 图像尺寸 */
	Size image_size;
	/* 标定板上角点的三维坐标 */
	vector<vector<Point3f>> object_points;
	/* 摄像机内参数矩阵 */
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	/* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	/* 每幅图像的旋转向量 */
	vector<Mat> tvecsMat;
	/* 每幅图像的平移向量 */
	vector<Mat> rvecsMat;

	image_size.width = w;
	image_size.height = h;

	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t < img_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}

	cout << "开始标定………………" << endl;
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	bool ok = checkRange(cameraMatrix);
	if (ok)
	{
		cout << "标定完成！！" << endl;
	}
	else
	{
		cout << "标定失败！！" << endl;
		return;
	}

	cout << "cameraMatrix:" << cameraMatrix << endl;
	cout << "distCoeffs:" << distCoeffs << endl;
#ifdef TEST_19_1_RGB
	FileStorage fs("C:\\Users\\Administrator\\Desktop\\02\\intrix_rgb.yml", FileStorage::WRITE);
	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	fs.release();
#else
	FileStorage fs("C:\\Users\\Administrator\\Desktop\\02\\intrix_ir.yml", FileStorage::WRITE);
	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	fs.release();
#endif
	cout << "开始评价标定结果………………\n";
	/* 所有图像的平均误差的总和 */
	double total_err = 0.0;
	/* 每幅图像的平均误差 */
	double err = 0.0;
	/* 保存重新计算得到的投影点 */
	vector<Point2f> image_points2;
	cout << "\t每幅图像的标定误差：\n";

	for (i = 0; i<img_count; i++)
	{
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		vector<Point3f> tempPointSet = object_points[i];
		/* 计算新的投影点和旧的投影点之间的误差*/
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);

		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += (err /= (board_size.height * board_size.width));
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	std::cout << "总体平均误差：" << total_err / img_count << "像素" << endl;
	std::cout << "评价完成！" << endl;


	//std::cout << "开始矫正！" << endl;
	//Mat distCoeffs_img = imread("C:\\Users\\Administrator\\Desktop\\02\\rgb_0.jpg");
	//distCoeffs_fixed(cameraMatrix, distCoeffs, distCoeffs_img);
	//std::cout << "矫正完成！" << endl;
}

void main()
{
	int count = 1;
	const int pic_number = 19;

#ifdef TEST_19_1_RGB
	const int pic_width = 1080;
	const int pic_height = 1920;
#else
	const int pic_width = 720;
	const int pic_height = 1280;
#endif
	/* 标定板上每行、列的角点数 */
	Size board_size_rgb = Size(9, 6);
	/* 标定板方格尺寸 */
	Size2f square_size(22.0, 22.0);
	/* 缓存每幅图像上检测到的角点 */
	vector<Point2f> corner_point_rgb;
	/* 总角点 */
	vector<vector<Point2f>> corner_point_rgb_total;

	for (;;)
	{
		corner_point_rgb.clear();
		std::stringstream rgbfile;
#ifdef TEST_19_1_RGB
		rgbfile << "C:\\Users\\Administrator\\Desktop\\02\\rgb_" << count << ".jpg";
#else
		rgbfile << "C:\\Users\\Administrator\\Desktop\\02\\mono_" << count << ".png";
#endif
		Mat rgbMat = imread(rgbfile.str());
		//Mat rgbMat;flip(rgbMat1, rgbMat, 1);
		if (0 == findChessboardCorners(rgbMat, board_size_rgb, corner_point_rgb))
		{
			cout << "rgbMat 检测角点失败！！" << endl;
			break;
		}
		else
		{
			Mat rgbMatGray;
			cvtColor(rgbMat, rgbMatGray, CV_BGR2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(rgbMatGray, corner_point_rgb, Size(5, 5)); //对粗提取的角点进行精确化
			corner_point_rgb_total.push_back(corner_point_rgb);  //保存亚像素角点
			/* 在图像上显示角点位置 */
			drawChessboardCorners(rgbMatGray, board_size_rgb, corner_point_rgb, true); //用于在图片中标记角点
			imshow("RGB Camera Calibration",rgbMatGray);//显示图片
			waitKey(10);
		}

		if (count >= pic_number)
			break;
		else
			count++;
	}

	startCalibration(pic_width, pic_height, count, board_size_rgb, square_size, corner_point_rgb_total);

	return;
}
#elif defined TEST_20

#include <Eigen\Eigen>

using namespace cv;
using namespace std;

#define COLOR_PLY

void depth2PointCloud(cv::Mat imDepth, cv::Mat imRGB, std::string filePath)
{
	/* 获取rgb和depth内参和畸变参数，畸变参数默认为0 */
	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);

	float fx = list_of_ints[6];
	float fy = list_of_ints[7];
	float cx = list_of_ints[4];
	float cy = list_of_ints[5];

	float fx_color = list_of_ints[2];
	float fy_color = list_of_ints[3];
	float cx_color = list_of_ints[0];
	float cy_color = list_of_ints[1];

	float factor = 10000.0f;

	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {

			unsigned short Zw = imDepth.at<unsigned short>(i, j);
			if (Zw != 0) {
				point_st myp;
				myp.z = (float)(Zw / factor);
				myp.x = (j - cx) * myp.z / fx;
				myp.y = (i - cy) * myp.z / fy;
#ifdef COLOR_PLY /* 生成color点云 */
				Eigen::Vector4f vet(myp.x, myp.y, myp.z, 1);
				Eigen::MatrixXf calMat(4, 4);

				//depth2rgb
				calMat << 0.99999893258385263994, -0.00020954017523547248, 0.00144600279052944079, 0.01485886320820467739,
					0.00022019810355153883, 0.99997278453393245723, -0.00737439517851025406, -0.00005709708398696868,
					-0.00144441820483160225, 0.00737470571403396787, 0.99997176328718451188, 0.00173990029789993146,
					0, 0, 0, 1;

				Eigen::Vector4f pos = calMat * vet;
				myp.x = pos[0];
				myp.y = pos[1];
				myp.z = pos[2];

				/* for color */
				float ux = fx_color * myp.x / myp.z + cx_color;
				float uy = fy_color * myp.y / myp.z + cy_color;

				int x = static_cast<int>(ux + 0.5f);
				int y = static_cast<int>(uy + 0.5f);

				if (x < 0 || x >= RGB_W || y < 0 || y >= RGB_H)
				{
					//cout << "x < 0 || x >= RGB_W || y < 0 || y >= RGB_H" << endl;
					myp.r = 0; myp.g = 0; myp.b = 0;
				}
				else
				{
					myp.b = imRGB.at<Vec3b>(y, x)[0];
					myp.g = imRGB.at<Vec3b>(y, x)[1];
					myp.r = imRGB.at<Vec3b>(y, x)[2];
				}
#endif
				points.push_back(myp);
			}
		}
	}

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
#ifdef COLOR_PLY /* 生成color点云 */
	ply << "property uchar red" << "\n";
	ply << "property uchar green" << "\n";
	ply << "property uchar blue" << "\n";
#endif
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
#ifdef COLOR_PLY /* 生成color点云 */
		ply << points[i].x << " " << points[i].y << " " << points[i].z << " " <<
			(int)points[i].r << " " << (int)points[i].g << " " << (int)points[i].b << "\n";
#else
		ply << points[i].x << "     " << points[i].y << "      " << points[i].z << "\n";
#endif
	}
	ply.close();
}

void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, float squaresize)
{
	vector<Point3f> imgpoint;

	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

void startCali(const vector<string>& imagelist,
	Mat ir_cameraMatrix, Mat ir_distCoeffs,
	Mat rgb_cameraMatrix, Mat rgb_distCoeffs,
	Size boardSize, Size2f squareSize, float T_factor,
	string d2c_path)
{

	vector<vector<Point2f>> imagePointRGB;
	vector<vector<Point2f>> imagePointIR;
	vector<vector<Point3f>> objRealPoint;
	Size imageSize;

	/* 获取rgb和depth内参和畸变参数，畸变参数默认为0 */
	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);

	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	for (int i = 0; i < imagelist.size() / 2; i++) {
		vector<Point2f> cornerRGB;
		vector<Point2f> cornerIR;

		Mat rgb = imread(imagelist[i * 2 + 1]);
		Mat ir = imread(imagelist[i * 2]);

		BOOL isFindRGB = findChessboardCorners(rgb, boardSize, cornerRGB);
		BOOL isFindIR = findChessboardCorners(ir, boardSize, cornerIR);

		Mat grayrgb;
		cvtColor(rgb, grayrgb, CV_BGR2GRAY);
		Mat grayir;
		cvtColor(ir, grayir, CV_BGR2GRAY);

		if (isFindRGB == true && isFindIR == true)
		{
			cornerSubPix(grayrgb, cornerRGB, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//drawChessboardCorners(rgb, boardSize, cornerRGB, isFindRGB);
			//imshow("rgb", rgb);
			imagePointRGB.push_back(cornerRGB);

			cornerSubPix(grayir, cornerIR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//drawChessboardCorners(ir, boardSize, cornerIR, isFindIR);
			//imshow("ir", ir);
			imagePointIR.push_back(cornerIR);

			//waitKey(10);
		}
		else
		{
			cout << "寻找角点失败!!" << endl;
			return;
		}
	}

	calRealPoint(objRealPoint, boardSize.width, boardSize.height, imagelist.size() / 2, squareSize.width);

	Mat R, T, E, F;

	double rms = stereoCalibrate(objRealPoint, imagePointIR, imagePointRGB,
		ir_cameraMatrix, ir_distCoeffs, rgb_cameraMatrix, rgb_distCoeffs,
		imageSize,
		R, T, E, F,
		CALIB_FIX_INTRINSIC,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "rms = " << rms << endl;

	cout << "R:" << R << endl;
	cout << "T:" << T << endl;

	ofstream fout;
	stringstream ss;
	ss << d2c_path;
	fout.open(ss.str());
	fout << std::fixed << std::setprecision(20) << R.at<double>(0, 0) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(0, 1) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(0, 2) << ", ";
	fout << std::fixed << std::setprecision(20) << T.at<double>(0) / T_factor << ", " << "\n";
	fout << std::fixed << std::setprecision(20) << R.at<double>(1, 0) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(1, 1) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(1, 2) << ", ";
	fout << std::fixed << std::setprecision(20) << T.at<double>(1) / T_factor << ", " << "\n";
	fout << std::fixed << std::setprecision(20) << R.at<double>(2, 0) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(2, 1) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(2, 2) << ", ";
	fout << std::fixed << std::setprecision(20) << T.at<double>(2) / T_factor << ", " << "\n";
	fout << "0, 0, 0, 1;" << "\n";

	fout << std::fixed << std::setprecision(20) << "depth_fx:" << list_of_ints[6] << "\n";
	fout << std::fixed << std::setprecision(20) << "depth_fy:" << list_of_ints[7] << "\n";
	fout << std::fixed << std::setprecision(20) << "depth_cx:" << list_of_ints[4] << "\n";
	fout << std::fixed << std::setprecision(20) << "depth_cy:" << list_of_ints[5] << "\n";
	fout << std::fixed << std::setprecision(20) << "color_fx:" << list_of_ints[2] << "\n";
	fout << std::fixed << std::setprecision(20) << "color_fy:" << list_of_ints[3] << "\n";
	fout << std::fixed << std::setprecision(20) << "color_cx:" << list_of_ints[0] << "\n";
	fout << std::fixed << std::setprecision(20) << "color_cy:" << list_of_ints[1] << "\n";

	fout.close();
}

int main(int argc, char** argv)
{
	/* 校准板参数设定 */
	Size board_size(9, 6);
	Size2f square_size(27.0, 27.0);

	/* 变量初始化 */
	vector<string> image_list;
	/* RT矩阵中的T的倍率关系，跟着点云的单位走,
	如果点云精度是米，则设置为1.0，如果点云精度是毫米，则设置为1000.0 */
	const float RT_T_factor = 1000.0;

	/* 测试图片参数 */
	unsigned char* rgb_image0 = new unsigned char[RGB_W * RGB_H * 3];
	unsigned char* ir_image0 = new unsigned char[DEP_W * DEP_H];
	unsigned short* depth_image0 = new unsigned short[DEP_W * DEP_H];
	int count = 0;

	/* start Camera */
	MyCam.StartAllCam();
	MyCam.setLaserOff(true);

	/* 获取rgb和depth内参和畸变参数，畸变参数默认为0 */
	std::vector<float> list_of_ints = MyCam.get_cam_intrinsics(0);

	double fx_ir = list_of_ints[6];
	double fy_ir = list_of_ints[7];
	double cx_ir = list_of_ints[4];
	double cy_ir = list_of_ints[5];

	double k1_ir = 0.0;
	double k2_ir = 0.0;
	double p1_ir = 0.0;
	double p2_ir = 0.0;
	double k3_ir = 0.0;

	double fx_color = list_of_ints[2];
	double fy_color = list_of_ints[3];
	double cx_color = list_of_ints[0];
	double cy_color = list_of_ints[1];

	double k1_color = 0.0;
	double k2_color = 0.0;
	double p1_color = 0.0;
	double p2_color = 0.0;
	double k3_color = 0.0;

	cv::Mat irCamIntrix = (Mat_<double>(3, 3) << fx_ir, 0, cx_ir, 0, fy_ir, cy_ir, 0, 0, 1);
	cv::Mat irCamDistco = (Mat_<double>(5, 1) << k1_ir, k2_ir, p1_ir, p2_ir, k3_ir);
	cv::Mat rgbCamIntrix = (Mat_<double>(3, 3) << fx_color, 0, cx_color, 0, fy_color, cy_color, 0, 0, 1);
	cv::Mat rgbCamDistco = (Mat_<double>(5, 1) << k1_color, k2_color, p1_color, p2_color, k3_color);

	cout << "按c键保存图片!!" << endl;

	for (;;)
	{
		MyCam.infraredFrame(0, ir_image0);
		Mat irMat(DEP_H, DEP_W, CV_8UC1, ir_image0);
		imshow("irMat", irMat);

		MyCam.grabAndDeptFrame(0, rgb_image0, depth_image0);
		Mat rgbMat(RGB_H, RGB_W, CV_8UC3, rgb_image0);
		imshow("rgbMat", rgbMat);

		Mat depthMat(DEP_H, DEP_W, CV_16UC1, depth_image0);
		imshow("depthMat", depthMat);

		char c = waitKey(1);
		if (c == 27) {
			cout << "数据保存完成!!" << endl;
			break;
		}
		else if (c == 'c')
		{
			std::stringstream filepath_ir;
			filepath_ir << "E:\\Project\\demo\\dataset\\TEST_20\\ir_" << count << ".png";
			cv::imwrite(filepath_ir.str(), irMat);

			std::stringstream filepath_rgb;
			filepath_rgb << "E:\\Project\\demo\\dataset\\TEST_20\\rgb_" << count << ".png";
			cv::imwrite(filepath_rgb.str(), rgbMat);

			image_list.push_back(filepath_ir.str());
			image_list.push_back(filepath_rgb.str());

			count++;

			cout << "第 " << count << " 张" << endl;
		}
		else if (c == 's')
		{
			if (count > 15) {
				cout << "开始校准!!" << endl;
				std::stringstream filepath_d2c;
				filepath_d2c << "E:\\Project\\demo\\dataset\\TEST_20\\D2C.txt";
				startCali(image_list, irCamIntrix, irCamDistco, rgbCamIntrix, rgbCamDistco, board_size, square_size, RT_T_factor, filepath_d2c.str());
				cout << "校准完成!!" << endl;
			}
			else
			{
				cout << "请准备校准图片" << endl;
			}
		}
		else if (c == 't')
		{
			/* 测试RT准确性 */
			depth2PointCloud(depthMat, rgbMat, "E:\\Project\\demo\\dataset\\TEST_20\\ply.ply");
		}
	}
}
#elif defined TEST_20_1

#include <Eigen\Eigen>

using namespace cv;
using namespace std;

void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, float squaresize)
{
	vector<Point3f> imgpoint;

	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

void startCali(const vector<string>& imagelist, 
	Mat ir_cameraMatrix, Mat ir_distCoeffs,
	Mat rgb_cameraMatrix, Mat rgb_distCoeffs, 
	Size boardSize, Size2f squareSize, float T_factor,
	string d2c_path)
{

	vector<vector<Point2f>> imagePointRGB;
	vector<vector<Point2f>> imagePointIR;
	vector<vector<Point3f>> objRealPoint;
	Size imageSize;

	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	for (int i = 0; i < imagelist.size() / 2; i++) {
		vector<Point2f> cornerRGB;
		vector<Point2f> cornerIR;

		Mat rgb = imread(imagelist[i * 2 + 1]);
		//Mat rgb; flip(rgb1, rgb, 1);
		Mat ir = imread(imagelist[i * 2]);
		//Mat ir; flip(ir1, ir, 1);

		BOOL isFindRGB = findChessboardCorners(rgb, boardSize, cornerRGB);
		BOOL isFindIR = findChessboardCorners(ir, boardSize, cornerIR);

		Mat grayrgb;
		cvtColor(rgb, grayrgb, CV_BGR2GRAY);
		Mat grayir;
		cvtColor(ir, grayir, CV_BGR2GRAY);

		if (isFindRGB == true && isFindIR == true)
		{
			cornerSubPix(grayrgb, cornerRGB, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//drawChessboardCorners(rgb, boardSize, cornerRGB, isFindRGB);
			//imshow("rgb", rgb);
			imagePointRGB.push_back(cornerRGB);

			cornerSubPix(grayir, cornerIR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//drawChessboardCorners(ir, boardSize, cornerIR, isFindIR);
			//imshow("ir", ir);
			imagePointIR.push_back(cornerIR);

			//waitKey(10);
		}
		else
		{
			cout << "寻找角点失败!!" << endl;
			return;
		}
	}

	calRealPoint(objRealPoint, boardSize.width, boardSize.height, imagelist.size() / 2, squareSize.width);

	Mat R, T, E, F;

	double rms = stereoCalibrate(objRealPoint, imagePointIR, imagePointRGB,
		ir_cameraMatrix, ir_distCoeffs, rgb_cameraMatrix, rgb_distCoeffs,
		imageSize,
		R, T, E, F,
		CALIB_FIX_INTRINSIC,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "rms = " << rms << endl;

	cout << "R:" << R << endl;
	cout << "T:" << T << endl;

	ofstream fout;
	stringstream ss;
	ss << d2c_path;
	fout.open(ss.str());
	fout << std::fixed << std::setprecision(20) << R.at<double>(0, 0) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(0, 1) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(0, 2) << ", ";
	fout << std::fixed << std::setprecision(20) << T.at<double>(0) / T_factor << ", " << "\n";
	fout << std::fixed << std::setprecision(20) << R.at<double>(1, 0) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(1, 1) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(1, 2) << ", ";
	fout << std::fixed << std::setprecision(20) << T.at<double>(1) / T_factor << ", " << "\n";
	fout << std::fixed << std::setprecision(20) << R.at<double>(2, 0) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(2, 1) << ", ";
	fout << std::fixed << std::setprecision(20) << R.at<double>(2, 2) << ", ";
	fout << std::fixed << std::setprecision(20) << T.at<double>(2) / T_factor << ", " << "\n";
	fout << "0, 0, 0, 1;" << "\n";
	fout.close();
}

int main(int argc, char** argv)
{
	/* 校准板参数设定 */
	Size board_size(9, 6);
	Size2f square_size(22.0, 22.0);

	/* 变量初始化 */
	vector<string> image_list;
	/* RT矩阵中的T的倍率关系，跟着点云的单位走,
	如果点云精度是米，则设置为1.0，如果点云精度是毫米，则设置为1000.0 */
	const float RT_T_factor = 1.0;

	/* 测试图片参数 */
	int count = 1;
	const int pic_number = 19;

	/* 获取rgb和depth内参和畸变参数，畸变参数默认为0 */
#if 0
	double fx_ir = 853.5391734952462;
	double fy_ir = 855.674013508456;
	double cx_ir = 644.2137859285822;
	double cy_ir = 365.0641449141763;

	double k1_ir = -0.04136205083590676;
	double k2_ir = 0.4343361665883851;
	double p1_ir = 0.004405336669985451;
	double p2_ir = -0.007118606420216946;
	double k3_ir = -1.266784810430207;

	double fx_color = 1497.363516651454;
	double fy_color = 1500.403238796904;
	double cx_color = 906.6706247438445;
	double cy_color = 561.392425387452;

	double k1_color = -0.1321401239138183;
	double k2_color = 2.399033853150498;
	double p1_color = 0.002556803513408845;
	double p2_color = -0.009550886903681452;
	double k3_color = -9.613238787958464;
#else
	Mat cameraMattrix_rgb;
	Mat distCoeffs_rgb;
	FileStorage fs_rgb("C:\\Users\\Administrator\\Desktop\\02\\intrix_rgb.yml", FileStorage::READ);
	fs_rgb["cameraMatrix"] >> cameraMattrix_rgb;
	fs_rgb["distCoeffs"] >> distCoeffs_rgb;
	fs_rgb.release();

	Mat cameraMattrix_ir;
	Mat distCoeffs_ir;
	FileStorage fs_ir("C:\\Users\\Administrator\\Desktop\\02\\intrix_ir.yml", FileStorage::READ);
	fs_ir["cameraMatrix"] >> cameraMattrix_ir;
	fs_ir["distCoeffs"] >> distCoeffs_ir;
	fs_ir.release();

	double fx_ir = cameraMattrix_ir.at<double>(0, 0);
	double fy_ir = cameraMattrix_ir.at<double>(1, 1);
	double cx_ir = cameraMattrix_ir.at<double>(0, 2);
	double cy_ir = cameraMattrix_ir.at<double>(1, 2);

	double k1_ir = distCoeffs_ir.at<double>(0, 0);
	double k2_ir = distCoeffs_ir.at<double>(0, 1);
	double p1_ir = distCoeffs_ir.at<double>(0, 2);
	double p2_ir = distCoeffs_ir.at<double>(0, 3);
	double k3_ir = distCoeffs_ir.at<double>(0, 4);

	double fx_color = cameraMattrix_rgb.at<double>(0, 0);
	double fy_color = cameraMattrix_rgb.at<double>(1, 1);
	double cx_color = cameraMattrix_rgb.at<double>(0, 2);
	double cy_color = cameraMattrix_rgb.at<double>(1, 2);

	double k1_color = distCoeffs_rgb.at<double>(0, 0);
	double k2_color = distCoeffs_rgb.at<double>(0, 1);
	double p1_color = distCoeffs_rgb.at<double>(0, 2);
	double p2_color = distCoeffs_rgb.at<double>(0, 3);
	double k3_color = distCoeffs_rgb.at<double>(0, 4);

	//std::cout << fx_ir << std::endl;
	//std::cout << fy_ir << std::endl;
	//std::cout << cx_ir << std::endl;
	//std::cout << cy_ir << std::endl;
	//std::cout << k1_ir << std::endl;
	//std::cout << k2_ir << std::endl;
	//std::cout << p1_ir << std::endl;
	//std::cout << p2_ir << std::endl;
	//std::cout << k3_ir << std::endl;

	//std::cout << fx_color << std::endl;
	//std::cout << fy_color << std::endl;
	//std::cout << cx_color << std::endl;
	//std::cout << cy_color << std::endl;
	//std::cout << k1_color << std::endl;
	//std::cout << k2_color << std::endl;
	//std::cout << p1_color << std::endl;
	//std::cout << p2_color << std::endl;
	//std::cout << k3_color << std::endl;
#endif
	
	cv::Mat irCamIntrix = (Mat_<double>(3, 3) << fx_ir, 0, cx_ir, 0, fy_ir, cy_ir, 0, 0, 1);
	cv::Mat irCamDistco = (Mat_<double>(5, 1) << k1_ir, k2_ir, p1_ir, p2_ir, k3_ir);
	cv::Mat rgbCamIntrix = (Mat_<double>(3, 3) << fx_color, 0, cx_color, 0, fy_color, cy_color, 0, 0, 1);
	cv::Mat rgbCamDistco = (Mat_<double>(5, 1) << k1_color, k2_color, p1_color, p2_color, k3_color);

	//cout << "按c键保存图片!!" << endl;

	for (;;)
	{
		std::stringstream filepath_rgb;
		filepath_rgb << "C:\\Users\\Administrator\\Desktop\\02\\rgb_" << count << ".jpg";

		std::stringstream filepath_ir;
		filepath_ir << "C:\\Users\\Administrator\\Desktop\\02\\mono_" << count << ".png";

		image_list.push_back(filepath_ir.str());
		image_list.push_back(filepath_rgb.str());

		if (count >= pic_number)
			break;
		else
			count++;
	}

	cout << "开始校准!!" << endl;
	std::stringstream filepath_d2c;
	filepath_d2c << "C:\\Users\\Administrator\\Desktop\\02\\D2C.txt";
	startCali(image_list, irCamIntrix, irCamDistco, rgbCamIntrix, rgbCamDistco, board_size, square_size, RT_T_factor, filepath_d2c.str());
	cout << "校准完成!!" << endl;
}
#elif defined TEST_20_2

#include <Eigen\Eigen>

using namespace cv;
using namespace std;

#define CAM_TYPE_MONO 1
#define CAM_TYPE_RGB 2

//#define MONO_WIDTH 640
//#define MONO_HEIGHT 480

//#define RGB_WIDTH 1520
//#define RGB_HEIGHT 2016

#define PICTURE_NUM 14

int startCalibration(int img_count, int cam_type, Size img_size, Size board_size, Size2f square_size, vector<vector<Point2f>> image_points_seq)
{
	cout << "内参标定初始化。。。" << endl;

	if (img_count < 15)
	{
		std::cout << "输入图片太少，至少输入15组图片！！" << std::endl;
		return -1;
	}

	/* 图像尺寸 */
	Size image_size;
	/* 标定板上角点的三维坐标 */
	vector<vector<Point3f>> object_points;
	/* 摄像机内参数矩阵 */
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	/* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	/* 每幅图像的旋转向量 */
	vector<Mat> tvecsMat;
	/* 每幅图像的平移向量 */
	vector<Mat> rvecsMat;

	image_size.width = img_size.width;
	image_size.height = img_size.height;

	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t < img_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	if (cam_type == CAM_TYPE_MONO)
		cout << "开始标定MONO内参………………" << endl;
	else
		cout << "开始标定RGB内参………………" << endl;

	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

	bool ok = checkRange(cameraMatrix);
	if (ok)
	{
		cout << "标定完成！！" << endl;
	}
	else
	{
		cout << "标定失败！！" << endl;
		return -1;
	}

	cout << "cameraMatrix:" << cameraMatrix << endl;
	cout << "distCoeffs:" << distCoeffs << endl;

	if (cam_type == CAM_TYPE_MONO) {
		FileStorage fs("output\\intrinsics_mono.yml", FileStorage::WRITE);
		fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
		fs.release();
	}
	else {
		FileStorage fs("output\\intrinsics_rgb.yml", FileStorage::WRITE);
		fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
		fs.release();
	}

	cout << "开始评价标定结果………………\n";
	/* 所有图像的平均误差的总和 */
	double total_err = 0.0;
	/* 每幅图像的平均误差 */
	double err = 0.0;
	/* 保存重新计算得到的投影点 */
	vector<Point2f> image_points2;
	cout << "\t每幅图像的标定误差：\n";

	for (i = 0; i<img_count; i++)
	{
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		vector<Point3f> tempPointSet = object_points[i];
		/* 计算新的投影点和旧的投影点之间的误差*/
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);

		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += (err /= (board_size.height * board_size.width));
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	std::cout << "总体平均误差：" << total_err / img_count << "像素" << endl;
	std::cout << "评价完成！" << endl;

	return 0;
}

int cali_intrinsics(int cam_type, Size board_size, Size2f square_size)
{
	int count = 0;
	int ret = 0;
	Size img_size = Size(0, 0);

	/* 缓存每幅图像上检测到的角点 */
	vector<Point2f> corner_point;
	/* 总角点 */
	vector<vector<Point2f>> corner_point_total;

	for (;;)
	{
		corner_point.clear();
		std::stringstream file;

		if(cam_type == CAM_TYPE_MONO)
			file << "input\\mono_" << count << ".png";
		else
			file << "input\\rgb_" << count << ".jpg";

		Mat srcImg = imread(file.str());		

		if (0 == findChessboardCorners(srcImg, board_size, corner_point))
		{
			cout << "srcPic 检测角点失败！！:" << count << endl;
			return -1;
		}
		else
		{
			Mat srcImgGray;
			cvtColor(srcImg, srcImgGray, CV_BGR2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(srcImgGray, corner_point, Size(5, 5)); //对粗提取的角点进行精确化
			corner_point_total.push_back(corner_point);  //保存亚像素角点
			/* 在图像上显示角点位置 */
			//drawChessboardCorners(srcImgGray, board_size, corner_point, true); //用于在图片中标记角点
			//imshow("Camera Calibration", srcImgGray);//显示图片
			//waitKey(10);
		}

		if (count >= PICTURE_NUM)
			break;
		else
			count++;

		img_size.width = srcImg.cols;
		img_size.height = srcImg.rows;
	}

	ret = startCalibration(count + 1, cam_type, img_size, board_size, square_size, corner_point_total);

	return ret;
}

void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, float squaresize)
{
	vector<Point3f> imgpoint;

	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

int startCali(const vector<string>& imagelist,
	Mat mono_cameraMatrix, Mat mono_distCoeffs,
	Mat rgb_cameraMatrix, Mat rgb_distCoeffs,
	Size boardSize, Size2f squareSize, float T_factor,
	string d2c_path)
{

	vector<vector<Point2f>> imagePointRGB;
	vector<vector<Point2f>> imagePointMONO;
	vector<vector<Point3f>> objRealPoint;
	Size imageSize;

	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return -1;
	}

	for (int i = 0; i < imagelist.size() / 2; i++) {
		vector<Point2f> cornerRGB;
		vector<Point2f> cornerMONO;

		Mat rgb = imread(imagelist[i * 2 + 1]);
		Mat mono = imread(imagelist[i * 2]);

		BOOL isFindRGB = findChessboardCorners(rgb, boardSize, cornerRGB);
		BOOL isFindMONO = findChessboardCorners(mono, boardSize, cornerMONO);

		Mat grayrgb;
		cvtColor(rgb, grayrgb, CV_BGR2GRAY);
		Mat graymono;
		cvtColor(mono, graymono, CV_BGR2GRAY);

		if (isFindRGB == true && isFindMONO == true)
		{
			cornerSubPix(grayrgb, cornerRGB, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//drawChessboardCorners(rgb, boardSize, cornerRGB, isFindRGB);
			//imshow("rgb", rgb);
			imagePointRGB.push_back(cornerRGB);

			cornerSubPix(graymono, cornerMONO, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//drawChessboardCorners(ir, boardSize, cornerIR, isFindIR);
			//imshow("ir", ir);
			imagePointMONO.push_back(cornerMONO);

			//waitKey(10);
		}
		else
		{
			cout << "寻找角点失败!!" << endl;
			return -1;
		}
	}

	calRealPoint(objRealPoint, boardSize.width, boardSize.height, imagelist.size() / 2, squareSize.width);

	Mat R, T, E, F;

	double rms = stereoCalibrate(objRealPoint, imagePointMONO, imagePointRGB,
		mono_cameraMatrix, mono_distCoeffs, rgb_cameraMatrix, rgb_distCoeffs,
		imageSize,
		R, T, E, F,
		CALIB_FIX_INTRINSIC,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	cout << "rms = " << rms << endl;
	cout << "R:" << R << endl;
	cout << "T:" << T << endl;

	ofstream fout;
	stringstream ss;
	ss << d2c_path;
	fout.open(ss.str());
	fout << "fx_ir:" << std::fixed << std::setprecision(6) << mono_cameraMatrix.at<double>(0, 0) << "\n";
	fout << "fy_ir:" << std::fixed << std::setprecision(6) << mono_cameraMatrix.at<double>(1, 1) << "\n";
	fout << "cx_ir:" << std::fixed << std::setprecision(6) << mono_cameraMatrix.at<double>(0, 2) << "\n";
	fout << "cy_ir:" << std::fixed << std::setprecision(6) << mono_cameraMatrix.at<double>(1, 2) << "\n";
	fout << "fx_rgb:" << std::fixed << std::setprecision(6) << rgb_cameraMatrix.at<double>(0, 0) << "\n";
	fout << "fy_rgb:" << std::fixed << std::setprecision(6) << rgb_cameraMatrix.at<double>(1, 1) << "\n";
	fout << "cx_rgb:" << std::fixed << std::setprecision(6) << rgb_cameraMatrix.at<double>(0, 2) << "\n";
	fout << "cy_rgb:" << std::fixed << std::setprecision(6) << rgb_cameraMatrix.at<double>(1, 2) << "\n";
	fout << "matrix:";
	fout << std::fixed << std::setprecision(6) << R.at<double>(0, 0) << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(0, 1) << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(0, 2) << ", ";
	fout << std::fixed << std::setprecision(6) << T.at<double>(0) / T_factor << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(1, 0) << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(1, 1) << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(1, 2) << ", ";
	fout << std::fixed << std::setprecision(6) << T.at<double>(1) / T_factor << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(2, 0) << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(2, 1) << ", ";
	fout << std::fixed << std::setprecision(6) << R.at<double>(2, 2) << ", ";
	fout << std::fixed << std::setprecision(6) << T.at<double>(2) / T_factor << ", ";
	fout << "0, 0, 0, 1" << "\n";
	fout.close();

	return 0;
}

int cali_extrinsics(Size board_size, Size2f square_size)
{
	int count = 0;
	int ret = 0;

	/* 变量初始化 */
	vector<string> image_list;
	/* RT矩阵中的T的倍率关系，跟着点云的单位走,
	如果点云精度是米，则设置为1.0，如果点云精度是毫米，则设置为1000.0 */
	const float RT_T_factor = 1000.0;

	/* 获取rgb和mono内参和畸变参数，畸变参数默认为0 */
	Mat cameraMattrix_mono;
	Mat distCoeffs_mono;
	FileStorage fs_mono("output\\intrinsics_mono.yml", FileStorage::READ);
	fs_mono["cameraMatrix"] >> cameraMattrix_mono;
	fs_mono["distCoeffs"] >> distCoeffs_mono;
	fs_mono.release();

	Mat cameraMattrix_rgb;
	Mat distCoeffs_rgb;
	FileStorage fs_rgb("output\\intrinsics_rgb.yml", FileStorage::READ);
	fs_rgb["cameraMatrix"] >> cameraMattrix_rgb;
	fs_rgb["distCoeffs"] >> distCoeffs_rgb;
	fs_rgb.release();

	double fx_mono = cameraMattrix_mono.at<double>(0, 0);
	double fy_mono = cameraMattrix_mono.at<double>(1, 1);
	double cx_mono = cameraMattrix_mono.at<double>(0, 2);
	double cy_mono = cameraMattrix_mono.at<double>(1, 2);

	double k1_mono = distCoeffs_mono.at<double>(0, 0);
	double k2_mono = distCoeffs_mono.at<double>(0, 1);
	double p1_mono = distCoeffs_mono.at<double>(0, 2);
	double p2_mono = distCoeffs_mono.at<double>(0, 3);
	double k3_mono = distCoeffs_mono.at<double>(0, 4);

	double fx_rgb = cameraMattrix_rgb.at<double>(0, 0);
	double fy_rgb = cameraMattrix_rgb.at<double>(1, 1);
	double cx_rgb = cameraMattrix_rgb.at<double>(0, 2);
	double cy_rgb = cameraMattrix_rgb.at<double>(1, 2);

	double k1_rgb = distCoeffs_rgb.at<double>(0, 0);
	double k2_rgb = distCoeffs_rgb.at<double>(0, 1);
	double p1_rgb = distCoeffs_rgb.at<double>(0, 2);
	double p2_rgb = distCoeffs_rgb.at<double>(0, 3);
	double k3_rgb = distCoeffs_rgb.at<double>(0, 4);

	cv::Mat monoCamIntrix = (Mat_<double>(3, 3) << fx_mono, 0, cx_mono, 0, fy_mono, cy_mono, 0, 0, 1);
	cv::Mat monoCamDistco = (Mat_<double>(5, 1) << k1_mono, k2_mono, p1_mono, p2_mono, k3_mono);
	cv::Mat rgbCamIntrix = (Mat_<double>(3, 3) << fx_rgb, 0, cx_rgb, 0, fy_rgb, cy_rgb, 0, 0, 1);
	cv::Mat rgbCamDistco = (Mat_<double>(5, 1) << k1_rgb, k2_rgb, p1_rgb, p2_rgb, k3_rgb);

	for (;;)
	{
		std::stringstream filepath_mono;
		filepath_mono << "input\\mono_" << count << ".png";

		std::stringstream filepath_rgb;
		filepath_rgb << "input\\rgb_" << count << ".jpg";

		image_list.push_back(filepath_mono.str());
		image_list.push_back(filepath_rgb.str());

		if (count >= PICTURE_NUM)
			break;
		else
			count++;
	}

	cout << "开始校准!!" << endl;
	std::stringstream filepath_d2c;
	filepath_d2c << "output\\camerainfo_rs3m.txt";
	ret = startCali(image_list, monoCamIntrix, monoCamDistco, rgbCamIntrix, rgbCamDistco, board_size, square_size, RT_T_factor, filepath_d2c.str());
	cout << "校准完成!!" << endl;

	return ret;
}

int main(int argc, char** argv)
{
	/* 标定板上每行、列的角点数 */
	Size board_size = Size(9, 6);
	/* 标定板方格尺寸 */
	Size2f square_size(27.0, 27.0);
	
	if (cali_intrinsics(CAM_TYPE_MONO, board_size, square_size) == -1)
	{
		std::cout << "标定mono相机内参失败！！" << std::endl;
		return -1;
	}

	if (cali_intrinsics(CAM_TYPE_RGB, board_size, square_size) == -1)
	{
		std::cout << "标定rgb相机内参失败！！" << std::endl;
		return -1;
	}

	if (cali_extrinsics(board_size, square_size) == -1)
	{
		std::cout << "标定mono相机和rgb相机的外参失败！！" << std::endl;
		return -1;
	}
	
	return 0;
}
#elif defined TEST_21

using namespace std;
using namespace cv;

#define MONO_ARR_SIZE 16
#define MONO_SIZE_PER_CHANNEL 1

#define DEPTH_ARR_SIZE 8
#define DEPTH_SIZE_PER_CHANNEL 2

struct FileData {
	string ply;
	string format;
	int element;
	string propx;
	string propy;
	string propz;
	string end;

	std::vector<Point3f> vet;
};

float stringToFloat(const string& str)
{
	istringstream iss(str);
	float num;
	iss >> num;
	return num;
}

int stringToInt(const string& str)
{
	istringstream iss(str);
	int num;
	iss >> num;
	return num;
}

void getPlyData(string fileContent, struct FileData *ply) {

	if (fileContent.c_str() == nullptr)
	{
		return;
	}

	//强转字节流
	ifstream myfile(fileContent);
	string line;
	int count = 0;
	while (!myfile.eof())
	{
		getline(myfile, line);
		std::istringstream iss(line);
		if (strlen(line.c_str()) > 0) {
			if (line[0] == 'p' &&line[1] == 'l')
			{
				ply->ply = iss.str();
			}
			else if (line[0] == 'f' &&line[1] == 'o')
			{
				ply->format = iss.str();
			}
			else if (line[0] == 'e' && line[1] == 'l')
			{
				string element, vertex, num;
				iss >> element >> vertex >> num;
				ply->element = stringToInt(num);
			}
			else if (!line.compare("property float32 x"))
			{
				ply->propx = iss.str();
			}
			else if (!line.compare("property float32 y"))
			{
				ply->propy = iss.str();
			}
			else if (!line.compare("property float32 z"))
			{
				ply->propz = iss.str();
			}
			else if (line[0] == 'e' && line[1] == 'n')
			{
				ply->end = iss.str();
			}
			else
			{
				string x, y, z;
				Point3f vet;
				iss >> x >> y >> z;
				vet.x = stringToFloat(x);
				vet.y = stringToFloat(y);
				vet.z = stringToFloat(z);
				ply->vet.push_back(vet);
			}
		}
	}

	ply->vet.resize(ply->element);
}

void depth2PointCloud(cv::Mat imDepth, cv::Mat imRGB, float fx, float fy, float cx, float cy, std::string filePath)
{
	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {

			unsigned short Zw = imDepth.at<unsigned short>(i, j);
			if (Zw != 0) {
				point_st myp;

				myp.z = (float)(Zw / 10000.0);
				myp.x = (j - cx) * myp.z / fx;
				myp.y = (i - cy) * myp.z / fy;

				myp.b = imRGB.at<Vec3b>(i, j)[0];
				myp.g = imRGB.at<Vec3b>(i, j)[1];
				myp.r = imRGB.at<Vec3b>(i, j)[2];

				points.push_back(myp);
			}
		}
	}

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "property uchar red" << "\n";
	ply << "property uchar green" << "\n";
	ply << "property uchar blue" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
		ply << points[i].x << " " << points[i].y << " " << points[i].z << " " <<
			(int)points[i].r << " " << (int)points[i].g << " " << (int)points[i].b << "\n";
	}
	ply.close();
}

Mat distCoeffs_fixed(Mat intrinsic_matrix, Mat distortion_coeffs, Mat image, Mat& newCameraMatrix)
{
	Size image_size = image.size();

	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);

	newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));

	newCameraMatrix = getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0);

	cout << "矫正后的新相机内参:" << newCameraMatrix << endl;

	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, newCameraMatrix, image_size, CV_32FC1, mapx, mapy);

	Mat t = image.clone();

	cv::remap(image, t, mapx, mapy, INTER_NEAREST);

	return t;
}

int main(int argc, char** argv)
{
	float fx = 217.869247;
	float fy = 217.869247;
	float cx = 110.485161;
	float cy = 84.037331;

	float k1 = -0.136084;
	float k2 = -1.520149;
	float p1 = -0.000889;
	float p2 = 0.000396;
	float k3 = 3.170387;

#if 1
	/* mono raw to png */
	char mono_byte;
	std::vector<char> mono_data;
	string mono_raw = "C:\\Users\\Administrator\\Desktop\\2\\mono-34.raw";
	std::ifstream mono(mono_raw.c_str(), ios::binary);
	if (!mono)
	{
		cout << "Read file error" << endl;
		return -1;
	}
	mono_data.clear();
	mono_byte = 0;
	while (!mono.eof())
	{
		for (int i = 0; i < MONO_ARR_SIZE; ++i)
		{
			mono.read((char*)&mono_byte, MONO_SIZE_PER_CHANNEL);
			mono_data.push_back(mono_byte);
		}
	}
	mono.close();
	mono_data.resize(172 * 224);
	Mat mono_mat(172, 224, CV_8UC1, mono_data.data());
	cvtColor(mono_mat, mono_mat, CV_GRAY2RGB);	
	//imwrite("C:\\Users\\Administrator\\Desktop\\2\\mono-34-rgb.png", mono_mat);

	/* mono cali */
	Mat tofCamIntrix = (Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	Mat tofCamDistco = (Mat_<float>(1, 5) << k1, k2, p1, p2, k3);
	Mat newCameraMatrix;
	Mat mono_cali;
	mono_cali = distCoeffs_fixed(tofCamIntrix, tofCamDistco, mono_mat, newCameraMatrix);
	fx = newCameraMatrix.at<float>(0, 0);
	fy = newCameraMatrix.at<float>(1, 1);
	cx = newCameraMatrix.at<float>(0, 2);
	cy = newCameraMatrix.at<float>(1, 2);
	imwrite("C:\\Users\\Administrator\\Desktop\\2\\mono-34-rgb-cali.png", mono_cali);

	/* depth raw to png */
	/*unsigned short depth_byte;
	std::vector<unsigned short> depth_data;
	string depth_raw = "C:\\Users\\Administrator\\Desktop\\2\\depth-34.raw";
	std::ifstream depth(depth_raw.c_str(), ios::binary);
	if (!depth)
	{
		cout << "Read file error" << endl;
		return -1;
	}
	depth_data.clear();
	depth_byte = 0;
	while (!depth.eof())
	{
		for (int i = 0; i < DEPTH_ARR_SIZE; ++i)
		{
			depth.read((char*)&depth_byte, DEPTH_SIZE_PER_CHANNEL);
			depth_data.push_back(depth_byte);
		}
	}
	depth.close();
	depth_data.resize(172 * 224);
	Mat depth_mat(172, 224, CV_16UC1, depth_data.data());
	imwrite("C:\\Users\\Administrator\\Desktop\\2\\depth-raw-34.png", depth_mat);*/

	/* ply to depth 用原始内参 */
	struct FileData ply;
	getPlyData("C:\\Users\\Administrator\\Desktop\\2\\pmd-34.ply",&ply);
	Mat ply2depth = Mat::zeros(172, 224, CV_16UC1);
	for (int i=0; i < ply.vet.size(); i++)
	{
		float ux = fx * ply.vet[i].x / ply.vet[i].z + cx;
		float uy = fy * ply.vet[i].y / ply.vet[i].z + cy;

		int x = static_cast<int>(ux + 0.5f);
		int y = static_cast<int>(uy + 0.5f);

		if (x >= 0 && x < ply2depth.cols && y >= 0 && y < ply2depth.rows)
		{
			ply2depth.at<unsigned short>(y, x) = static_cast<int>(ply.vet[i].z * 10000);
		}
	}
	//imwrite("C:\\Users\\Administrator\\Desktop\\2\\ply2depth.png", ply2depth);

	/* ply2depth cali */
	Mat ply2depth_cali;
	ply2depth_cali = distCoeffs_fixed(tofCamIntrix, tofCamDistco, ply2depth, newCameraMatrix);
	imwrite("C:\\Users\\Administrator\\Desktop\\2\\ply2depth_cali.png", ply2depth_cali);

#else
	Mat ply2depth_cali = imread("C:\\Users\\Administrator\\Desktop\\2\\ply2depth_cali.png", -1);
	Mat mono_cali = imread("C:\\Users\\Administrator\\Desktop\\2\\mono-34-rgb-cali.png");
	depth2PointCloud(ply2depth_cali, mono_cali, fx, fy, cx, cy, "C:\\Users\\Administrator\\Desktop\\2\\new.ply");
#endif
}
#elif defined TEST_22

using namespace std;
using namespace cv;

#define MONO_ARR_SIZE 16
#define MONO_SIZE_PER_CHANNEL 1

#define DEPTH_ARR_SIZE 8
#define DEPTH_SIZE_PER_CHANNEL 2

void depth2PointCloud(cv::Mat imDepth, cv::Mat imRGB, float fx, float fy, float cx, float cy, std::string filePath)
{
	std::vector<point_st> points;

	points.clear();

	for (int i = 0; i < imDepth.rows; i++) {
		for (int j = 0; j < imDepth.cols; j++) {

			unsigned short Zw = imDepth.at<unsigned short>(i, j);
			if (Zw != 0) {
				point_st myp;

				myp.z = (float)((Zw & 0x1fff)/ 10000.0);
				myp.x = (j - cx) * myp.z / fx;
				myp.y = (i - cy) * myp.z / fy;

				myp.b = imRGB.at<Vec3b>(i, j)[0];
				myp.g = imRGB.at<Vec3b>(i, j)[1];
				myp.r = imRGB.at<Vec3b>(i, j)[2];

				points.push_back(myp);
			}
		}
	}

	std::ofstream ply(filePath.c_str());
	int num_p = points.size();
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "property uchar red" << "\n";
	ply << "property uchar green" << "\n";
	ply << "property uchar blue" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < points.size(); i++) {
		ply << points[i].x << " " << points[i].y << " " << points[i].z << " " <<
			(int)points[i].r << " " << (int)points[i].g << " " << (int)points[i].b << "\n";
	}
	ply.close();
}

int main(int argc, char** argv)
{
	float fx = 217.869247;
	float fy = 217.869247;
	float cx = 110.485161;
	float cy = 84.037331;

	float k1 = -0.136084;
	float k2 = -1.520149;
	float p1 = -0.000889;
	float p2 = 0.000396;
	float k3 = 3.170387;

	/* mono raw to png */
	char mono_byte;
	std::vector<char> mono_data;
	string mono_raw = "C:\\Users\\Administrator\\Desktop\\2\\mono-34.raw";
	std::ifstream mono(mono_raw.c_str(), ios::binary);
	if (!mono)
	{
		cout << "Read file error" << endl;
		return -1;
	}
	mono_data.clear();
	mono_byte = 0;
	while (!mono.eof())
	{
		for (int i = 0; i < MONO_ARR_SIZE; ++i)
		{
			mono.read((char*)&mono_byte, MONO_SIZE_PER_CHANNEL);
			mono_data.push_back(mono_byte);
		}
	}
	mono.close();
	mono_data.resize(172 * 224);
	Mat mono_mat(172, 224, CV_8UC1, mono_data.data());
	cvtColor(mono_mat, mono_mat, CV_GRAY2RGB);
	imwrite("C:\\Users\\Administrator\\Desktop\\2\\mono-34-rgb.png", mono_mat);


	/* depth raw to png */
	unsigned short depth_byte;
	std::vector<unsigned short> depth_data;
	string depth_raw = "C:\\Users\\Administrator\\Desktop\\2\\depth-34.raw";
	std::ifstream depth(depth_raw.c_str(), ios::binary);
	if (!depth)
	{
		cout << "Read file error" << endl;
		return -1;
	}
	depth_data.clear();
	depth_byte = 0;
	while (!depth.eof())
	{
		for (int i = 0; i < DEPTH_ARR_SIZE; ++i)
		{
			depth.read((char*)&depth_byte, DEPTH_SIZE_PER_CHANNEL);
			depth_data.push_back(depth_byte);
		}
	}
	depth.close();
	depth_data.resize(172 * 224);
	Mat depth_mat(172, 224, CV_16UC1, depth_data.data());
	imwrite("C:\\Users\\Administrator\\Desktop\\2\\depth-raw-34.png", depth_mat);

	Mat ply2depth_cali = imread("C:\\Users\\Administrator\\Desktop\\2\\depth-raw-34.png", -1);
	Mat mono_cali = imread("C:\\Users\\Administrator\\Desktop\\2\\mono-34-rgb.png");
	depth2PointCloud(ply2depth_cali, mono_cali, fx, fy, cx, cy, "C:\\Users\\Administrator\\Desktop\\2\\new.ply");
}
#elif defined TEST_23

using namespace std;
using namespace cv;

#define X1  
#define Y1

#define K1
#define K2

#define HEIGHT 480
#define WIDTH 640

int main(int argc, char** argv)
{
	VideoCapture capture;
	capture.open(1);

	if (!capture.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	Mat img;

	while (1)
	{
		capture >> img;	

		if (img.empty())
		{
			continue;
		}

		//cout << img.rows << endl;
		//cout << img.cols << endl;

		//equalizeHist();
		
		imshow("CamerFace", img);      // 显示				
		if (waitKey(1) > 0)		// delay ms 等待按键退出		
		{
			break;
		}
	}

}
#elif defined TEST_24

using namespace std;
using namespace cv;

CascadeClassifier lefteyeCascade;
CascadeClassifier righteyeCascade;

int main(int argc, char** argv)
{
	VideoCapture capture;
	capture.open(2);

	if (!capture.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	Mat img, imgGray;
	vector<Rect> lefteye;
	vector<Rect> righteye;
	lefteyeCascade.load("E:/3rd_lib/opencv2.4.13.5/build/share/OpenCV/haarcascades/haarcascade_mcs_leftear.xml");
	righteyeCascade.load("E:/3rd_lib/opencv2.4.13.5/build/share/OpenCV/haarcascades/haarcascade_mcs_rightear.xml");

	while (1)
	{
		capture >> img;

		if (img.empty())
		{
			continue;
		}

		if (img.channels() == 3)
		{
			cvtColor(img, imgGray, CV_RGB2GRAY);
		}
		else
		{
			imgGray = img;
		}

		lefteyeCascade.detectMultiScale(imgGray, lefteye, 1.2, 6, 0, Size(0, 0));
		if (lefteye.size() > 0)
		{
			for (int i = 0; i < lefteye.size(); i++)
			{
				rectangle(img, Point(lefteye[i].x, lefteye[i].y), Point(lefteye[i].x + lefteye[i].width, lefteye[i].y + lefteye[i].height), Scalar(0, 255, 0), 1, 8);
			}
		}
		else
		{
			righteyeCascade.detectMultiScale(imgGray, righteye, 1.2, 6, 0, Size(0, 0));
			if (righteye.size() > 0)
			{
				for (int i = 0; i < righteye.size(); i++)
				{
					rectangle(img, Point(righteye[i].x, righteye[i].y), Point(righteye[i].x + righteye[i].width, righteye[i].y + righteye[i].height), Scalar(0, 255, 0), 1, 8);
				}
			}
		}

		imshow("CamerFace", img);		
		if (waitKey(1) > 0)
		{
			break;
		}
	}
}
#elif defined TEST_25

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

	//yuvN21 to rgb
	const int YUV_ARR_SIZE = 16;
	const int YUV_SIZE_PER_CHANNEL = 1;

	for (int i = 0; i < 33;i++) {

		unsigned char yuv_ori;
		std::vector<unsigned char> yuv_table;
		std::stringstream filepath_yuv;
		filepath_yuv << "C:\\Users\\Administrator\\Desktop\\555\\rgb\\rgb_" << i << ".yuv";
		const char* pFileName_yuv = filepath_yuv.str().c_str();

		ifstream infile_yuv(pFileName_yuv, ios::binary);
		if (!infile_yuv)
		{
			cout << "Read file error" << endl;
			return -1;
		}

		yuv_table.clear();

		while (!infile_yuv.eof())
		{
			for (int i = 0; i < YUV_ARR_SIZE; ++i)
			{
				infile_yuv.read((char*)&yuv_ori, YUV_SIZE_PER_CHANNEL);
				yuv_table.push_back(yuv_ori);
			}
		}
		infile_yuv.close();
		cout << yuv_table.size() << endl;
		yuv_table.resize(3110400);

		Mat YUV(1080 * 3 / 2, 1920, CV_8UC1, yuv_table.data());
		Mat RGB;
		cvtColor(YUV, RGB, CV_YUV2BGR_NV21);

		std::stringstream filepath_rgb;
		filepath_rgb << "C:\\Users\\Administrator\\Desktop\\555\\rgb\\rgb_" << i << ".jpg";
		imwrite(filepath_rgb.str(), RGB);
	}

	//yuyv to gray(mono图)
	const int MONO_ARR_SIZE = 16;
	const int MONO_SIZE_PER_CHANNEL = 1;
	unsigned char* mono_buf = new unsigned char[1280 * 720 * 2];

	for (int j = 0; j < 33; j++) {

		unsigned char mono_ori;
		std::vector<unsigned char> mono_table;
		std::stringstream filepath_mono_yuv;
		filepath_mono_yuv << "C:\\Users\\Administrator\\Desktop\\555\\depth\\depth_" << j << ".yuv";
		const char* pFileName_mono = filepath_mono_yuv.str().c_str();

		ifstream infile_mono(pFileName_mono, ios::binary);
		if (!infile_mono)
		{
			cout << "Read file error" << endl;
			return -1;
		}
		mono_table.clear();

		while (!infile_mono.eof())
		{
			for (int i = 0; i < MONO_ARR_SIZE; ++i)
			{
				infile_mono.read((char*)&mono_ori, MONO_SIZE_PER_CHANNEL);
				mono_table.push_back(mono_ori);
			}
		}
		infile_mono.close();
		cout << mono_table.size() << endl;
		mono_table.resize(1280 * 900 * 2);

		memcpy(mono_buf, mono_table.data()+1280*180*2, 1280*720*2);

		Mat MONO_YUV(720, 1280, CV_8UC2, mono_buf);
		Mat MONO_GRAY;
		cvtColor(MONO_YUV, MONO_GRAY, CV_YUV2GRAY_YUYV);

		std::stringstream filepath_mono_rgb;
		filepath_mono_rgb << "C:\\Users\\Administrator\\Desktop\\555\\depth\\depth_" << j << ".jpg";
		imwrite(filepath_mono_rgb.str(), MONO_GRAY);
	}
}
#elif defined TEST_26

using namespace std;
using namespace cv;

int stringToInt(const string& str)
{
	istringstream iss(str);
	int num;
	iss >> num;
	return num;
}

float stringToFloat(const string& str)
{
	istringstream iss(str);
	float num;
	iss >> num;
	return num;
}

std::vector<std::string> split(std::string str, std::string separator) {
	std::vector<std::string> result;
	int cutAt;
	while ((cutAt = str.find_first_of(separator)) != str.npos) {
		if (cutAt>0) {
			result.push_back(str.substr(0, cutAt));
		}
		str = str.substr(cutAt + 1);
	}
	if (str.length()>0) {
		result.push_back(str);
	}
	return result;
}

typedef struct faceInfo
{
	int v_index[3];
	int vt_index[3];
	int vn_index[3];
};

typedef struct objData
{
	string mtllib;
	vector<cv::Vec3f> v;
	vector<cv::Vec2f> vt;
	vector<cv::Vec3f> vn;
	vector<faceInfo> faces;

	bool hasV = false;
	bool hasVn = false;
	bool hasVt = false;
};

objData* getObjData(string fileContent) {

	if (fileContent.c_str() == nullptr)
	{
		return NULL;
	}

	objData *obj = new objData;

	ifstream myfile(fileContent);
	string line;

	while (!myfile.eof())
	{
		getline(myfile, line);
		std::istringstream iss(line);
		if (strlen(line.c_str()) > 0) {
			if (line[0] == 'v' &&line[1] == 'n')
			{
				string type, a, b, c;
				cv::Vec3f vn;
				iss >> type >> a >> b >> c;
				vn[0] = stringToFloat(a);
				vn[1] = stringToFloat(b);
				vn[2] = stringToFloat(c);
				obj->vn.push_back(vn);
			}
			else if (line[0] == 'v' &&line[1] == ' ')
			{
				string type, a, b, c;
				cv::Vec3f v;
				iss >> type >> a >> b >> c;
				v[0] = stringToFloat(a);
				v[1] = stringToFloat(b);
				v[2] = stringToFloat(c);
				obj->v.push_back(v);
			}
			else if (line[0] == 'v' && line[1] == 't') 
			{
				string type, a, b;
				cv::Vec2f vt;
				iss >> type >> a >> b;
				vt[0] = stringToFloat(a);
				vt[1] = stringToFloat(b);
				obj->vt.push_back(vt);
			}
			else if (line[0] == 'f' && line[1] == ' ')
			{
				string type, a[3];
				faceInfo face;
				iss >> type >> a[0] >> a[1] >> a[2];
				for (int i = 0; i < 3; i++)
				{
					std::vector<std::string> indexs = split(a[i], "/");
					if (indexs.size() == 1)
					{
						face.v_index[i] = atoi(indexs[0].c_str());
						obj->hasV = true;
						obj->hasVt = false;
						obj->hasVn = false;
					}
					else if (indexs.size() == 2)
					{
						face.v_index[i] = atoi(indexs[0].c_str());
						face.vt_index[i] = atoi(indexs[1].c_str());
						obj->hasV = true;
						obj->hasVt = true;
						obj->hasVn = false;
					}
					else if (indexs.size() == 3)
					{
						face.v_index[i] = atoi(indexs[0].c_str());
						face.vt_index[i] = atoi(indexs[1].c_str());
						face.vn_index[i] = atoi(indexs[2].c_str());
						obj->hasV = true;
						obj->hasVt = true;
						obj->hasVn = true;
					}	
				}
				obj->faces.push_back(face);
			}
			else if (line[0] == 'm' && line[1] == 't') {
				string type, a;
				iss >> type >> a;
				obj->mtllib = type + ' ' + a;
			}
		}
	}

	return obj;
}

void setObjData(objData* obj, string filename)
{
	std::ofstream myobj(filename);
	myobj << obj->mtllib << endl;

	for (int index = 0; index < obj->v.size(); index++) {
		myobj << "v " << obj->v[index][0] << " " << obj->v[index][1] << " " << obj->v[index][2] << "\n";
	}

	if (obj->hasVt) {
		for (int index = 0; index < obj->vt.size(); index++)
		{
			myobj << "vt " << obj->vt[index][0] << " " << obj->vt[index][1] << "\n";
		}
	}

	if (obj->hasVn) {
		for (int index = 0; index < obj->vn.size(); index++)
		{
			myobj << "vn " << obj->vn[index][0] << " " << obj->vn[index][1] << " " << obj->vn[index][2] << "\n";
		}
	}

	for (int m = 0; m < obj->faces.size(); m++)
	{
		myobj << "f " << obj->faces[m].v_index[0];
		if (obj->hasVt)myobj << "/" << obj->faces[m].vt_index[0];
		if (obj->hasVn)myobj << "/" << obj->faces[m].vn_index[0];

		myobj << " " << obj->faces[m].v_index[1];
		if (obj->hasVt)myobj << "/" << obj->faces[m].vt_index[1];
		if (obj->hasVn)myobj << "/" << obj->faces[m].vn_index[1];

		myobj << " " << obj->faces[m].v_index[2];
		if (obj->hasVt)myobj << "/" << obj->faces[m].vt_index[2];
		if (obj->hasVn)myobj << "/" << obj->faces[m].vn_index[2];

		myobj << "\n";
	}

	myobj.close();

}

int main(int argc, char** argv)
{
	objData *obj = getObjData("C:\\Users\\Administrator\\Desktop\\1\\final_obj.obj");
	
	setObjData(obj, "C:\\Users\\Administrator\\Desktop\\1\\final_obj_1.obj");

	return 0;
}
#elif defined TEST_27
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h> 

using namespace std;
using namespace cv;

dlib::frontal_face_detector detector;
dlib::shape_predictor predictor;

void morpKeypoints(const std::vector<Point2f>& points1, const std::vector<Point2f>& points2, std::vector<Point2f>& pointsMorph, double alpha)
{
	for (int i = 0; i < points1.size(); i++)
	{
		float x, y;
		x = (1 - alpha) * points1[i].x + alpha * points2[i].x;
		y = (1 - alpha) * points1[i].y + alpha * points2[i].y;

		pointsMorph.push_back(Point2f(x, y));
	}
}

void applyAffineTransform(Mat &warpImage, Mat &src, std::vector<Point2f> & srcTri, std::vector<Point2f> & dstTri)
{
	Mat warpMat = getAffineTransform(srcTri, dstTri);

	warpAffine(src, warpImage, warpMat, warpImage.size(), cv::INTER_LINEAR, BORDER_REFLECT_101);
}

void morphTriangle(Mat &img1, Mat &img2, Mat &img, std::vector<Point2f> &t1, std::vector<Point2f> &t2, std::vector<Point2f> &t, double alpha)
{
	Rect r = cv::boundingRect(t);
	Rect r1 = cv::boundingRect(t1);
	Rect r2 = cv::boundingRect(t2);

	std::vector<Point2f> t1Rect, t2Rect, tRect;
	std::vector<Point> tRectInt;
	for (int i = 0; i < 3; ++i)
	{
		tRect.push_back(Point2f(t[i].x - r.x, t[i].y - r.y));
		tRectInt.push_back(Point(t[i].x - r.x, t[i].y - r.y));

		t1Rect.push_back(Point2f(t1[i].x - r1.x, t1[i].y - r1.y));
		t2Rect.push_back(Point2f(t2[i].x - r2.x, t2[i].y - r2.y));
	}

	Mat mask = Mat::zeros(r.height, r.width, CV_32FC3);
	fillConvexPoly(mask, tRectInt, Scalar(1.0, 1.0, 1.0), 16, 0);

	Mat img1Rect, img2Rect;
	img1(r1).copyTo(img1Rect);
	img2(r2).copyTo(img2Rect);

	Mat warpImage1 = Mat::zeros(r.height, r.width, img1Rect.type());
	Mat warpImage2 = Mat::zeros(r.height, r.width, img2Rect.type());

	applyAffineTransform(warpImage1, img1Rect, t1Rect, tRect);
	applyAffineTransform(warpImage2, img2Rect, t2Rect, tRect);

	//Mat imgRect = (1.0 - alpha)*warpImage1 + alpha*warpImage2;
	Mat imgRect = (alpha)*warpImage1 + (1.0 - alpha)*warpImage2;

	multiply(imgRect, mask, imgRect);
	multiply(img(r), Scalar(1.0, 1.0, 1.0) - mask, img(r));
	img(r) = img(r) + imgRect;
}

typedef struct correspondens {
	std::vector<int> index;
};

void delaunayTriangulation(const std::vector<Point2f>& points1, const std::vector<Point2f>& points2,
	std::vector<Point2f>& pointsMorph, double alpha, std::vector<correspondens>& delaunayTri, Size imgSize)
{
	morpKeypoints(points1, points2, pointsMorph, alpha);

	Rect rect(0, 0, imgSize.width, imgSize.height);

	cv::Subdiv2D subdiv(rect);
	for (std::vector<Point2f>::iterator it = pointsMorph.begin(); it != pointsMorph.end(); it++)
		subdiv.insert(*it);

	std::vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);

	for (size_t i = 0; i < triangleList.size(); ++i)
	{
		std::vector<Point2f> pt;
		correspondens ind;
		Vec6f t = triangleList[i];
		pt.push_back(Point2f(t[0], t[1]));
		pt.push_back(Point2f(t[2], t[3]));
		pt.push_back(Point2f(t[4], t[5]));

		if (rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
		{
			int count = 0;
			for (int j = 0; j < 3; ++j)
				for (size_t k = 0; k < pointsMorph.size(); k++) {
					if (abs(pt[j].x - pointsMorph[k].x) < 1.0   &&  abs(pt[j].y - pointsMorph[k].y) < 1.0)
					{
						ind.index.push_back(k);
						count++;
					}
				}
			if (count == 3) {
				delaunayTri.push_back(ind);
			}
		}
	}
}

void morp(Mat &img1, Mat &img2, Mat& imgMorph, double alpha, const std::vector<Point2f> &points1, const std::vector<Point2f> &points2, const std::vector<correspondens> &triangle)
{
	img1.convertTo(img1, CV_32F);
	img2.convertTo(img2, CV_32F);

	std::vector<Point2f> points;
	morpKeypoints(points1, points2, points, alpha);

	int v0, v1, v2;

	for (int i = 0; i < triangle.size(); ++i)
	{
		correspondens corpd = triangle[i];

		v0 = corpd.index[0];
		v1 = corpd.index[1];
		v2 = corpd.index[2];

		std::vector<Point2f> t1, t2, t;

		t1.push_back(points1[v0]);
		t1.push_back(points1[v1]);
		t1.push_back(points1[v2]);

		t2.push_back(points2[v0]);
		t2.push_back(points2[v1]);
		t2.push_back(points2[v2]);

		t.push_back(points[v0]);
		t.push_back(points[v1]);
		t.push_back(points[v2]);

		morphTriangle(img1, img2, imgMorph, t1, t2, t, alpha);
	}

}

void addKeypoints(std::vector<Point2f>& points, Size imgSize)
{
	points.push_back(Point2f(1, 1));
	points.push_back(Point2f(1, imgSize.height - 1));
	points.push_back(Point2f(imgSize.width - 1, imgSize.height - 1));
	points.push_back(Point2f(imgSize.width - 1, 1));
	points.push_back(Point2f(1, imgSize.height / 2));
	points.push_back(Point2f(imgSize.width / 2, imgSize.height - 1));
	points.push_back(Point2f(imgSize.width - 1, imgSize.height / 2));
	points.push_back(Point2f(imgSize.width / 2, 1));
}

void dlibInit(void)
{
	detector = dlib::get_frontal_face_detector();
	dlib::deserialize("E:\\3rd_lib\\dlib-19.9\\shape_predictor_68_face_landmarks.dat") >> predictor;
}

int getLandmarksByDlib(std::vector<Point2f>& points, Mat img, int scale, bool isTarget)
{
	Mat temp;

	resize(img, temp, Size(img.cols/scale, img.rows/scale));

	dlib::cv_image<dlib::bgr_pixel> cimg(temp);

	// Detect faces
	std::vector<dlib::rectangle> faces = detector(cimg);

	// Find the pose of each face 
	if (faces.size() > 0)
	{
		//track features
		dlib::full_object_detection shape = predictor(cimg, faces[0]);

		//draw features
		for (unsigned int i = 0; i < 60; ++i)
		{
			Point2f pt;

			pt.x = shape.part(i).x() * scale;
			pt.y = shape.part(i).y() * scale;

			if (pt.x >= 0 && pt.x < img.cols && pt.y >= 0 && pt.y < img.rows) {
				points.push_back(pt);
				//circle(img, Point(pt.x, pt.y), 2, Scalar(0, 0, 255), -1);
				//std::cout << "point" << i << ":" << pt.x << " " << pt.y << std::endl;
			}	
		}

		if (isTarget == true)
		{}

		return 0;
	}
	
	return -1;
}

int main(int argc, char** argv)
{
	int fdt_scale = 2;

	std::vector<Point2f> landmarks1;
	std::vector<Point2f> landmarks2;

	dlibInit();

	Mat img1 = imread("C:\\Users\\Administrator\\Desktop\\test\\huan.jpg");
	Mat img2 = imread("C:\\Users\\Administrator\\Desktop\\test\\song.jpg");

	Mat img1_prcs = img1.clone();
	Mat img2_prcs = img2.clone();

	if (getLandmarksByDlib(landmarks1, img1, fdt_scale, false) || getLandmarksByDlib(landmarks2, img2, fdt_scale, true))
	{
		std::cout << "get landmark failed!!" << std::endl;
		return 0;
	}

	addKeypoints(landmarks1, img1.size());
	addKeypoints(landmarks2, img2.size());

#if 0
	std::vector<Vec6f> triangleList1;
	Rect rect1(0, 0, img1.size().width, img1.size().height);
	Subdiv2D subdiv1(rect1);
	for (std::vector<Point2f>::iterator it = landmarks1.begin(); it != landmarks1.end(); it++) {
		subdiv1.insert(*it);
	}
	subdiv1.getTriangleList(triangleList1);

	std::vector<Vec6f> triangleList2;
	Rect rect2(0, 0, img2.size().width, img2.size().height);
	Subdiv2D subdiv2(rect2);
	for (std::vector<Point2f>::iterator it = landmarks2.begin(); it != landmarks2.end(); it++) {
		subdiv2.insert(*it);
	}
	subdiv2.getTriangleList(triangleList2);

	for (int i = 0; i < triangleList1.size(); i++)
	{
		line(img1, Point(triangleList1[i][0], triangleList1[i][1]), Point(triangleList1[i][2], triangleList1[i][3]), Scalar(0, 0, 255));
		line(img1, Point(triangleList1[i][2], triangleList1[i][3]), Point(triangleList1[i][4], triangleList1[i][5]), Scalar(0, 0, 255));
		line(img1, Point(triangleList1[i][4], triangleList1[i][5]), Point(triangleList1[i][0], triangleList1[i][1]), Scalar(0, 0, 255));
	}

	for (int i = 0; i < triangleList2.size(); i++)
	{
		line(img2, Point(triangleList2[i][0], triangleList2[i][1]), Point(triangleList2[i][2], triangleList2[i][3]), Scalar(0, 0, 255));
		line(img2, Point(triangleList2[i][2], triangleList2[i][3]), Point(triangleList2[i][4], triangleList2[i][5]), Scalar(0, 0, 255));
		line(img2, Point(triangleList2[i][4], triangleList2[i][5]), Point(triangleList2[i][0], triangleList2[i][1]), Scalar(0, 0, 255));
	}
#else
	Mat imgMorph = Mat::zeros(img1.size(), CV_32FC3);
	std::vector<Point2f> pointsMorph;
	std::vector<correspondens> delaunayTri;
	float alpha = 1.0f;

	delaunayTriangulation(landmarks1, landmarks2, pointsMorph, alpha, delaunayTri, img1.size());
	morp(img1_prcs, img2_prcs, imgMorph, alpha, landmarks1, landmarks2, delaunayTri);

	Mat resultImg;
	imgMorph.convertTo(resultImg, CV_8UC3);

#endif

	while (1) {
		imshow("img1", img1);
		imshow("img2", img2);
		imshow("resultImg", resultImg);

		char c = waitKey(1);
		if (c == 27) {
			break;
		}
	}

	return 0;
}
#elif defined TEST_28

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	Mat cameraMattrix1 = (Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
	Mat distCoeffs1 = (Mat_<double>(5, 1) << 0.1, 0.01, -0.001, 0, 0);
	
	FileStorage fsw("E:\\Project\\demo\\dataset\\TEST_28\\test.yml", FileStorage::WRITE);
	fsw << "cameraMatrix" << cameraMattrix1 << "distCoeffs" << distCoeffs1;
	fsw.release();

	Mat cameraMattrix2;
	Mat distCoeffs2;
	FileStorage fsr("E:\\Project\\demo\\dataset\\TEST_28\\test.yml", FileStorage::READ);
	fsr["cameraMatrix"] >> cameraMattrix2;
	fsr["distCoeffs"] >> distCoeffs2;

	std::cout << cameraMattrix2.at<double>(0, 0) << std::endl;
	std::cout << cameraMattrix2.at<double>(0, 2) << std::endl;
	std::cout << cameraMattrix2.at<double>(2, 0) << std::endl;

	std::cout << distCoeffs2.at<double>(0, 0) << std::endl;
	std::cout << distCoeffs2.at<double>(1, 0) << std::endl;


	return 0;
}

#elif defined TEST_29

#include <Eigen\Eigen>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	Mat pose;

	for (int i = 1; i < 63; i++) {
		std::stringstream poseL_file;
		poseL_file << "C:\\Users\\Administrator\\Desktop\\pose\\poseL2\\cam_" << setfill('0') << setw(4) << i << ".yml";
		FileStorage poseL(poseL_file.str(), FileStorage::READ);
		poseL["R"] >> pose;
		poseL.release();

		Eigen::Matrix3d R;

		R(0, 0) = pose.at<double>(0, 0);
		R(0, 1) = pose.at<double>(0, 1);
		R(0, 2) = pose.at<double>(0, 2);

		R(1, 0) = pose.at<double>(1, 0);
		R(1, 1) = pose.at<double>(1, 1);
		R(1, 2) = pose.at<double>(1, 2);

		R(2, 0) = pose.at<double>(2, 0);
		R(2, 1) = pose.at<double>(2, 1);
		R(2, 2) = pose.at<double>(2, 2);

		//cout << R.eulerAngles(0, 1, 2)*180.0 / DEMO_PI << endl;

		//std::cout << "pose:" << pose << std::endl;
		//std::cout << "R:" << R << std::endl;

		double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
		double xx = 0, yy = 0, zz = 0;

		bool singular = sy < 1e-6;
		if (singular)
		{
			xx = atan2(-R(1, 2), R(1, 1));
			yy = atan2(-R(2, 0), sy);
			zz = 0;
		}
		else
		{
			xx = atan2(R(2, 1), R(2, 2));
			yy = atan2(-R(2, 0), sy);
			zz = atan2(R(1, 0), R(0, 0));
		}

		//std::cout << "x:" << xx * 180.0 / DEMO_PI << std::endl;
		std::cout << "y:" << yy * 180.0 / DEMO_PI << std::endl;
		//std::cout << "z:" << zz * 180.0 / DEMO_PI << std::endl;
	}

	return 0;
}
#elif defined TEST_30

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	Mat img = imread("C:\\Users\\Administrator\\Desktop\\depth\\landmark.png");
	imshow("原始图像", img);
	Mat out = img.clone();
	Mat M = Mat(2, 3, CV_64F);
	M.at<double>(0, 0) = 1.0f;
	M.at<double>(0, 1) = 0.0f;
	M.at<double>(0, 2) = 0.0f;
	M.at<double>(1, 0) = 0.0f;
	M.at<double>(1, 1) = 1.0f;
	M.at<double>(1, 2) = 0.0f;

	//图像移动
	std::cout << "图像移动" << std::endl;
	int type = 1;
	double tx = 0;
	double ty = 0;
	while (type == 1)
	{
		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'a':
			tx += 1.0;
			ty += 1.0;			
			break;
		case 's':
			tx -= 1.0;
			ty -= 1.0;
			break;
		}
		M.at<double>(0, 2) = tx;
		M.at<double>(1, 2) = ty;
		warpAffine(img, out, M, out.size());
		imshow("图像移动", out);
	}

	//图像缩放
	std::cout << "图像缩放" << std::endl;
	type = 2;
	double sx = 1;
	double sy = 1;
	while (type == 2)
	{
		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'a':
			sx += 0.01;
			sy += 0.01;
			break;
		case 's':
			sx -= 0.01;
			sy -= 0.01;
			break;
		}
		M.at<double>(0, 0) = sx;
		M.at<double>(1, 1) = sy;
		Size newSize(img.cols*sx, img.rows*sy);
		warpAffine(img, out, M, newSize);
		imshow("图像缩放", out);
	}

	//图像旋转
	std::cout << "图像旋转" << std::endl;
	type = 3;
	double degree = 0;

	Point2f center(img.cols / 2, img.rows / 2);
	while (type == 3)
	{
		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'a':
			degree += DEMO_PI / 180;
			break;
		case 's':
			degree -= DEMO_PI / 180;
			break;
		}
		M.at<double>(0, 0) = cos(degree);
		M.at<double>(0, 1) = -sin(degree);
		M.at<double>(0, 2) = center.x - center.x*cos(degree) + center.y*sin(degree);
		M.at<double>(1, 0) = sin(degree);
		M.at<double>(1, 1) = cos(degree);
		M.at<double>(1, 2) = center.y - center.x*sin(degree) - center.y*cos(degree);
		Rect newRect = RotatedRect(center, img.size(), degree*(180 / DEMO_PI)).boundingRect();
		M.at<double>(0, 2) += newRect.width / 2.0 - center.x;
		M.at<double>(1, 2) += newRect.height / 2.0 - center.y;
		warpAffine(img, out, M, newRect.size());
		imshow("图像旋转", out);
	}

	//图像错切
	std::cout << "图像斜切" << std::endl;
	type = 4;
	double shx = 0;
	double shy = 0;
	while (type == 4)
	{
		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'a':
			shx += 0.01;
			break;
		case 's':
			shx -= 0.01;
			break;
		case 'd':
			shy += 0.01;
			break;
		case 'f':
			shy -= 0.01;
			break;
		}
		M.at<double>(0, 1) = shx;
		M.at<double>(0, 2) = (shx > 0) ? 0 : img.rows*fabs(shx);
		M.at<double>(1, 0) = shy;
		M.at<double>(1, 2) = (shy > 0) ? 0 : img.cols*fabs(shy);
		Size newSize(img.cols + img.rows*fabs(shx), img.rows + img.cols*fabs(shy));
		warpAffine(img, out, M, newSize);
		imshow("图像斜切", out);
	}

	return 0;
}
#elif defined TEST_31

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h> 

using namespace std;
using namespace cv;

void rotation(Mat& input, Mat& output, Point2f center, double theta)
{
	Mat M = Mat(2, 3, CV_64F);

	M.at<double>(0, 0) = cos(theta);
	M.at<double>(0, 1) = -sin(theta);
	M.at<double>(0, 2) = center.x - center.x*cos(theta) + center.y*sin(theta);
	M.at<double>(1, 0) = sin(theta);
	M.at<double>(1, 1) = cos(theta);
	M.at<double>(1, 2) = center.y - center.x*sin(theta) - center.y*cos(theta);
	warpAffine(input, output, M, output.size());
}

int main()
{
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor predictor;
	dlib::deserialize("E:\\3rd_lib\\dlib-19.9\\shape_predictor_68_face_landmarks.dat") >> predictor;

	Mat temp = imread("C:\\Users\\Administrator\\Desktop\\test\\test_Color.png");
	dlib::cv_image<dlib::bgr_pixel> cimg(temp);

	cout << "人脸检测中。。。。" << endl;
	std::vector<dlib::rectangle> faces = detector(cimg);
	if (faces.size() == 0)
	{
		cout << "未检测到人脸" << endl;
		return 0;
	}
	
	dlib::full_object_detection shape = predictor(cimg, faces[0]);
	for (unsigned int i = 0; i < 68; ++i)
	{
		cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
	}
	imshow("temp", temp);

	//42:右侧内眼角
	//39:左侧内眼角
	//30:鼻尖点

	//求左右内眼角构成的直线与水平线的夹角
	double deltaX = shape.part(42).x() - shape.part(39).x();
	double deltaY = shape.part(42).y() - shape.part(39).y();
	double theta = -atan(deltaY / deltaX);
	cout << "theta:" << theta << endl;

	//将图像以鼻尖点为中心点旋转theta角度
	Mat outputImg = Mat(temp.rows, temp.cols, temp.type());
	Point2f center(shape.part(30).x(), shape.part(30).y());
	rotation(temp, outputImg, center, theta);
	imshow("outputImg", outputImg);
	while (1)
	{
		char c = (char)waitKey(10);
		if (c == 27)
			break;
	}
}
#elif defined TEST_32

#include <Eigen\Eigen>

using namespace std;
using namespace cv;

int main()
{
	Eigen::MatrixXf calMat(4, 4);

	calMat << 1,  2,  3,  4,
	          5,  6,  7,  8,
	          9, 10, 11, 12,
	         13, 14, 15, 16;

	std::cout << "a:" << calMat.block<2/*2行*/, 4/*4列*/>(0/*从第0行开始*/, 1/*从第1列开始*/) << std::endl;

	Eigen::Matrix<float, 8, 1> calMat1;
	calMat1 << 1, 2, 3, 4, 5, 6, 7, 8;
	std::cout << "b:" << calMat1.segment<2/*2个*/>(5/*从位置5开始*/) << std::endl;
}
#elif defined TEST_33

#include <Eigen\Eigen>
#include <opencv2/core/eigen.hpp>
#include <io.h>

/* vet number */
#define PCA_ROWS 12789

/* save mode */
#define OPENCV_MODE        (0)
#define TXT_MODE           (1)
#define FILE_SAVE_MODE     (TXT_MODE)

/* multiple */
#define VERTEX_MULTIPLE    (1000)

using namespace std;
using namespace cv;

float stringToFloat(const string& str)
{
	istringstream iss(str);
	float num;
	iss >> num;
	return num;
}

typedef struct objData
{
	vector<cv::Vec3f> v;
};

objData* loadObj(string fileContent) {

	if (fileContent.c_str() == nullptr)
	{
		return NULL;
	}

	objData *obj = new objData;

	ifstream myfile(fileContent);
	string line;

	while (!myfile.eof())
	{
		getline(myfile, line);
		std::istringstream iss(line);
		if (strlen(line.c_str()) > 0) {
			if (line[0] == 'v' &&line[1] == ' ')
			{
				string type, a, b, c;
				cv::Vec3f v;
				iss >> type >> a >> b >> c;
				v[0] = stringToFloat(a) * VERTEX_MULTIPLE;
				v[1] = stringToFloat(b) * VERTEX_MULTIPLE;
				v[2] = stringToFloat(c) * VERTEX_MULTIPLE;
				obj->v.push_back(v);
			}
		}
	}
	myfile.close();

	return obj;
}

void save2Txt(string type, string path, Mat &data)
{
	if (strcmp(type.c_str(), "eigenValues") == 0)
	{
		ofstream eigenValuesWrite(path);
		for (int i = 0; i < data.cols; i++)
		{
			eigenValuesWrite << data.at<float>(0, i) << endl;
		}
		eigenValuesWrite.close();
	}
	else if (strcmp(type.c_str(), "eigenVector") == 0)
	{
		ofstream eigenVectorWrite(path);
		for (int i = 0; i < data.rows; i++)
		{
			for (int j = 0; j < data.cols; j++)
			{
				eigenVectorWrite << data.at<float>(i, j) << " ";
			}
			eigenVectorWrite << endl;
		}
		eigenVectorWrite.close();
	}
	else if (strcmp(type.c_str(), "meanShape") == 0)
	{
		ofstream meanShapeWrite(path);
		for (int i = 0; i < data.rows; i++)
		{
			meanShapeWrite << data.at<float>(i, 0) << endl;
		}
		meanShapeWrite.close();
	}
	else if (strcmp(type.c_str(), "alpha") == 0)
	{
		ofstream alphaWrite(path);
		for (int i = 0; i < data.rows; i++)
		{
			alphaWrite << data.at<float>(i, 0) << endl;
		}
		alphaWrite.close();
	}
}

void readFromTxt(string type, string path, Mat &data)
{
	if (strcmp(type.c_str(), "eigenValues") == 0)
	{
		ifstream eigenValuesRead(path);
		string line;
		int row = 0;
		while (!eigenValuesRead.eof())
		{
			getline(eigenValuesRead, line);
			if (strlen(line.c_str()) > 0) {
				std::istringstream iss(line);
				string value;
				iss >> value;
				data.at<float>(0, row) = stringToFloat(value);
			}
			row++;
		}
		eigenValuesRead.close();
	}
	else if (strcmp(type.c_str(), "eigenVector") == 0)
	{
		ifstream eigenVectorRead(path);
		string line;
		int row = 0;
		int col = 0;
		while (!eigenVectorRead.eof())
		{
			getline(eigenVectorRead, line);
			if (strlen(line.c_str()) > 0) {
				std::istringstream iss(line);
				string value;
				col = 0;
				while (iss >> value) {
					data.at<float>(row, col) = stringToFloat(value);
					col++;
				}
			}
			row++;
		}
		eigenVectorRead.close();
	}
	else if (strcmp(type.c_str(), "meanShape") == 0)
	{
		ifstream meanShapeRead(path);
		string line;
		int row = 0;
		while (!meanShapeRead.eof())
		{
			getline(meanShapeRead, line);
			if (strlen(line.c_str()) > 0) {
				std::istringstream iss(line);
				string value;
				iss >> value;
				data.at<float>(row, 0) = stringToFloat(value);
			}
			row++;
		}
		meanShapeRead.close();
	}
}

void createPly(string filePath, Mat &points)
{
	std::ofstream ply(filePath);
	int num_p = PCA_ROWS;
	ply << "ply" << "\n" << "format ascii 1.0" << "\n";
	ply << "element vertex " << num_p << "\n";
	ply << "property float x" << "\n";
	ply << "property float y" << "\n";
	ply << "property float z" << "\n";
	ply << "end_header" << "\n";

	for (int i = 0; i < num_p; i++) {
		ply << points.at<float>(i * 3 + 0, 0) << " " 
			<< points.at<float>(i * 3 + 1, 0) << " "
			<< points.at<float>(i * 3 + 2, 0) << "\n";
	}
	ply.close();
}

void createPcaMatrix(std::vector<string> &fileList, Mat &PCA)
{
	for (int col = 0; col < fileList.size(); col++)
	{
		objData *obj = NULL;
		obj = loadObj(fileList[col]);
		for (int row = 0; row < PCA_ROWS; row++)
		{
			PCA.at<float>(row * 3 + 0, col) = obj->v[row][0];
			PCA.at<float>(row * 3 + 1, col) = obj->v[row][1];
			PCA.at<float>(row * 3 + 2, col) = obj->v[row][2];
		}
		delete obj;
		cout << (float)(col + 1) / fileList.size() << "%" << endl;
	}
}

Mat getMeanShape(Mat &PCA)
{
	Mat mean = Mat::zeros(PCA_ROWS * 3, 1, CV_32FC1);

	for (int i = 0; i < PCA.cols; i++)
	{
		mean += PCA.col(i);
	}

	mean = mean / PCA.cols;

	for (int i = 0; i < PCA.cols; i++)
	{
		PCA.col(i) -= mean;
	}

	return mean;
}

void calculatePCA(Mat &eigenValues, Mat &eigenVector, Mat &mean, std::vector<string> &fileList)
{
	Mat PcaMatrix(PCA_ROWS * 3, fileList.size(), CV_32FC1);
	createPcaMatrix(fileList, PcaMatrix);

	Mat PcaMatrixDelta = PcaMatrix.clone();
	mean = getMeanShape(PcaMatrixDelta);

	PCA pcaset(PcaMatrixDelta, Mat(), 1);/*0: as ROW, 1: as COL*/
	eigenValues = pcaset.eigenvalues.t();
	eigenVector = pcaset.eigenvectors.t();
}

void getFileList(string path, std::vector<string> &fileList, std::vector<string> &folder)
{
	intptr_t hFile = 0; //文件句柄  64位下long 改为 intptr_t
	struct _finddata_t fileinfo;
	string p;

	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib & _A_SUBDIR)) {
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					string f = path + "\\" + fileinfo.name + "\\model.obj";
					if ((_access(f.c_str(), 0)) != -1) //file exist?
					{
						cout << f << endl;
						fileList.push_back(f);
						cout << fileinfo.name << endl;
						folder.push_back(fileinfo.name);
					}	
				}
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

int printPcaEigenValues(Mat eValues, int div)
{
	float sum = 0.f;

	for (int i = 0; i < eValues.cols; i++)
	{
		sum += eValues.at<float>(0, i);
	}
	cout << "sum:" << sum << endl;

	int interval = eValues.cols / div;
	for (int num = 1; num <= (eValues.cols / interval); num++)
	{
		float percent = 0.f;
		int frontPart = num * interval;
		for (int i = 0; i < frontPart; i++)
		{
			percent += eValues.at<float>(0, i);
		}
		percent = percent / sum;
		cout << "front " << frontPart << " percent is " << percent << endl;
		/*if (percent >= 1.0)
			return frontPart;*/
	}

	return eValues.cols;
}

void main(void)
{
	string dataBase = "C:\\Users\\Administrator\\Desktop\\50\\female";
	std::vector<string> fileList;
	std::vector<string> folder;
	getFileList(dataBase, fileList, folder);
	cout << fileList.size() << endl;

	Mat eigenValues(1, fileList.size(), CV_32F);
	Mat eigenVector(PCA_ROWS * 3, fileList.size(), CV_32F);
	Mat meanShape(PCA_ROWS * 3, 1, CV_32F);
	
	calculatePCA(eigenValues, eigenVector, meanShape, fileList);

#if (FILE_SAVE_MODE == OPENCV_MODE)
	FileStorage pcaWrite(dataBase + "\\" + "PCA.yml", FileStorage::WRITE);
	pcaWrite << "eigenValues" << eigenValues <<
	            "eigenVector" << eigenVector <<
	            "meanShape"   << meanShape;
	pcaWrite.release();
#else
	save2Txt("eigenValues", dataBase + "\\" + "eigenValues.txt", eigenValues);
	save2Txt("eigenVector", dataBase + "\\" + "eigenVector.txt", eigenVector);
	save2Txt("meanShape", dataBase + "\\" + "meanShape.txt", meanShape);
#endif
	//验证PCA结果
#if (FILE_SAVE_MODE == OPENCV_MODE)
	FileStorage pcaRead(dataBase + "\\" + "PCA.yml", FileStorage::READ);
	pcaRead["eigenValues"] >> eigenValues;
	pcaRead["eigenVector"] >> eigenVector;
	pcaRead["meanShape"] >> meanShape;
	pcaRead.release();
#else
	readFromTxt("eigenValues", dataBase + "\\" + "eigenValues.txt", eigenValues);
	readFromTxt("eigenVector", dataBase + "\\" + "eigenVector.txt", eigenVector);
	readFromTxt("meanShape", dataBase + "\\" + "meanShape.txt", meanShape);
#endif
	int frontPart = printPcaEigenValues(eigenValues, 10);
	Mat eigenVectorPart = eigenVector.colRange(0, frontPart).clone();

	for (int i = 0; i < fileList.size(); i++)
	{
		string input = fileList[i];
		cout << "load obj: " << input << " for test..." << endl;

		Mat testData(PCA_ROWS * 3, 1, CV_32F);
		objData *testObj = loadObj(input);
		for (int row = 0; row < PCA_ROWS; row++)
		{
			testData.at<float>(row * 3 + 0, 0) = testObj->v[row][0];
			testData.at<float>(row * 3 + 1, 0) = testObj->v[row][1];
			testData.at<float>(row * 3 + 2, 0) = testObj->v[row][2];
		}
		delete testObj;

		//Ax = B;
		Eigen::Matrix<float, Eigen::Dynamic, 1> B;
		cv2eigen(testData - meanShape, B);

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A;
		cv2eigen(eigenVectorPart, A);

		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x;
		x = A.fullPivHouseholderQr().solve(B);

		Mat alpha;
		eigen2cv(x, alpha);

		Mat target = eigenVectorPart * alpha;
		target = target + meanShape;
		createPly(dataBase + "\\" + "testData" + folder[i] + ".ply", target);

		//output alpha yml file
#if (FILE_SAVE_MODE == OPENCV_MODE)
		FileStorage alphaWrite(dataBase + "\\" + "alpha.yml", FileStorage::WRITE);
		alphaWrite << "alpha" << alpha;
		alphaWrite.release();
#else
		save2Txt("alpha", dataBase + "\\" + "alpha" + folder[i] + ".txt", alpha);
#endif
	}
}

#endif

```

