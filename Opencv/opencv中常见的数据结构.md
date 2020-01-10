### Opencv 中常见的数据结构

##### 1. Mat类

Mat类是用于读取和保存图像以及其他矩阵数据的数据结构，默认情况下其尺寸为0.

###### 1.图像的载入和显示

```
//  图像读取
//第一个为图像路径，第二个是指加载图像的颜色类型，默认为1
Mat img = imread("left_0.png",1);
```

颜色类型的选取：>0,   返回一个3通道的彩色图像

​                               =0，返回灰度图像

​                               <0,   返回包含Alpha通道的加载图像

```
//  图像显示
imshow("窗口名字", img);
```

```
//  图像保存
imwrite("rectify_left.png", img);
```

###### 2. Mat初始化方法

Mat的初始化方法主要有以下几种：

```
//使用Mat()
Mat M(2,2, CV_8UC3, Scalar(0,0,255));
Mat r = Mat(10,3,CV_8UC3);
//CV_8UC3指的是：CV_ [每一项的位数] [有符号或无符号] [类型前缀] C [通道数]
```

```
//使用Create()函数
M.create(4,4, CV_8UC(2));
```

```
//MATLAB风格初始化，zeros()，ones()，eyes()
//指定使用的尺寸和数据类型
Mat E = Mat::eye(4, 4, CV_64F)；
cout << "E = " << endl << " " << E << endl << endl;
Mat O = Mat::ones(2, 2, CV_32F);
cout << "O = " << endl << " " << O << endl << endl;
Mat Z = Mat::zeros(3,3, CV_8UC1);
cout << "Z = " << endl << " " << Z << endl << endl;
```

```
//使用已有的Mat对象创建
Mat RowClone = C.row(1).clone();
cout << "RowClone = " << endl << " " << RowClone << endl << endl;
```

```
//自定义
Mat array = (Mat_<double>(3, 3) << 0, -1, 5, -1, 5, -1, 0, -1, 0);
```

###### 3. Mat的拷贝

```
Mat img;
Mat M;
img.copyTo(M);
//
M = img.clone();
```

##### 2.常用的数据结构和函数

###### 1.点的表示：Point类

注意 Point_<int>, Point2i、Point互相等价

```
Point point;
point.x = 0;
point.y = 0;
//
Point point = Point(0,0);
```

###### 2. Scalar类：颜色的表示

Scalar()是一个具有四个元素的数组，在OpenCV中被大量用于传递像素值，比如RGB颜色。但RGB颜色值为三个参数，对于Scalar函数来说，如果用不到第四个参数，则不需要写出来。对于下面这行代码：

```
Scalar(a, b, c);
```

那么定义RGB的颜色值，红色的分量为c，绿色的分量为b，蓝色的分量为a。
 需要注意的一点是，Scalar的源头是Scalar_类，而Scalar_类是Vec4x的一个变种，我们常用的Scalar类其实就是Scalar_。这也就是为什么很多函数的参数输入可以是Mat，也可以是Scalar。

###### 3. Size类：尺寸的表示

```
//Size_(_Tp _width, _Tp_height);
//构造出的Size的宽度和高度都是5
Size(5,5);
cv::Size sz;　　//空构造
cv::Size2i sz;　　//2位int型
cv::Size2f sz;　　//2位float型
cv::Size sz2(sz1);　　//将sz1拷贝给sz2
cv::Size2f sz(w,h);　　//赋值构造，w和h分别对应属性width和height
sz.width;　　//取宽度　　
sz.height;　　//取高度
```

###### 4. Rect类：矩形的表示

Rect类的成员变量有x、y、width、height，分别为左上角点的坐标和矩形的宽和高

常用的成员函数有size()返回值为Size；area()返回的是矩形的面积；contains(Point)判断点是否在矩形内；t1()返回的是左上角点坐标；br()返回的是右下角的坐标。

```
Rect rect(x、y、width、height);//初始化
Rect rect =rect1 & rect2; //求两个矩阵的交集
Rect rect=rect1 | rect2; //求两个矩阵的并集
Rect rectShift =rect + point; //让矩形进行平移操作
Rect rectScale = rect +size; //让矩形进行缩放操作
```

###### 5.  cvtColor() 函数:颜色空间的转换

RGB颜色向HSV、HSI等颜色空间的转换，也可以转化为灰度图像

```
//void cvtColor(InputArray src,OutputArray dst, int code, int dstCn=0);
cvtColor(img1,img2, COLOR_GRAY2BGR);
```

##### 3.其他常见的数据项

```
//定义二维点
Point2f p(6, 2);
cout << "【2维点】p = " << p << ";\n" << endl;
//定义三维点
Point3f p3f(8, 2, 0);
cout << "【3维点】p3f = " << p3f << ";\n" << endl;
```

```
//vector与Mat的转换关系
vector<float> v;
v.push_back(3);
v.push_back(5);
v.push_back(7);

cout << "【基于Mat的vector】shortvec = " << Mat(v) << ";\n"<<endl;

vector<Point2f> points(20);
for (size_t i = 0; i < points.size(); ++i)
points[i] = Point2f((float)(i * 5), (float)(i % 7));

cout << "【二维点向量】points = " << points<<";";
```

##### 4.基本图形的绘制

```
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;


#define WINDOW_NAME1 "【绘制图1】"        //为窗口标题定义的宏 
#define WINDOW_NAME2 "【绘制图2】"        //为窗口标题定义的宏 
#define WINDOW_WIDTH 600//定义窗口大小的宏


void DrawEllipse( Mat img, double angle );//绘制椭圆
void DrawFilledCircle( Mat img, Point center );//绘制圆
void DrawPolygon( Mat img );//绘制多边形
void DrawLine( Mat img, Point start, Point end );//绘制线段

int main( void )
{

	// 创建空白的Mat图像
	Mat atomImage = Mat::zeros( WINDOW_WIDTH, WINDOW_WIDTH, CV_8UC3 );
	Mat rookImage = Mat::zeros( WINDOW_WIDTH, WINDOW_WIDTH, CV_8UC3 );

	// ---------------------<1>绘制化学中的原子示例图------------------------

	//【1.1】先绘制出椭圆
	DrawEllipse( atomImage, 90 );
	DrawEllipse( atomImage, 0 );
	DrawEllipse( atomImage, 45 );
	DrawEllipse( atomImage, -45 );

	//【1.2】再绘制圆心
	DrawFilledCircle( atomImage, Point( WINDOW_WIDTH/2, WINDOW_WIDTH/2) );

	// ----------------------------<2>绘制组合图-----------------------------
	//【2.1】先绘制出椭圆
	DrawPolygon( rookImage );

	// 【2.2】绘制矩形
	rectangle( rookImage,
		Point( 0, 7*WINDOW_WIDTH/8 ),
		Point( WINDOW_WIDTH, WINDOW_WIDTH),
		Scalar( 0, 255, 255 ),
		-1,
		8 );

	// 【2.3】绘制一些线段
	DrawLine( rookImage, Point( 0, 15*WINDOW_WIDTH/16 ), Point( WINDOW_WIDTH, 15*WINDOW_WIDTH/16 ) );
	DrawLine( rookImage, Point( WINDOW_WIDTH/4, 7*WINDOW_WIDTH/8 ), Point( WINDOW_WIDTH/4, WINDOW_WIDTH ) );
	DrawLine( rookImage, Point( WINDOW_WIDTH/2, 7*WINDOW_WIDTH/8 ), Point( WINDOW_WIDTH/2, WINDOW_WIDTH ) );
	DrawLine( rookImage, Point( 3*WINDOW_WIDTH/4, 7*WINDOW_WIDTH/8 ), Point( 3*WINDOW_WIDTH/4, WINDOW_WIDTH ) );

	// ---------------------------<3>显示绘制出的图像------------------------
	imshow( WINDOW_NAME1, atomImage );
	moveWindow( WINDOW_NAME1, 0, 200 );
	imshow( WINDOW_NAME2, rookImage );
	moveWindow( WINDOW_NAME2, WINDOW_WIDTH, 200 );

	waitKey( 0 );
	return(0);
}



//-------------------------------【DrawEllipse( )函数】--------------------------------
//		描述：自定义的绘制函数，实现了绘制不同角度、相同尺寸的椭圆
//-----------------------------------------------------------------------------------------
void DrawEllipse( Mat img, double angle )
{
	int thickness = 2;
	int lineType = 8;

	ellipse( img,
		Point( WINDOW_WIDTH/2, WINDOW_WIDTH/2 ),
		Size( WINDOW_WIDTH/4, WINDOW_WIDTH/16 ),
		angle,
		0,
		360,
		Scalar( 255, 129, 0 ),
		thickness,
		lineType );
}


//-----------------------------------【DrawFilledCircle( )函数】---------------------------
//		描述：自定义的绘制函数，实现了实心圆的绘制
//-----------------------------------------------------------------------------------------
void DrawFilledCircle( Mat img, Point center )
{
	int thickness = -1;
	int lineType = 8;

	circle( img,
		center,
		WINDOW_WIDTH/32,
		Scalar( 0, 0, 255 ),
		thickness,
		lineType );
}


//-----------------------------------【DrawPolygon( )函数】--------------------------
//		描述：自定义的绘制函数，实现了凹多边形的绘制
//--------------------------------------------------------------------------------------
void DrawPolygon( Mat img )
{
	int lineType = 8;

	//创建一些点
	Point rookPoints[1][20];
	rookPoints[0][0]  = Point(    WINDOW_WIDTH/4,   7*WINDOW_WIDTH/8 );
	rookPoints[0][1]  = Point(  3*WINDOW_WIDTH/4,   7*WINDOW_WIDTH/8 );
	rookPoints[0][2]  = Point(  3*WINDOW_WIDTH/4,  13*WINDOW_WIDTH/16 );
	rookPoints[0][3]  = Point( 11*WINDOW_WIDTH/16, 13*WINDOW_WIDTH/16 );
	rookPoints[0][4]  = Point( 19*WINDOW_WIDTH/32,  3*WINDOW_WIDTH/8 );
	rookPoints[0][5]  = Point(  3*WINDOW_WIDTH/4,   3*WINDOW_WIDTH/8 );
	rookPoints[0][6]  = Point(  3*WINDOW_WIDTH/4,     WINDOW_WIDTH/8 );
	rookPoints[0][7]  = Point( 26*WINDOW_WIDTH/40,    WINDOW_WIDTH/8 );
	rookPoints[0][8]  = Point( 26*WINDOW_WIDTH/40,    WINDOW_WIDTH/4 );
	rookPoints[0][9]  = Point( 22*WINDOW_WIDTH/40,    WINDOW_WIDTH/4 );
	rookPoints[0][10] = Point( 22*WINDOW_WIDTH/40,    WINDOW_WIDTH/8 );
	rookPoints[0][11] = Point( 18*WINDOW_WIDTH/40,    WINDOW_WIDTH/8 );
	rookPoints[0][12] = Point( 18*WINDOW_WIDTH/40,    WINDOW_WIDTH/4 );
	rookPoints[0][13] = Point( 14*WINDOW_WIDTH/40,    WINDOW_WIDTH/4 );
	rookPoints[0][14] = Point( 14*WINDOW_WIDTH/40,    WINDOW_WIDTH/8 );
	rookPoints[0][15] = Point(    WINDOW_WIDTH/4,     WINDOW_WIDTH/8 );
	rookPoints[0][16] = Point(    WINDOW_WIDTH/4,   3*WINDOW_WIDTH/8 );
	rookPoints[0][17] = Point( 13*WINDOW_WIDTH/32,  3*WINDOW_WIDTH/8 );
	rookPoints[0][18] = Point(  5*WINDOW_WIDTH/16, 13*WINDOW_WIDTH/16 );
	rookPoints[0][19] = Point(    WINDOW_WIDTH/4,  13*WINDOW_WIDTH/16 );

	const Point* ppt[1] = { rookPoints[0] };
	int npt[] = { 20 };

	fillPoly( img,
		ppt,
		npt,
		1,
		Scalar( 255, 255, 255 ),
		lineType );
}


//-----------------------------------【DrawLine( )函数】--------------------------
//		描述：自定义的绘制函数，实现了线的绘制
//---------------------------------------------------------------------------------
void DrawLine( Mat img, Point start, Point end )
{
	int thickness = 2;
	int lineType = 8;
	line( img,
		start,
		end,
		Scalar( 0, 0, 0 ),
		thickness,
		lineType );
}
```

