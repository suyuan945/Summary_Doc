##### 1.像素访问

opencv 中的像素访问主要有三种方式：下标访问，指针访问，迭代器访问。

```
//下标方式
void colorReduce(Mat& inputImage, Mat& outputImage, int div)  
{  
	//参数准备
	outputImage = inputImage.clone();  //拷贝实参到临时变量
	int rowNumber = outputImage.rows;  //行数
	int colNumber = outputImage.cols;  //列数

	//存取彩色图像像素
	for(int i = 0;i < rowNumber;i++)  
	{  
		for(int j = 0;j < colNumber;j++)  
		{  	
			// ------------------------【开始处理每个像素】--------------------
			outputImage.at<Vec3b>(i,j)[0] =  outputImage.at<Vec3b>(i,j)[0]/div*div + div/2;  //蓝色通道
			outputImage.at<Vec3b>(i,j)[1] =  outputImage.at<Vec3b>(i,j)[1]/div*div + div/2;  //绿色通道
			outputImage.at<Vec3b>(i,j)[2] =  outputImage.at<Vec3b>(i,j)[2]/div*div + div/2;  //红是通道
			// -------------------------【处理结束】----------------------------
		}  // 行处理结束     
	}  
}  
```

```
//指针方式
void colorReduce(Mat& inputImage, Mat& outputImage, int div)  
{  
	//参数准备
	outputImage = inputImage.clone();  //拷贝实参到临时变量
	int rowNumber = outputImage.rows;  //行数
	int colNumber = outputImage.cols*outputImage.channels();  //列数 x 通道数=每一行元素的个数

	//双重循环，遍历所有的像素值
	for(int i = 0;i < rowNumber;i++)  //行循环
	{  
		uchar* data = outputImage.ptr<uchar>(i);  //获取第i行的首地址
		for(int j = 0;j < colNumber;j++)   //列循环
		{  	
			// ---------【开始处理每个像素】-------------     
			data[j] = data[j]/div*div + div/2;  
			// ----------【处理结束】---------------------
		}  //行处理结束
	}  
}  
```

```
//迭代器方式
void colorReduce(Mat& inputImage, Mat& outputImage, int div)  
{  
	//参数准备
	outputImage = inputImage.clone();  //拷贝实参到临时变量
	//获取迭代器
	Mat_<Vec3b>::iterator it = outputImage.begin<Vec3b>();  //初始位置的迭代器
	Mat_<Vec3b>::iterator itend = outputImage.end<Vec3b>();  //终止位置的迭代器

	//存取彩色图像像素
	for(;it != itend;++it)  
	{  
		// ------------------------【开始处理每个像素】--------------------
		(*it)[0] = (*it)[0]/div*div + div/2;  
		(*it)[1] = (*it)[1]/div*div + div/2;  
		(*it)[2] = (*it)[2]/div*div + div/2;  
		// ------------------------【处理结束】----------------------------
	}  
```

##### 2.颜色通道的分离与融合

在图像处理时，我们接触到的彩色以RGB居多，为了分析图像在某一通道上的特性，需要将图像的颜色通道进行分离，或者是在对某一颜色通道处理后重新进行融合。opencv提供了split()函数来进行颜色通道的分离，提供了merge()函数来进行颜色通道的融合。

```
//多通道的融合示例
bool  MultiChannelBlending()
{
	//【0】定义相关变量
	Mat srcImage;
	Mat logoImage;
	vector<Mat> channels;
	Mat  imageBlueChannel;

	//=================【蓝色通道部分】=================
	//	描述：多通道混合-蓝色分量部分
	//============================================

	// 【1】读入图片
	logoImage= imread("dota_logo.jpg",0);
	srcImage= imread("dota_jugg.jpg");

	if( !logoImage.data ) { printf("Oh，no，读取logoImage错误~！ \n"); return false; }
	if( !srcImage.data ) { printf("Oh，no，读取srcImage错误~！ \n"); return false; }

	//【2】把一个3通道图像转换成3个单通道图像
	split(srcImage,channels);//分离色彩通道

	//【3】将原图的蓝色通道引用返回给imageBlueChannel，注意是引用，相当于两者等价，修改其中一个另一个跟着变
	imageBlueChannel= channels.at(0);
	//【4】将原图的蓝色通道的（500,250）坐标处右下方的一块区域和logo图进行加权操作，将得到的混合结果存到imageBlueChannel中
	addWeighted(imageBlueChannel(Rect(500,250,logoImage.cols,logoImage.rows)),1.0,
		logoImage,0.5,0,imageBlueChannel(Rect(500,250,logoImage.cols,logoImage.rows)));

	//【5】将三个单通道重新合并成一个三通道
	merge(channels,srcImage);

	//【6】显示效果图
	namedWindow(" <1>游戏原画+logo蓝色通道");
	imshow(" <1>游戏原画+logo蓝色通道",srcImage);


	//=================【绿色通道部分】=================
	//	描述：多通道混合-绿色分量部分
	//============================================

	//【0】定义相关变量
	Mat  imageGreenChannel;

	//【1】重新读入图片
	logoImage= imread("dota_logo.jpg",0);
	srcImage= imread("dota_jugg.jpg");

	if( !logoImage.data ) { printf("读取logoImage错误~！ \n"); return false; }
	if( !srcImage.data ) { printf("读取srcImage错误~！ \n"); return false; }

	//【2】将一个三通道图像转换成三个单通道图像
	split(srcImage,channels);//分离色彩通道

	//【3】将原图的绿色通道的引用返回给imageBlueChannel，注意是引用，相当于两者等价，修改其中一个另一个跟着变
	imageGreenChannel= channels.at(1);
	//【4】将原图的绿色通道的（500,250）坐标处右下方的一块区域和logo图进行加权操作，将得到的混合结果存到imageGreenChannel中
	addWeighted(imageGreenChannel(Rect(500,250,logoImage.cols,logoImage.rows)),1.0,
		logoImage,0.5,0.,imageGreenChannel(Rect(500,250,logoImage.cols,logoImage.rows)));

	//【5】将三个独立的单通道重新合并成一个三通道
	merge(channels,srcImage);

	//【6】显示效果图
	namedWindow("<2>游戏原画+logo绿色通道");
	imshow("<2>游戏原画+logo绿色通道",srcImage);



	//=================【红色通道部分】=================
	//	描述：多通道混合-红色分量部分
	//============================================

	//【0】定义相关变量
	Mat  imageRedChannel;

	//【1】重新读入图片
	logoImage= imread("dota_logo.jpg",0);
	srcImage= imread("dota_jugg.jpg");

	if( !logoImage.data ) { printf("Oh，no，读取logoImage错误~！ \n"); return false; }
	if( !srcImage.data ) { printf("Oh，no，读取srcImage错误~！ \n"); return false; }

	//【2】将一个三通道图像转换成三个单通道图像
	split(srcImage,channels);//分离色彩通道

	//【3】将原图的红色通道引用返回给imageBlueChannel，注意是引用，相当于两者等价，修改其中一个另一个跟着变
	imageRedChannel= channels.at(2);
	//【4】将原图的红色通道的（500,250）坐标处右下方的一块区域和logo图进行加权操作，将得到的混合结果存到imageRedChannel中
	addWeighted(imageRedChannel(Rect(500,250,logoImage.cols,logoImage.rows)),1.0,
		logoImage,0.5,0.,imageRedChannel(Rect(500,250,logoImage.cols,logoImage.rows)));

	//【5】将三个独立的单通道重新合并成一个三通道
	merge(channels,srcImage);

	//【6】显示效果图
	namedWindow("<3>游戏原画+logo红色通道 ");
	imshow("<3>游戏原画+logo红色通道 ",srcImage);

	return true;
}
```

![原图1](./data/dota_jugg.jpg#pic_center)

<center>原图</center>

![logo](./data/dota_logo.jpg#pic_center)

<center>logo图</center>

<img src=".\data\src_blue.png#pic_center" alt="蓝色通道融合" style="zoom:100%;" />

<center>效果图</center>