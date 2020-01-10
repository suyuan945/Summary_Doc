#### PCL中的点云滤波算法

在获取点云数据时，由于设备精度、操作者经验、环境因素等带来的影响，点云数据中将不可避免地出现一些噪声点。

PCL中的点云处理模块提供了很多灵活实用的滤波处理算法，例如双边滤波、高斯滤波、条件滤波、直通滤波、基于随机采样一致性滤波等。

常见的需要对点云滤波处理主要有：

(1)点云数据密度不规则需要平滑；

(2)因为遮挡等问题造成离群点需要去除； 

(3)大量数据需要进行下采样；

(4)噪音数据需要去除。

以下将逐一介绍各种算法的使用，针对原理部分不在此展开阐释。

###### 1.直通滤波

```
//   ***直通滤波器***  保留有效点云
void passThrough_filter_par(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud,
	bool use_z,float z_min,float z_max ,bool use_y ,float y_min ,float y_max ,bool use_x ,float x_min ,float x_max)
{

	// 创建滤波器对象
	//z方向
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passThrough_z(new pcl::PointCloud<pcl::PointXYZ>);
	if (use_z)
	{
		
		pcl::PassThrough<pcl::PointXYZ> pass_z;
		pass_z.setInputCloud(source_cloud);
		pass_z.setFilterFieldName("z");
		pass_z.setFilterLimits(z_min, z_max);
		//pass.setFilterLimitsNegative (true);
		pass_z.filter(*cloud_passThrough_z);
	}
	//y方向
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passThrough_y(new pcl::PointCloud<pcl::PointXYZ>);
	if (use_y)
	{
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passThrough_y(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass_y;
		pass_y.setInputCloud(cloud_passThrough_z);
		pass_y.setFilterFieldName("y");
		pass_y.setFilterLimits(y_min,y_max);
		//pass.setFilterLimitsNegative (true);
		pass_y.filter(*cloud_passThrough_y);

	}
	
	//x方向
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passThrough_x(new pcl::PointCloud<pcl::PointXYZ>);
	if(use_x)
	{
		
		pcl::PassThrough<pcl::PointXYZ> pass_x;
		pass_x.setInputCloud(cloud_passThrough_y);
		pass_x.setFilterFieldName("x");
		pass_x.setFilterLimits(x_min, x_max);
		//pass.setFilterLimitsNegative (true);
		pass_x.filter(*filter_cloud);
	}
	
}
```

###### 2.统计滤波

```
//   ***统计滤波器***  去除离散点云和噪声
void statistical_filter_par(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud,float distance)
{

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outrem;//创建统计滤波对象
	outrem.setInputCloud(source_cloud);
	outrem.setMeanK(distance);//附近邻近点数
	outrem.setStddevMulThresh(1);//判断是否离群点
	outrem.filter(*filter_cloud);

}
```

###### 3.体素滤波

```
//   ***体素滤波器***  去除离散点云和噪声
void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filter_cloud,float lx,float ly, float lz)
{
	// Create the filtering object
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(source_cloud);
	sor.setLeafSize(lx, ly, lz);
	sor.filter(*filter_cloud);

}
```

