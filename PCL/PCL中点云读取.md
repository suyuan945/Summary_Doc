#### 点云数据的读取

##### 1.PCl中点云的数据格式

```
//可用于可视化
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//不可用于可视化
pcl::PointCloud<pcl::PointXYZ> cloud;
//点云的填充
cloud.width  = 15;
cloud.height = 1;
cloud.points.resize (cloud.width * cloud.height);
//生成数据
for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }
```

##### 2.PCD点云的读取和保存

```
//读取PCD文件并可视化
void read_pcd(string pcd_path)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
		PCL_ERROR("Couldn't read file rabbit.pcd\n");
		//return(-1);
	}
	std::cout << cloud->points.size() << std::endl;
    //可视化
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
}

void read_pcd_file(string pcd_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
		PCL_ERROR("Couldn't read file rabbit.pcd\n");
		//return(-1);
	}
	std::cout << cloud->points.size() << std::endl;

}
```

```
//保存PCD文件
pcl::io::savePCDFileASCII(save_pcd_path, *cloud);
```

##### 3.PLY点云的读取和保存

```
//读取ply文件并可视化
void read_ply(string ply_path)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *cloud) == -1) //* load the file 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		system("PAUSE");
	}
	//pcl::StatisticalOutlierRemoval::applyFileter()
	pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(cloud);

	system("PAUSE");
}

void read_ply_file(string ply_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *cloud) == -1) //* load the file 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		system("PAUSE");
	}
	
}
```

```
//保存PLY
pcl::io::savePLYFile(save_ply_path, *cloud);
```

4.PTS点云的读取及转换

```
//读取pts文件并保存为pcd
void read_pts(string pts_path, string save_pcd_path,string save_ply_path)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ> cloud;


	//ifstream pts_file(pts_path);
	//string line;
	int point_count = 0;
	//统计行数
	//while (!pts_file.eof())
	//{
	//	getline(pts_file, line);
	//	if (strlen(line.c_str()) > 0)
	//	{
	//		istringstream iss(line);
	//		string x, y, z;
	//		iss >> x >> y >> z;
	//		/*cloud.points[point_count].x = stringToFloat(x);
	//		cloud.points[point_count].y = stringToFloat(y);
	//		cloud.points[point_count].z = stringToFloat(z);*/
	//		point_count++;
	//	}
	//}
	//pts_file.close();
	// Fill in the cloud data
	/*cloud.width = 15040896;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);*/
	//test
	cloud->width = 14957495;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);


	ifstream pts_file(pts_path);
	string line;
	//统计行数
	while (!pts_file.eof())
	{
		getline(pts_file, line);
		if (strlen(line.c_str()) > 0)
		{
			istringstream iss(line);
			string x, y, z;
			iss >> x >> y >> z;
			/*cloud.points[point_count].x = stringToFloat(x);
			cloud.points[point_count].y = stringToFloat(y);
			cloud.points[point_count].z = stringToFloat(z);*/

			cloud->points[point_count].x = stringToFloat(x);
			cloud->points[point_count].y = stringToFloat(y);
			cloud->points[point_count].z = stringToFloat(z);
			point_count++;
		}
	}
	pts_file.close();




	//ifstream pt(pts_path);
	//string pt_line;
	//while (!pt.eof())
	//{
	//	getline(pt, pt_line);
	//	if (strlen(pt_line.c_str()) > 0)
	//	{
	//		istringstream iss(pt_line);
	//		string x, y, z;
	//		iss >> x >> y >> z;
	//		/*cloud.points[point_count].x = stringToFloat(x);
	//		cloud.points[point_count].y = stringToFloat(y);
	//		cloud.points[point_count].z = stringToFloat(z);*/

	//		cloud->points[point_count].x = float(atof(x.c_str()));
	//		cloud->points[point_count].y = float(atof(y.c_str()));
	//		cloud->points[point_count].z = float(atof(z.c_str()));

	//		/*cloud.points[point_count].x = float(atof(x.c_str()));
	//		cloud.points[point_count].y = float(atof(y.c_str()));
	//		cloud.points[point_count].z = float(atof(z.c_str()));*/
	//	}

	//}
	//pt.close();

	pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(cloud);
    system("PAUSE");
	//保存PCD和PLY文件
	pcl::io::savePCDFileASCII(save_pcd_path, *cloud);
	pcl::io::savePLYFile(save_ply_path, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to test_pcd.pcd." << std::endl;

}
```

