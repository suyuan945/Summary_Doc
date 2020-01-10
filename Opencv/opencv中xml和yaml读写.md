##### 1.YAML和XML的读操作

```
//yaml的写入
FileStorage fs("test.yaml", FileStorage::WRITE);  
fs << "frameCount" << 5;  
Mat cameraMatrix = (Mat_<double>(3,3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);  Mat distCoeffs = (Mat_<double>(5,1) << 0.1, 0.01, -0.001, 0, 0);  
fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;  
fs.release();  
```

##### 1.YAML和XML的写操作

```
FileStorage fs2("test.yaml", FileStorage::READ);  
// 第一种方法，对FileNode操作
int frameCount = (int)fs2["frameCount"];  
// 第二种方法，使用FileNode运算符> > 
Mat cameraMatrix2, distCoeffs2;  
fs2["cameraMatrix"] >> cameraMatrix2;  
fs2["distCoeffs"] >> distCoeffs2;   
```