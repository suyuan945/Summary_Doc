```
# _*_ coding:utf-8 _*_

import numpy as np
import cv2
import dlib

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

# cv2读取图像
img = cv2.imread("D:\\data_compare\\guang\\2.JPG")
#img = cv2.imread("G:\\Test\\data_test\\unre3D1031-xr-song\\image\\img_0037.png")
cv2.imshow("1",img)
#cv2.imshow("1",img)cv2.flip(img,img,1)
#cv2.imshow("2",img)
#取灰度
img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

# 人脸数rects
rects = detector(img_gray, 0)
for i in range(len(rects)):
    landmarks = np.matrix([[p.x, p.y] for p in predictor(img,rects[i]).parts()])
    for idx, point in enumerate(landmarks):
        # 68点的坐标
        pos = (point[0, 0], point[0, 1])
        for j  in range(0,len(point)):
        	#print(print(j))
        	fileObject = open('landmarks.txt', 'a')
            #fileObject = open('G:\\8Dlib\\1\\IMG_7396_landmarkpoints.txt', 'a')
        	fileObject.write(str(idx)+" "+str(point[0, 0])+" " +str(point[0, 1])+'\n')
        	#fileObject.write('\n')  

        print(idx,pos)

        # 利用cv2.circle给每个特征点画一个圈，共68个
        cv2.circle(img, pos, 2,(0, 0, 255),1)
        # 利用cv2.putText输出1-68
        #font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(img, str(idx+1), pos, font, 0.3, (0, 255, 255), 1,cv2.LINE_AA)
fileObject.close()
cv2.namedWindow("img", 2)
cv2.imshow("img", img)
cv2.imwrite("G:\\8Dlib\\new9.jpg",img)
#cv2.imwrite("3uv.jpg",img)
cv2.waitKey(0)
```

