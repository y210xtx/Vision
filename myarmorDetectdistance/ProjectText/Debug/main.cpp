////-----------------------------------【视频暂停与画图】----------------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Mydemo.h>
#define HIGH 5.8
#define WIDTH 14.3
#define WINDOW_NAME "测距"        //为窗口标题定义的宏
//进行位置解算
using namespace cv;
using namespace std;

Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
Mat cameraMatrix = Mat::eye(3, 3, CV_64F);


//自定义的物体世界坐标，单位为mm
vector<Point3f> obj = vector<Point3f>{
	cv::Point3f(-WIDTH, -HIGH, 0),	//tl
	cv::Point3f(WIDTH, -HIGH, 0),	//tr
	cv::Point3f(WIDTH, HIGH, 0),	//br
	cv::Point3f(-WIDTH, HIGH, 0)	//bl
};
cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1);//init rvec
cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1);//init tvec


#define WINDOW_NAME "【滑动条的创建】"        //为窗口标题定义的宏 
enum ArmorColor{RED,BLUE};
ArmorColor armorColor = RED;
int value = 80;
int valuemax=255;
int a = 76;
RNG g_rng(12345);//随机数
//		描述：全局函数的声明
//------------------------------------------------------------------------------------------------

//声明存储图像的变量
Mat g_srcImage1, tempImage;
Mat frameCalibration;
int detect(Mat& src);
void callback(int,void*);
int main(int argc, char** argv)
{
	//VideoCapture capture("炮台素材蓝车旋转-ev--3.MOV");
	VideoCapture capture(1);

	Mydemo mydemo;

	capture >> g_srcImage1;
	//创建窗体
	namedWindow(WINDOW_NAME, 1);
	//【0】改变console字体颜色
	system("color 9F");

	cameraMatrix.at<double>(0, 0) = 5.705146448781369e+02;
	cameraMatrix.at<double>(0, 1) = -0.332617873514110;
	cameraMatrix.at<double>(0, 2) = 3.135890274081354e+02;
	cameraMatrix.at<double>(1, 1) = 5.707809815499222e+02;
	cameraMatrix.at<double>(1, 2) = 2.798646537851680e+02;


	distCoeffs.at<double>(0, 0) = 0.035553102905935;
	distCoeffs.at<double>(1, 0) = -0.063483045359161;
	distCoeffs.at<double>(2, 0) = 0.002130716412763;
	distCoeffs.at<double>(3, 0) = 3.172591045820162e-04;
	distCoeffs.at<double>(4, 0) = 0;
	Mat view, rview, map1, map2;
	Mat tempImage;
	Size imageSize;
	imageSize = g_srcImage1.size();
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, map1, map2);

	//TickMeter tm;//不好用
	//tm.start();

	//设置帧率
	double setframeRate = 1000;
	int sleep = 1;

	//计算时间
	static long startcount = getTickCount();
	long currentcount = 0;
	//显示帧率
	double Time = 0;
	double frameNum = 0;
	double framRate = 0;

	char frame[50];
	char time[50];
	//sprintf_s(frame, "FPS: %d", framRate);
	//putText(g_srcImage1, frame, Point(0,10),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,255),1,8);


	while (1)
	{
		capture >> g_srcImage1;
		remap(g_srcImage1, frameCalibration, map1, map2, INTER_LINEAR);//给出目标到原的映射关系
		frameNum++;
		sprintf_s(frame, "FPS: %f", framRate);
		putText(frameCalibration, frame, Point(0, 20), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 255), 1, 8);
		detect(frameCalibration);
		imshow(WINDOW_NAME, frameCalibration);
		if (waitKey(sleep) >= 0) {
			while (waitKey(1) == -1)
			{
				//g_srcImage1.copyTo(tempImage);//拷贝源图到临时变量
				createTrackbar("阈值", WINDOW_NAME, &value, valuemax, callback);
				//imshow(WINDOW_NAME, tempImage);
			}
		}
		currentcount = getTickCount();
		Time = (currentcount - startcount) / getTickFrequency();
		framRate = frameNum / Time;
		//sprintf_s(time, "time: %d", tm.getTimeSec());
		putText(frameCalibration, time, Point(100, 20), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 255), 1, 8);
	}
	return 0;
}
int detect(Mat& src) {
	Mat  gray,binImage;
	vector<Mat> channels;
	split(src, channels);
	if (armorColor == 1)
		gray = channels.at(2) - channels.at(0);
	else
		gray = channels.at(0) - channels.at(2);
	threshold(gray, binImage, a, 255, THRESH_BINARY);
	Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	//膨胀
	dilate(binImage, binImage, element);
	//找轮廓
	vector<vector<Point>>contours;
	vector<RotatedRect>RoteRect;
	findContours(binImage.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);//绘制最外层轮廓，并且只保留每个方向的终点元素
	//遍历轮廓
	
	for (const auto& contour : contours) {
		float Area = contourArea(contour);
		if (contour.size() < 5 || Area < 210)continue;//5和210改
		//椭圆拟合得到外接矩阵
		RotatedRect Rec = fitEllipse(contour);//始终以较长的方向为高
		//矫正灯条  
		while (Rec.angle > 90)Rec.angle -= 180;
		if (Rec.angle >= 45.0)
		{
			swap(Rec.size.width, Rec.size.height);
			Rec.angle -= 90.0;
		}
		else if (Rec.angle < -45.0)
		{
			swap(Rec.size.width, Rec.size.height);
			Rec.angle += 90.0;
		}
		if (Rec.size.width / Rec.size.height > 1  || Area / Rec.size.area() < 0.5 || Rec.size.height/Rec.size.width>8)continue;
		//Rect linghtR = Rec.boundingRect();
		//const Rect srcBound(Point(0, 0), g_srcImage1.size());
		//linghtR &= srcBound;

		RoteRect.push_back(Rec);
	}
	if (RoteRect.empty()) {
		return 0;
	}
	//drawContours(g_srcImage1, con, -1, Scalar(0, 0, 255), 2, 8);//-1为绘制所有轮廓
	for (int i = 0; i < RoteRect.size();i++) {
		Point2f vertex[4];
		RoteRect[i].points(vertex);
		for (int i = 0; i < 4; i++) {
			line(frameCalibration, vertex[i], vertex[(i + 1) % 4], Scalar(255, 0, 0), 2);
		}
	}
	/*sort(RoteRect.begin(), RoteRect.end(), [](const RotatedRect& Image1, const RotatedRect& Image2)
		{
			return Image1.center.x < Image2.center.y;
		});*/
	for (size_t i = 0; i < RoteRect.size(); i++)
	{//遍历所有灯条进行匹配
		for (size_t j = i + 1; (j < RoteRect.size()); j++)
		{
			const RotatedRect& leftLight = RoteRect[i];
			const RotatedRect& rightLight = RoteRect[j];
			//角差
			float angleDiff_ = abs(leftLight.angle - rightLight.angle);
			float angleDiff_2 = leftLight.angle - rightLight.angle;

			//长度差比率
			float LenDiff_ratio = abs(leftLight.size.height - rightLight.size.height) / max(leftLight.size.height, rightLight.size.height);
			//筛选
			if (angleDiff_ > 7.0 ||					//角度差大于7.0
				LenDiff_ratio > 0.2||
				angleDiff_2>0.3)				//高度比率0.2
			{
				continue;
			}
			//cout << angleDiff_2<<endl;
			
			//左右灯条相距距离
			//float dis = cvex::distance(leftLight.center, rightLight.center);
			//左右灯条长度的平均值
			float meanLen = (leftLight.size.height + rightLight.size.height) / 2;
			//左右灯条中心点y的差值
			float yDiff = abs(leftLight.center.y - rightLight.center.y);
			//y差比率
			float yDiff_ratio = yDiff / meanLen;
			//左右灯条中心点x的差值
			float xDiff = abs(leftLight.center.x - rightLight.center.x);
			//x差比率
			float xDiff_ratio = xDiff / meanLen;
			//相距距离与灯条长度比值
			//float ratio = dis / meanLen;
			//筛选
			if (
				yDiff_ratio > 0.5 
				||xDiff_ratio >5
				//ratio > _param.armor_max_aspect_ratio_ ||
				//ratio < _param.armor_min_aspect_ratio_
				)
			{
				continue;
			}
			
			//将装甲板框出来
			//Rect rect1 = leftLight.boundingRect();
			//Rect rect2 = rightLight.boundingRect();
			//rectangle(g_srcImage1, rect1.tl(), rect2.br(), Scalar(255, 0, 255),2);
			Point2f vectex1[4];
			Point2f vectex2[4];
			vector<Point2f>points;
			leftLight.points(vectex1);
			rightLight.points(vectex2);
			for (int i = 0; i < 4; i++) {
				points.push_back(vectex1[i]);
			}
			for (int i = 0; i < 4; i++) {
				points.push_back(vectex2[i]);
			}
			RotatedRect Rrect = minAreaRect(points);
			Rect rect = boundingRect(points);
			rectangle(frameCalibration, rect, Scalar(255, 0, 255), 2);

			/*Point point2(rect.tl().x+rect.width,rect.tl().y+rect.height);
			Point point1(rect.tl().x,rect.tl().y+rect.height);
			
			Point point3(rect.tl().x+rect.width,rect.tl().y);
			Point point4(rect.tl());*/
			vector<Point2d> Dpoints;
			Point2f bond[4];
			Rrect.points(bond);
			Dpoints.push_back(bond[0]);
			Dpoints.push_back(bond[1]);
			Dpoints.push_back(bond[2]);
			Dpoints.push_back(bond[3]);


			solvePnP(obj, Dpoints, cameraMatrix, distCoeffs, rVec, tVec, false, SOLVEPNP_ITERATIVE);
			//输出平移向量
			cout << "tvec: " << tVec << endl;
		}
	}
}
void callback(int ,void*) {
	a = value;
	Mat  gray, binImage;
	vector<Mat> channels;
	split(g_srcImage1, channels);
	if (armorColor == 1)
		gray = channels.at(2) - channels.at(0);
	else
		gray = channels.at(0) - channels.at(2);
	threshold(gray, binImage, a, 255, THRESH_BINARY);
	Mat sizelow = Mat::zeros(512, 512, CV_8UC3);
	resize(binImage, sizelow,sizelow.size());
	imshow("output", sizelow);
}

//问题，灯条长度相差很大，也会进行匹配
//内八问题
//解决：在已经匹配上了的灯条上加角度差