#include "opencv2/opencv.hpp"
#include <iostream>

#define HEIGH 5.5
#define WIDTH 3.2
#define WINDOW_NAME "���"        //Ϊ���ڱ��ⶨ��ĺ�
//����λ�ý���
using namespace cv;
using namespace std;

Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

//�Զ���������������꣬��λΪmm
vector<Point3f> obj = vector<Point3f>{
    cv::Point3f(-WIDTH, -HEIGH, 0),	//tl
    cv::Point3f(WIDTH, -HEIGH, 0),	//tr
    cv::Point3f(WIDTH, HEIGH, 0),	//br
    cv::Point3f(-WIDTH, HEIGH, 0)	//bl
};
cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1);//init rvec
cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1);//init tvec
Mat ROI;
RNG g_rng(12345);//�����
void on_MouseHandle(int event, int x, int y, int flags, void* param);
void DrawRectangle(cv::Mat& img, cv::Rect box);
void calAngle(Mat cam, Mat dis, Rect in);
Rect g_rectangle;
bool g_bDrawingBox = false;//�Ƿ���л���
int main()
{

    VideoCapture inputVideo(1);
    if (!inputVideo.isOpened())
    {
        cout << "Could not open the input video: " << endl;
        return -1;
    }
    Mat frame;
    Mat frameCalibration;

    inputVideo >> frame;


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
    imageSize = frame.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map1, map2);



    //��������
    namedWindow(WINDOW_NAME, 1);
    //��2�������������ص�����
    setMouseCallback(WINDOW_NAME, on_MouseHandle, (void*)&frameCalibration);

    while (1) //Show the image captured in the window and repeat
    {
        inputVideo >> frame;              // read
        //if (frame.empty()) break;         // check if at end      ����
        remap(frame, frameCalibration, map1, map2, INTER_LINEAR);//����Ŀ�굽ԭ��ӳ���ϵ
        imshow(WINDOW_NAME, frameCalibration);
        if (waitKey(1) > 1) {
            while (waitKey(30) == -1) {
                frameCalibration.copyTo(tempImage);//����Դͼ����ʱ����
                DrawRectangle(tempImage, g_rectangle);
                imshow(WINDOW_NAME, tempImage);

            }
        }
    }
    return 0;
}

void on_MouseHandle(int event, int x, int y, int flags, void* param)//��갴��ʱ�ƶ�һ��,ͼ�ξͻ����һ��,�ɿ���չ�����ͼ��
{
    Mat& image = *(cv::Mat*) param;
    switch (event)
    {
        //����ƶ���Ϣ
    case EVENT_MOUSEMOVE:
    {
        if (g_bDrawingBox)//����Ƿ���л��Ƶı�ʶ��Ϊ�棬���¼�³��Ϳ�RECT�ͱ�����
        {
            g_rectangle.width = x - g_rectangle.x;
            g_rectangle.height = y - g_rectangle.y;
        }
    }
    break;

    //���������Ϣ
    case EVENT_LBUTTONDOWN:
    {
        g_bDrawingBox = true;
        g_rectangle = Rect(x, y, 0, 0);//��¼��ʼ��
    }
    break;

    //���̧����Ϣ
    case EVENT_LBUTTONUP:
    {
        g_bDrawingBox = false;//�ñ�ʶ��Ϊfalse
        //�Կ�͸�С��0�Ĵ���
        if (g_rectangle.width < 0)
        {
            g_rectangle.x += g_rectangle.width;
            g_rectangle.width *= -1;
        }

        if (g_rectangle.height < 0)
        {
            g_rectangle.y += g_rectangle.height;
            g_rectangle.height *= -1;
        }
        DrawRectangle(image, g_rectangle);
        calAngle(cameraMatrix, distCoeffs,g_rectangle);

    }
    break;
    }
}
//-----------------------------------��DrawRectangle( )������------------------------------
//		�������Զ���ľ��λ��ƺ���
//-----------------------------------------------------------------------------------------------
void DrawRectangle(Mat& img, Rect box)
{
    rectangle(img, box.tl(), box.br(), Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255)));//�����ɫ
}

void calAngle(Mat cam, Mat dis,Rect in)
{
    double fx = cam.at<double>(0, 0);
    double fy = cam.at<double>(1, 1);
    double disw = fx * WIDTH/in.width ;
    double dish = fy * HEIGH/in.height ;
    cout << "��" << in.width << endl;
    cout << "��" << in.height << endl;
    cout << "�øߵõ��ľ���" << dish << endl;
    cout << "�ÿ�õ��ľ���" << disw << endl;
    cout << "ƽ������" << (dish + disw) / 2 << endl;
}