#include <iostream>
#include <opencv2/opencv.hpp>
#include <numeric>
#include<chrono>
using namespace cv;
using namespace cv::ml;
using namespace std;
//��ȡ������
double getDistance(Point A, Point B)
{
    double dis;
    dis = pow((A.x - B.x), 2) + pow((A.y - B.y), 2);
    return sqrt(dis);
}
//��׼��������hog
vector<float> stander(Mat im)
{

    if (im.empty() == 1)
    {
        cout << "filed open" << endl;
    }
    resize(im, im, Size(48, 48));

    vector<float> result;

    HOGDescriptor hog(cvSize(48, 48), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9, 1, -1,
        HOGDescriptor::L2Hys, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS);           //��ʼ��HOG������
    hog.compute(im, result);
    return result;
}
//��ͼƬת��Ϊsvm�����ʽ
Mat get(Mat input)
{
    vector<float> vec = stander(input);
    if (vec.size() != 900) cout << "wrong not 900" << endl;
    Mat output(1, 900, CV_32FC1);

    Mat_<float> p = output;
    int jj = 0;
    for (vector<float>::iterator iter = vec.begin(); iter != vec.end(); iter++, jj++)
    {
        p(0, jj) = *(iter);
    }
    return output;
}


/*
 * �ο�: http://blog.csdn.net/liyuanbhu/article/details/50889951
 * ͨ����С���˷������Բ����Ϣ
 * pts: ���е�����
 * center: �õ���Բ������
 * radius: Բ�İ뾶
 */
static bool CircleInfo2(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius)
{
    center = cv::Point2d(0, 0);
    radius = 0.0;
    if (pts.size() < 3) return false;;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumX2 = 0.0;
    double sumY2 = 0.0;
    double sumX3 = 0.0;
    double sumY3 = 0.0;
    double sumXY = 0.0;
    double sumX1Y2 = 0.0;
    double sumX2Y1 = 0.0;
    const double N = (double)pts.size();
    for (int i = 0; i < pts.size(); ++i)
    {
        double x = pts.at(i).x;
        double y = pts.at(i).y;
        double x2 = x * x;
        double y2 = y * y;
        double x3 = x2 * x;
        double y3 = y2 * y;
        double xy = x * y;
        double x1y2 = x * y2;
        double x2y1 = x2 * y;

        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumX3 += x3;
        sumY3 += y3;
        sumXY += xy;
        sumX1Y2 += x1y2;
        sumX2Y1 += x2y1;
    }
    double C = N * sumX2 - sumX * sumX;
    double D = N * sumXY - sumX * sumY;
    double E = N * sumX3 + N * sumX1Y2 - (sumX2 + sumY2) * sumX;
    double G = N * sumY2 - sumY * sumY;
    double H = N * sumX2Y1 + N * sumY3 - (sumX2 + sumY2) * sumY;

    double denominator = C * G - D * D;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double a = (H * D - E * G) / (denominator);
    denominator = D * D - G * C;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double b = (H * C - E * D) / (denominator);
    double c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

    center.x = a / (-2);
    center.y = b / (-2);
    radius = std::sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}

//ģ��ƥ��
double TemplateMatch(cv::Mat image, cv::Mat tepl, cv::Point& point, int method)
{
    int result_cols = image.cols - tepl.cols + 1;
    int result_rows = image.rows - tepl.rows + 1;
    //    cout <<result_cols<<" "<<result_rows<<endl;
    cv::Mat result = cv::Mat(result_cols, result_rows, CV_32FC1);
    cv::matchTemplate(image, tepl, result, method);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

    switch (method)
    {
    case CV_TM_SQDIFF:
    case CV_TM_SQDIFF_NORMED://�����ַ������õ���ֵԽС˵��ƥ�侫��Խ��,���෽���෴
        point = minLoc;
        return minVal;

    default:
        point = maxLoc;
        return maxVal;

    }
}

//#define USE_CAMERA
//#define SAVE_VIDEO
//#define LEAF_IMG
//#define DEBUG
//#define DEBUG_LOG
#define USE_TEMPLATE
//#define USE_SVM
#define SHOW_RESULT
#define SHOW_CIRCLE
//#define SHOW_ALL_CONTOUR
#define RED
int main(int argc, char* argv[])
{
#ifdef USE_CAMERA
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
#else
    VideoCapture cap;
    cap.open("red.avi");
#endif
#ifdef LEAF_IMG
    //���ڼ�¼��Ҷ��ţ����㱣��ͼƬ
    int cnnt = 0;
#endif
#ifdef USE_SVM
    //load svm model
    Ptr<SVM> svm = SVM::create();
    svm = SVM::load("../RM19windmillDemo/SVM4_9.xml");
#endif
    // Save video
#ifdef SAVE_VIDEO
    VideoWriter writer;
    bool isRecording = false;
    time_t t;
    time(&t);
    const string fileName = "/home/happy/��Ƶ/" + to_string(t) + ".avi";
    writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(1280, 720));
    //    writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 480));
#endif

    Mat srcImage;
    cap >> srcImage;
    // �����Բ
    Mat drawcircle = Mat(srcImage.rows, srcImage.cols, CV_8UC3, Scalar(0, 0, 0));
#ifdef USE_TEMPLATE
    Mat templ[9];
    for (int i = 1; i <= 8; i++)
    {
        templ[i] = imread("template/template" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
    }
#endif
    vector<Point2f> cirV;

    Point2f cc = Point2f(0, 0);

    //������ѭ��

    while (true)
    {
        cap >> srcImage;
        auto t1 = chrono::high_resolution_clock::now();
#ifdef SAVE_VIDEO
        if (!writer.isOpened())
        {
            cout << "Capture failed." << endl;
            continue;
        }
        if (isRecording)
        {
            writer << srcImage;
        }
        if (!isRecording)
            cout << "Start capture. " + fileName + " created." << endl;
        isRecording = true;
#endif
        //�ָ���ɫͨ��
        vector<Mat> imgChannels;
        split(srcImage, imgChannels);
        //���Ŀ����ɫͼ��Ķ�ֵͼ
#ifdef RED
        Mat midImage2 = imgChannels.at(2) - imgChannels.at(0);
#endif
#ifndef RED
        Mat midImage2 = imgChannels.at(0) - imgChannels.at(2);
#endif
        //��ֵ��������Ϊ��ɫ��ͼ��Ϊ��ɫ
        //���ڲ�����Ҷ
        threshold(midImage2, midImage2, 100, 255, CV_THRESH_BINARY);
#ifdef DEBUG
        imshow("midImage2", midImage2);
#endif  
        int structElementSize = 2;
        Mat element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
        //����
        dilate(midImage2, midImage2, element);
        //�����㣬������Ҷ�Ͽ��ܴ��ڵ�С��
        structElementSize = 3;
        element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
        morphologyEx(midImage2, midImage2, MORPH_CLOSE, element);
#ifdef DEBUG
        imshow("dilate", midImage2);
#endif
        //��������
        vector<vector<Point>> contours2;
        vector<Vec4i> hierarchy2;
        findContours(midImage2, contours2, hierarchy2, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);

        RotatedRect rect_tmp2;
        bool findTarget = 0;

        //��������
        if (hierarchy2.size())
            for (int i = 0; i >= 0; i = hierarchy2[i][0])
            {
                rect_tmp2 = minAreaRect(contours2[i]);
                Point2f P[4];
                rect_tmp2.points(P);
                //char frame[10] = "p0";
                Point2f srcRect[4];
                Point2f dstRect[4];
                /*for (int n = 0; n < 4; n++) {
                    String frame = { "p" + to_string(n) };
                    putText(srcImage, frame, P[n], CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 255), 1, 8);
                }*/
                
                double width;
                double height;

                //������ȡ��ҶƬ�Ŀ���
                width = getDistance(P[0], P[1]);
                height = getDistance(P[1], P[2]);
                if (width > height)
                {
                    srcRect[0] = P[0];
                    srcRect[1] = P[1];
                    srcRect[2] = P[2];
                    srcRect[3] = P[3];
                }
                else
                {
                    swap(width, height);
                    srcRect[0] = P[1];
                    srcRect[1] = P[2];
                    srcRect[2] = P[3];
                    srcRect[3] = P[0];
                }
#ifdef SHOW_ALL_CONTOUR
                Scalar color(rand() & 255, rand() & 255, rand() & 255);
                drawContours(srcImage, contours2, i, color, 4, 8, hierarchy2);
#endif
                //ͨ�����ɸѡ
                double area = height * width;
                if (area > 5000) {
#ifdef DEBUG_LOG
                    cout << hierarchy2[i] << endl;

#endif
                    dstRect[0] = Point2f(0, 0);
                    dstRect[1] = Point2f(width, 0);
                    dstRect[2] = Point2f(width, height);
                    dstRect[3] = Point2f(0, height);
                    // Ӧ��͸�ӱ任�������ɹ������
                    Mat transform = getPerspectiveTransform(srcRect, dstRect);//�ѽǶȴ�б���л�������
                    Mat perspectMat;
                    warpPerspective(midImage2, perspectMat, transform, midImage2.size());
#ifdef DEBUG
                    imshow("warpdst", perspectMat);
#endif
                    // ��ȡ��ҶͼƬ
                    Mat testim;
                    testim = perspectMat(Rect(0, 0, width, height));
#ifdef LEAF_IMG
                    //���ڱ�����ҶͼƬ���Ա������ѵ��svm
                    string s = "leaf" + to_string(cnnt) + ".jpg";
                    cnnt++;
                    imwrite("./img/" + s, testim);
#endif

#ifdef DEBUG
                    imshow("testim", testim);
#endif
                    if (testim.empty())
                    {
                        cout << "filed open" << endl;
                        return -1;
                    }
#ifdef USE_TEMPLATE
                    cv::Point matchLoc;
                    double value;
                    Mat tmp1;
                    resize(testim, tmp1, Size(42, 20));
#endif
#if (defined DEBUG)&&(defined USE_TEMPLATE)
                    imshow("temp1", tmp1);
#endif
#ifdef USE_TEMPLATE
                    vector<double> Vvalue1;
                    vector<double> Vvalue2;
                    for (int j = 1; j <= 6; j++)
                    {
                        value = TemplateMatch(tmp1, templ[j], matchLoc, CV_TM_CCOEFF_NORMED);
                        Vvalue1.push_back(value);
                    }
                    for (int j = 7; j <= 8; j++)
                    {
                        value = TemplateMatch(tmp1, templ[j], matchLoc, CV_TM_CCOEFF_NORMED);
                        Vvalue2.push_back(value);
                    }
                    int maxv1 = 0, maxv2 = 0;

                    for (int t1 = 0; t1 < 6; t1++)
                    {
                        if (Vvalue1[t1] > Vvalue1[maxv1])
                        {
                            maxv1 = t1;
                        }
                    }
                    for (int t2 = 0; t2 < 2; t2++)
                    {
                        if (Vvalue2[t2] > Vvalue2[maxv2])
                        {
                            maxv2 = t2;
                        }
                    }
#endif
#if (defined DEBUG_LOG)&&(defined USE_TEMPLATE)
                    cout << Vvalue1[maxv1] << endl;
                    cout << Vvalue2[maxv2] << endl;
#endif
#ifdef USE_SVM
                    //ת��Ϊsvm��Ҫ��ĸ�ʽ
                    Mat test = get(testim);
#endif
                    //Ԥ���Ƿ���Ҫ�������Ҷ
#ifdef USE_TEMPLATE
                    if (Vvalue1[maxv1] > Vvalue2[maxv2] && Vvalue1[maxv1] > 0.6)
#endif
#ifdef USE_SVM
                        if (svm->predict(test) >= 0.9)
#endif
                        {
                            findTarget = true;
                            //����װ�װ�
                            if (hierarchy2[i][2] >= 0)
                            {
                                RotatedRect rect_tmp = minAreaRect(contours2[hierarchy2[i][2]]);
                                //Point2f vertex[4];
                                //rect_tmp.points(vertex);
                                //for (int i = 0; i < 4; i++) {
                                //    line(srcImage, vertex[i], vertex[(i + 1) % 4], Scalar(0, 255, 0), 2);
                                //}
                                Point2f Pnt[4];
                                rect_tmp.points(Pnt);
                                for (int n = 0; n < 4; n++) {
                                    String frame = { "p" + to_string(n) };
                                    putText(srcImage, frame, Pnt[n], CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 0, 255), 1, 8);
                                }
                                const float maxHWRatio = 0.7153846;
                                const float maxArea = 2000;
                                const float minArea = 500;

                                float width = rect_tmp.size.width;
                                float height = rect_tmp.size.height;
                                if (height > width)
                                    swap(height, width);
                                float area = width * height;

                                if (height / width > maxHWRatio || area > maxArea || area < minArea) {
#ifdef DEBUG
                                    cout << "hw " << height / width << "area " << area << endl;
                                    for (int j = 0; j < 4; ++j)
                                    {
                                        line(srcImage, Pnt[j], Pnt[(j + 1) % 4], Scalar(255, 0, 255), 4);
                                    }
                                    for (int j = 0; j < 4; ++j)
                                    {
                                        line(srcImage, P[j], P[(j + 1) % 4], Scalar(255, 255, 0), 4);
                                    }
                                    imshow("debug", srcImage);
                                    waitKey(0);
#endif
                                    continue;
                                }
                                Point centerP = rect_tmp.center;
                                //�����
                                circle(srcImage, centerP, 1, Scalar(0, 255, 0), 2);
#ifdef SHOW_CIRCLE
                                circle(drawcircle, centerP, 1, Scalar(0, 0, 255), 1);
                                //�������Բ����30�������Բ
                                if (cirV.size() < 30)
                                {
                                    cirV.push_back(centerP);
                                }
                                else
                                {
                                    float R;
                                    //�õ���ϵ�Բ��
                                    CircleInfo2(cirV, cc, R);
                                    circle(drawcircle, cc, 1, Scalar(255, 0, 0), 2);
#endif
#if (defined DEBUG_LOG)&& (defined SHOW_CIRCLE)
                                    cout << endl << "center " << cc.x << " , " << cc.y << endl;
#endif
#ifdef SHOW_CIRCLE
                                    cirV.erase(cirV.begin());

                                }
                                if (cc.x != 0 && cc.y != 0) {
                                    Mat rot_mat = getRotationMatrix2D(cc, 30, 1);
#endif
#if (defined DEBUG_LOG)&&(defined SHOW_CIRCLE)
                                    cout << endl << "center1 " << cc.x << " , " << cc.y << endl;
#endif
#ifdef SHOW_CIRCLE
                                    float sinA = rot_mat.at<double>(0, 1);//sin(60);
                                    float cosA = rot_mat.at<double>(0, 0);//cos(60);
                                    float xx = -(cc.x - centerP.x);
                                    float yy = -(cc.y - centerP.y);
                                    Point2f resPoint = Point2f(cc.x + cosA * xx - sinA * yy, cc.y + sinA * xx + cosA * yy);
                                    circle(srcImage, resPoint, 1, Scalar(0, 255, 0), 10);
                                }
#endif
                                for (int j = 0; j < 4; ++j)
                                {
                                    line(srcImage, Pnt[j], Pnt[(j + 1) % 4], Scalar(0, 255, 255), 2);
                                }
                            }
                        }
                }
#ifdef DEBUG_LOG
                cout << "width2 " << width << " height2 " << height << " Hwratio2 " << height / width << " area2 " << area << endl;
#endif

            }

#if (defined SHOW_CIRCLE)&&(defined SHOW_RESULT)
        imshow("circle", drawcircle);
#endif
#ifdef SHOW_RESULT
        imshow("Result", srcImage);
        if ('q' == waitKey(1))break;
#endif
        //����������ʱ��
        auto t2 = chrono::high_resolution_clock::now();
        cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
        //        t1 = chrono::high_resolution_clock::now();

    }
    return 0;
}


