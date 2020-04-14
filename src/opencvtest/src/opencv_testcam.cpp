#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>  
#include "iostream"
#include <opencvtest/contours.h>
#include <opencvtest/img_pro_info.h>
#include <typeinfo>
//static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
long int t_last = 0,t_now = 0;
long double t_during = 0.0;
int red_offset_add ;
int red_offset_sub ;
int red_offset_line ;
int alpha_slider_add;
int alpha_slider_sub;
int alpha_slider_line;
//存储轮廓信息
vector<int > Max,Min;
//存储前一帧的轮廓信息
opencvtest::img_pro_info info_pub_last;
//按照轮廓周长排序
// static inline bool LinesSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
// {  
//     return (contour1.size() > contour2.size());  
// }  
//按照轮廓面积排序  
static inline bool ContoursSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
{  
    return (cv::contourArea(contour1) > cv::contourArea(contour2));  
}
// static inline bool ContoursSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
// {  
//     return contour1.size() > contour2.size();
// }
struct img_pro_info
{
    float dis;
    int width_left;
    int width_right;
    int x_pos;
    int y_pos;
};

static inline bool x_order(Point contours1,Point contours2)
{
    return(contours1.x < contours2.x);
}

// void Point_order(const vector<vector<Point> > &contours_ploy)
// {
//     int size = contours_ploy.size();
//     std::sort(contours_ploy[0].begin(),contours_ploy[0].end(),x_order); 
// }
void sort_Rectangle(vector<vector<Point> > contours_ploy,vector<Point> &contours_max)
{
    //轮廓排序
    std::sort(contours_ploy[0].begin(),contours_ploy[0].end(),x_order);
    if(contours_ploy[0][0].y < contours_ploy[0][1].y)
    {
        contours_max[0] = Point(contours_ploy[0][0].x,contours_ploy[0][0].y);
        contours_max[1] = Point(contours_ploy[0][1].x,contours_ploy[0][1].y);
    }
    else
    {
        contours_max[0] = Point(contours_ploy[0][1].x,contours_ploy[0][1].y);
        contours_max[1] = Point(contours_ploy[0][0].x,contours_ploy[0][0].y);
    }
    if(contours_ploy[0][2].y < contours_ploy[0][3].y)
    {
        contours_max[2] = Point(contours_ploy[0][2].x,contours_ploy[0][2].y);
        contours_max[3] = Point(contours_ploy[0][3].x,contours_ploy[0][3].y);
    }
    else
    {
        contours_max[2] = Point(contours_ploy[0][3].x,contours_ploy[0][3].y);
        contours_max[3] = Point(contours_ploy[0][2].x,contours_ploy[0][2].y);
    }

}
void red_Filters(Mat img_hsv,Mat &img_red)
{
    int red_hsv = 120;
    int red_offset = 10;
    int s_hsv = 80;
    //红色区域筛选
    for (int i = 0; i < img_hsv.rows; i++)
    {
        for (int j = 0; j < img_hsv.cols; j++)
        {
            //CvScalar hsv_point = cvGet2D(img_hsv, i, j);//获取像素点为（j, i）点的HSV的值 
            Vec3i hsv_point = img_hsv.at<Vec3b>(i,j);
    /*
            例如 8U 类型的 RGB 彩色图像可以使用 <Vec3b>,3 通道 float 类型的矩阵可以使用 <Vec3f>。

            对于 Vec 对象，可以使用 [ ] 符号如操作数组般读写其元素，

            Vec3b color;        //用 color 变量描述一种 RGB 颜色
            color[0] = 255;     //0通道的B 分量
            color[1] = 0;       //1通道的G 分量
            color[2] = 0;       //2通道的R 分量 

            假设提前已知一幅图像img的数据类型为 unsigned char型灰度图（单通道），要对座标为(14,25)的像素重新赋值为25

            srcImage.at<uchar>(14,25) = 25;
            如果要操作的图片img是一幅数据类型同样为unsigned char的彩色图片，再次要求将座标（14,25）的像素赋值为25。

            Opencv中图像三原色在内存中的排列顺序为B-G-R

            img.at<Vec3b>(14, 25) [0] = 25;  //B  
            img.at<Vec3b>(14, 25) [1] = 25;  //G  
            img.at<Vec3b>(14, 25）[2] = 25;  //R 
    */
            if ((hsv_point.val[0]>red_hsv - red_offset)&&(hsv_point.val[0]<red_hsv + red_offset) && (hsv_point.val[1] > s_hsv))
            {
                //cvSet2D(img_red, i ,j, white_point);
                img_red.at<uchar>(i,j) = 255;//8U 类型的 RGB 彩色图像每个通道都是uchar类型的
            }
        }           
    }

}
void similar_Decision()
{
    //通过相似度查找符合条件的轮廓
    //cout<<"第一种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours_ploy[0],contours_ploy[0],CV_CONTOURS_MATCH_I1, 0)<<endl;
    //cout<<"第二种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours_ploy[0],contours_ploy[1],CV_CONTOURS_MATCH_I2, 0)<<endl;
    //cout<<"第三种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours_ploy[0],contours_ploy[1],CV_CONTOURS_MATCH_I3, 0)<<endl;
}
void center_Compute(vector<vector<Point> > contours_ploy_filter,int index,Moments &mu,Point2f &mc)
{
    mu = moments( contours_ploy_filter[index], true ); 
    /*
        moments()来计算图像中的特征矩(最高到三阶) 参考 https://www.cnblogs.com/mikewolf2002/p/3427564.html
        array:输入数组，可以是光栅图像(单通道，8-bit或浮点型二维数组),或者是一个二维数组(1 X N或N X 1),二维数组类型为Point或Point2f

        binaryImage:默认值是false，如果为true，则所有非零的像素都会按值1对待，也就是说相当于对图像进行了二值化处理，阈值为1，此参数仅对图像有效。
    */
    mc = Point2d( mu.m10/mu.m00 , mu.m01/mu.m00 );
    //cout<<mc.x<<"     "<<mc.y<<endl;
}
void angel_Compute()
{
    //计算方向
    // double a = mu[0].m20 / mu[0].m00 - mc[0].x*mc[0].x;
    // double b = mu[0].m11 / mu[0].m00 - mc[0].x*mc[0].y;
    // double c = mu[0].m02 / mu[0].m00 - mc[0].y*mc[0].y;
    // double theta = fastAtan2(2*b,(a - c))/2;//此为形状的方向
    //cout<<"角度 = "<<theta<<endl;
}
void double_Line(vector<Point2f> mc,vector<int > &contours_id)
{
    cout<<"double_Line arrived!"<<endl;
    int lenth = mc.size();
    double dis_min = fabs(mc[0].x-mc[1].x) + fabs(mc[0].y-mc[1].y);
    double dis_now = 0;
    contours_id.push_back(0);
    contours_id.push_back(1);
    for(int i = 0;i < lenth-1;i++)
    {
        for(int j = i + 1;j < lenth;j++)
        {
            dis_now = fabs(mc[i].x-mc[j].x) + fabs(mc[i].y-mc[j].y);
            if(dis_now < dis_min)
            {
                dis_min = dis_now;
                contours_id[0] = i;
                contours_id[1] = j;      
            }  
        }
    }
    cout<<"dis_min = "<<dis_min<<endl;
    cout<<"dis_min_real = "<<fabs(mc[contours_id[0]].x-mc[contours_id[1]].x) + fabs(mc[contours_id[0]].y-mc[contours_id[1]].y)<<endl;
}
int contours_Out_Decision(vector<Point>  contours_ploy)
{
    int return_flag = 0;
    for(int j = 0;j < contours_ploy.size();j++)
    {
        if( (contours_ploy[j].x < 20 || contours_ploy[j].x > 620)  || (contours_ploy[j].y < 20 || contours_ploy[j].y > 460) )
        {
            return (1); 
            return_flag  = 1;
        }
    }
    if(!return_flag)
    {
        return (0);
    }
}
void draw_Circle(vector<Point2f> mc,Mat &img)
{
    int lenth = mc.size();
    Point2d center;
    for(int i = 0;i <lenth;i++)
    {
        center = Point(floor(mc[i].x),floor(mc[i].y));
        cv::circle(img, center, 5, Scalar(0,0,0),2);
    }  
}
void approxPolyDP_Compute(vector<vector<Point> > contours,vector<vector<Point> > &contours_ploy)
{
     for (size_t i = 0; i< contours.size(); i++)
    {
        //将轮廓做多边形逼近
        approxPolyDP(contours[i], contours_ploy[i], 3, true);//第三个变量表征逼近的程度
    /*
        对应的函数为：
        void approxPolyDP(InputArray curve, OutputArray approxCurve, double epsilon, bool closed)；
        第一个参数 InputArray curve：输入的点集
        第二个参数OutputArray approxCurve：输出的点集，当前点集是能最小包容指定点集的。画出来即是一个多边形。
        第三个参数double epsilon：指定的精度，也即是原始曲线与近似曲线之间的最大距离。
        第四个参数bool closed：若为true，则说明近似曲线是闭合的；反之，若为false，则断开。
    */
        //printf("%s\n",typeid(contours[0]).name());//类型
        //cout<<"类型为"<<typeid(contours[0]).name()<<endl;

        //查找最小的矩形包围框
        //minRect[i] = minAreaRect( Mat(contours_ploy[i]) );
        //rects_ploy[i] = boundingRect(contours_ploy[i]);    
        // if (contours_ploy[i].size() >5)
        // {
        //     RotatedRect temp1 = minAreaRect(contours_ploy[i]);
        //     RotatedRect_ploy.push_back(temp1);
        //     // 椭圆
        //     // RotatedRect temp2 = fitEllipse(contours_ploy[i]);
        //     // ellipse_ploy.push_back(temp2);
        // }
    }
}
int contours_Filters(vector<vector<Point> > contours_ploy)
{
    //查找图像上最长的边框
    int length = contours_ploy.size();
    if(length < 1)
    {
        return -1;//说明什么都没识别到
    }
    int contours_size = 0;
    int height[length];
    int width[length];
    vector<int > index;
    //cout<<"lenth = "<<lenth<<endl;
    for(int i = 0; i < length;i++)
    {
        Max.push_back(0);
        Min.push_back(480);
    }
    for(size_t i = 0;i < length; i++)//size_t就理解成int就行，是一种匹配操作系统的自动类型。这个类型足以用来表示对象的大小
    {
        //重新初始化
        //i是第i个独立的多边形轮廓，j是每个轮廓上的第j个点。每个点一个point类型结构体，分xyz三个成员。
        contours_size = contours_ploy[i].size();
        for(int j = 0; j < contours_size; j++)
        {
            if((contours_ploy[i][j].y < Min[i]) && (!isnan(contours_ploy[i][j].y)) && (!isnan(-contours_ploy[i][j].y)))
            {//判断非空是因为打印出contours_ploy的时候发现其中有nan，虽然不知道原因，猜测可能是像素的问题，所以就直接加了非空的判断。
                Min[i] = contours_ploy[i][j].y;
            }
            else
            {
                ;
            }
            if((contours_ploy[i][j].y > Max[i]) && (!isnan(contours_ploy[i][j].y)) && (!isnan(-contours_ploy[i][j].y)))
            {
                Max[i] = contours_ploy[i][j].y;
            }
            else
            {
                ;
            }
        }
        height[i] = Max[i] - Min[i];
    }
    for(int i = 0; i < length;i++)
    {
        width[i] = floor(contourArea(contours_ploy[i])/height[i]);
    }
    for(int i = 0; i < length; i++)
    {
        if(height[i] > 200 && width[i] <150 && contourArea(contours_ploy[i]) > 500)
        {
            index.push_back(i);//提取出来满足一定要求的轮廓，具体是什么要求应该是看要识别的是框还是柱子还是什么其他东西。
        }
    }
    //返回宽度最宽的边框，如果看到了多个框的话这样就只返回最近的
    if(index.size())
    {
        for(int i = 0; i < index.size() - 1;i++)//相当于做了个排序,如果只是需要返回最宽的，判断大小就行，干嘛做排序啊
        {
            for(int j = i + 1; j < index.size();j++)
            {
                if(width[index[i]] < width[index[j]])
                {
                    int a = index[i];
                    index[i] = index[j];
                    index[j] = a;           
                }
            }
        }
        return index[0];
    }
    else
    {
        return -1;
    }
        //cout<<"height[index] = "<<height[index]<<endl;
        //cout<<"Max = "<<Max[index]<<endl;
        //cout<<"Min = "<<Min[index]<<endl;
        // cout<<"Point info:"<<endl;
        // for(int i = 0;i < contours_ploy[index].size();i++)
        // {
        //     cout<<contours_ploy[index][i].x<<"    "<<contours_ploy[index][i].y<<endl;
        // }
    
}
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher contours_pub_ ;


    
public:
    ImageConverter()
    : it_(nh_)//构造函数
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);//接受原始图像话题
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        //contours_pub_ = it_.advertise("/contours_topic", 1);
        contours_pub_ = nh_.advertise<opencvtest::img_pro_info>("/contours_topic", 50);    //发布结果话题
        //cv::namedWindow(OPENCV_WINDOW);
    }
 
    ~ImageConverter()//析构函数
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)   //接收到了图像就进这个函数
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            //当要把一个ROS 的sensor_msgs /image 信息转化为一cvimage，cvbridge有两个不同的使用情况：
            //这里是第一种，如果要修改某一位的数据。我们必须复制ROS信息数据的作为副本。
    /*
            输入是图像消息指针，以及可选的编码参数。编码是指cvimage的类型。

            tocvcopy复制从ROS消息的图像数据，可以自由修改返回的cvimage。即使当源和目的编码匹配。

            介绍集中cvbridge 中常见的数据编码的形式，cv_bridge可以有选择的对颜色和深度信息进行转化。为了使用指定的特征编码，就有下面集中的编码形式：

            mono8:  CV_8UC1， 灰度图像

            mono16: CV_16UC1,16位灰度图像

            bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序

            rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序

            bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道

            rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道

            注：这其中mono8和bgr8两种图像编码格式是大多数OpenCV的编码格式。
    */
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }
        // 矩阵访问
        //如果是采用Mat形式存储，想要访问灰度图像的灰度值，可以采用如下方法：
        //int value = img.at<uchar>(i,j)[k];//k表示通道数


        // 画圈
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        // {
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        // }

        //载入图片
        //cv::Mat img = imread("momo3.jpeg",1);
        //变量定义
        cv::Mat img_gauss,img_hsv,img_red,img_red_gauss,img_red_median;//,img_gray,img_binary;
        //白色的点，用于赋值
        // CvScalar white_point;
        // white_point.val[0] = 255;
        // white_point.val[1] = 255;
        // white_point.val[2] = 255;
        cv::Mat img = cv_ptr->image;     //数据拷贝成Mat格式，opencv的图像多使用这个格式
        img_red = Mat::zeros(img.size(),CV_8UC1); //创建一个值为全0的 单通道图像矩阵， 大小和img相同
    /*      
        mono8:  CV_8UC1， 灰度图像
        mono16: CV_16UC1,16位灰度图像
        bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序
        rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序
        bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道
        rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道
    */
	    cv::GaussianBlur(img,img_gauss,cv::Size(3,3),0);    //高斯滤波，去除高斯噪声，高斯核的大小3*3
    /*输入含义：
        img:高斯滤波前的图像
        img_gauss:高斯滤波后的图像
        cv::Size(3,3) 高斯核大小
        0  高斯函数均值
    */
        cvtColor(img_gauss,img_hsv,CV_RGB2HSV);       //色域转换，rgb转hsv，我那个笔记里写了转换公式
    /*
            cvtColor函数是OpenCV里用于图像颜色空间转换，可以实现RGB颜色、HSV颜色、HSI颜色、lab颜色、YUV颜色等转换，也可以彩色和灰度图互转。
        输入含义：
            img_gauss   变换前图像
            img_hsv     变换后图像
            CV_RGB2HSV  变换方式
        注：变换方式有几十种

        还有其他等效写法：       
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) //个人猜cvt是convert的意思
    */        
        // cvtColor(img_hsv,img_gray,CV_RGB2GRAY);       //rgb转灰度
        red_Filters(img_hsv,img_red);                 //将图像中的红色部分过滤出来，红色部分赋值到img_red中显示,相当于二值化？
        //求取灰度均值
        // cv:Scalar gray_Val = cv::mean(img_gray);
        // float gray_Mean_0 = gray_Val.val[0];
        // std::cout<<"gray mean value = "<<gray_Mean_0<<std::endl;
        //某一点赋值
        // img.at<Vec3b>(200,100)[0] = 120;
        // img.at<Vec3b>(200,100)[1] = 255;
        // img.at<Vec3b>(200,100)[2] = 240;
        // Vec3i hsv_point = img_hsv.at<Vec3b>(122,100);
        // cout<<"hsv0 = "<<hsv_point.val[0]<<endl;
        // cout<<"hsv1 = "<<hsv_point.val[1]<<endl;
        // cout<<"hsv2 = "<<hsv_point.val[2]<<endl;
        //Vec3i rgb_point = img.at<Vec3b>(200,100);
        /////////////////////////////////////////////////////////////////////////////////
        //二值化处理
        //int thresh = 120;
        //int maxValue = 250;
        //threshold(img_gray,img_binary, thresh, maxValue, THRESH_BINARY);
        /////////////////////////////////////////////////////////////////////////////////
        //canny算子
        // Mat canny_img;
	    // cv::Canny(img_gray,canny_img,120,250);
        //imshow("CannyImg", canny_img);
        /////////////////////////////////////////////////////////////////////////////////
        //存储轮廓
        vector<vector<Point> > contours;                     //存储轮廓信息
	    vector<Vec4i> hierarchy;                             //求取轮廓时会生成的一个变量，具体是干什么用的我没有用到
        //中值滤波
        cv::medianBlur(img_red,img_red_median,11);           //中值滤波，二值图像过滤椒盐噪声使用，可以将那些意外被识别为红色的干扰点去除
        //高斯滤波
        //cv::GaussianBlur(img_red_gauss,img_red_median,cv::Size(3,3),0);
        //查找轮廓
        findContours(img_red_median,contours,hierarchy,RETR_LIST,CHAIN_APPROX_NONE,Point());//提取轮廓
    /*
        参考网址 https://www.cnblogs.com/GaloisY/p/11062065.html
        参数1：单通道图像矩阵，可以是灰度图，但更常用的是二值图像，一般是经过Canny、拉普拉斯等边缘检测算子处理过的二值图像；

        参数2：contours定义为“vector<vector<Point>> contours”，是一个双重向量
        （向量内每个元素保存了一组由连续的Point构成的点的集合的向量），每一组点集就是一个轮廓，有多少轮廓，contours就有多少元素；

        参数3：hierarchy定义为“vector<Vec4i> hierarchy”，Vec4i的定义：typedef Vec<int, 4> Vec4i;
        （向量内每个元素都包含了4个int型变量），所以从定义上看，hierarchy是一个向量，向量内每个元素都是一个包含4个int型的数组。
        向量hierarchy内的元素和轮廓向量contours内的元素是一一对应的，向量的容量相同。
        hierarchy内每个元素的4个int型变量是hierarchy[i][0] ~ hierarchy[i][3]，
        分别表示当前轮廓 i 的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓的编号索引。
        如果当前轮廓没有对应的后一个轮廓、前一个轮廓、父轮廓和内嵌轮廓，则相应的hierarchy[i][*]被置为-1。

        参数4：定义轮廓的检索模式，取值如下：

                    CV_RETR_EXTERNAL：只检测最外围轮廓，包含在外围轮廓内的内围轮廓被忽略；

                    CV_RETR_LIST：检测所有的轮廓，包括内围、外围轮廓，但是检测到的轮廓不建立等级关系，
                    彼此之间独立，没有等级关系，这就意味着这个检索模式下不存在父轮廓或内嵌轮廓，
                    所以hierarchy向量内所有元素的第3、第4个分量都会被置为-1，具体下文会讲到；

                    CV_RETR_CCOMP: 检测所有的轮廓，但所有轮廓只建立两个等级关系，外围为顶层，
                    若外围内的内围轮廓还包含了其他的轮廓信息，则内围内的所有轮廓均归属于顶层；

                    CV_RETR_TREE: 检测所有轮廓，所有轮廓建立一个等级树结构。外层轮廓包含内层轮廓，内层轮廓还可以继续包含内嵌轮廓。

        参数5：定义轮廓的近似方法，取值如下：

                    CV_CHAIN_APPROX_NONE：保存物体边界上所有连续的轮廓点到contours向量内；

                    CV_CHAIN_APPROX_SIMPLE：仅保存轮廓的拐点信息，把所有轮廓拐点处的点保存入contours向量内，
                    拐点与拐点之间直线段上的信息点不予保留；

                    CV_CHAIN_APPROX_TC89_L1：使用teh-Chinl chain 近似算法;

                    CV_CHAIN_APPROX_TC89_KCOS：使用teh-Chinl chain 近似算法。

        参数6：Point偏移量，所有的轮廓信息相对于原始图像对应点的偏移量，相当于在每一个检测出的轮廓点上加上该偏移量，
        并且Point还可以是负值！
    */
        //findContours(img_red_median,contours,hierarchy,RETR_LIST,CHAIN_APPROX_TC89_L1,Point());
	    // Mat imageContours = Mat::zeros(img.size(),CV_8UC1);
        // Mat poly_image = Mat::zeros(img.size(),CV_8UC1); //凸包图像，通俗来讲就是将轮廓拟合成多边形
	    // Mat Contours = Mat::zeros(img.size(),CV_8UC1);  //绘制
        //轮廓排序
        std::sort(contours.begin(),contours.end(),ContoursSortFun); //对轮廓进行排序，排序的规则按照面积从大到小排序，也可以使用周长进行排序
                                                //ContoursSortFun 上面有内联函数 里面写了比较面积大小的功能。因为识别框的话只有俩轮廓
        //cout<<"第一种相似判断，结果两个轮廓之间的相似度为：  "<<matchShapes(contours[0],contours[1],CV_CONTOURS_MATCH_I1, 0)<<endl;
        // 定义逼近后的存储容器
        vector<vector<Point> > contours_ploy(contours.size());
        //vector<Rect> rects_ploy(contours.size());
        //vector<RotatedRect> RotatedRect_ploy;//注意：由于下面赋值的过程中有个点数大于5的条件，所以这里没有直接初始化，才有下面pushback的方法添加值。
        //vector<RotatedRect> minRect( contours_ploy.size() );
        approxPolyDP_Compute(contours,contours_ploy);     //对轮廓进行多边形逼近,也是上面的函数
        ///////////////////////////////////////////////////////////////////////
        //返回滤波之后的轮廓即结果
        int height_max_index = contours_Filters(contours_ploy);//上面自己定义的函数,用来挑选轮廓,返回值是最大最高的轮廓的索引
        //cout<<"height_max_index = "<<height_max_index<<endl;
        //////////////////////////////////////////////////////////////////////
        //publish信息
        opencvtest::img_pro_info info_pub;
        if(height_max_index < 0)  //说明没有符合要求的轮廓，没有目标
        {
            //publish信息
            opencvtest::img_pro_info info_pub;
            info_pub.find_obs_flag = 0;
            info_pub.dis = -1;
            info_pub.pos_left = -1;
            info_pub.pos_right = -1;
            info_pub.x_pos = -1;
            info_pub.y_pos = -1;
            info_pub_last = info_pub;
            contours_pub_.publish(info_pub);
        }
        else
        {
            //cout<<info_pub_last.dis<<"  "<<info_pub_last.pos_left<<"  "<<info_pub_last.pos_right<<"  "<<info_pub_last.x_pos<<"  "<<info_pub_last.y_pos<<endl;  
            //零阶原点距和一阶原点距
            Moments mu;
            Point2f mc;//中心坐标
            center_Compute(contours_ploy,height_max_index,mu,mc);  //计算轮廓中心点
            //中心点保存
            Point2d Center;
            Center = Point(floor(mc.x),floor(Min[height_max_index]+ 0.5*(Max[height_max_index] - Min[height_max_index])));
            float width = contourArea(contours_ploy[height_max_index])*1.0/(Max[height_max_index] - Min[height_max_index]);
            cout<<"width = "<<width<<endl;
            info_pub.find_obs_flag = 1;
            info_pub.dis = 117/width;    //计算距离，距离柱子一米的时候，在图像上是117个像素点，根据成像原理，像素点个数与距离成反比。
            cout<<"dis = "<<info_pub.dis<<endl;
            info_pub.pos_left = Center.x - floor(width/2);
            info_pub.pos_right = Center.x + floor(width/2);
            //cout<<"width_left = "<<info_pub.pos_left<<endl;
            //cout<<"width_right = "<<info_pub.pos_right<<endl;
            //存储信息容器清空
            Min.clear();
            Max.clear();
            //保存面积最大的轮廓信息 
            info_pub.x_pos = Center.x - 320;
            info_pub.y_pos = 240 - Center.y;
            //在图像上显示
            drawContours(img,contours,height_max_index,Scalar(0,255,0),3,8,hierarchy);
            cv::circle(img, Center, 7, Scalar(255,255,255),2);
            if(info_pub_last.dis == 0 || info_pub_last.dis == -1)//上一帧没有框在视野内
            {
                contours_pub_.publish(info_pub);
            }
            else
            {
                //当位置变化太大的时候，就把上一次的中心位置发出去
                if(abs(info_pub.pos_left - info_pub_last.pos_left) < 20 && abs(info_pub.pos_right - info_pub_last.pos_right) < 20)
                {
                    info_pub_last = info_pub;
                    contours_pub_.publish(info_pub);
                }
                else
                {
                    contours_pub_.publish(info_pub_last);
                }
            }  
        }
        //绘制出点
        //for(int j=0;j<contours[i].size();j++) 
        //{            //绘制出contours向量内所有的像素点
            //Point P=Point(contours[i][j].x,contours[i][j].y);
            //Contours.at<uchar>(P)=255;
        //}
        //输出hierarchy向量内容
        //char ch[256];
        //sprintf(ch,"%d",i);
        //string str=ch;
        //cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
        //绘制轮廓
        //cout<<"size = "<<contours.size()<<endl;
        //绘制凸包的图像
        //cv::circle(poly_image, Point(300,300), 10, Scalar(255),2);
        //drawContours(poly_image,poly,i,Scalar(255),1,8,hierarchy); 
        // if(contours_ploy.size() >= 3)
        // {
        //     for(int m = 0;m < contours_ploy[0].size();m++)
        //     {
        //         cout << "x = " << contours_ploy[0][m].x << "   y = " << contours_ploy[0][m].y << endl;
        //     }
        //     cout<<endl;
        // }
        //查找竖直方向上的直线
        // vector<Vec4i> Lines;
        // float x_mean = 0;
        // float y_mean = 0;
        // //Point vertical_center;
        // int num_vertical = 0;
        // cv::HoughLinesP(imageContours, Lines, 1, CV_PI/180, red_offset_line,20,8);
        // vector<Point2d> Lines_vertical(Lines.size()*2);
        // for (int i = 0; i < Lines.size(); i++)
	    // {
        //     if(abs(Lines[i][1] - Lines[i][3]) > 10 && atan(abs(Lines[i][1] - Lines[i][3])*1.0/abs(Lines[i][0] - Lines[i][2])) > M_PI*3/8)
        //     {
        //         Lines_vertical[num_vertical] = Point2d(Lines[i][0],Lines[i][1]);
        //         num_vertical++;
        //         Lines_vertical[num_vertical] = Point2d(Lines[i][2],Lines[i][3]);
        //         num_vertical++;
        //         //line(img, Point(Lines[i][0], Lines[i][1]), Point(Lines[i][2], Lines[i][3]), Scalar(0, 255, 0), 4, 8);
        //     }
	    // }
        //vertical_center = Point2d(floor(x_mean/num_vertical),floor(y_mean/num_vertical));

        //cout<<"直线条数 = "<<Lines.size()<<endl;     
        // //定义最终绘图的图片
        //Mat draw_rotateRect(img.size(), img.type(), Scalar::all(0));
        // //绘图圆形、矩形
        //RNG rng(12345);
        // //绘图椭圆形、旋转矩形
        // Point2f pot[4];
        // for (size_t i = 0; i<RotatedRect_ploy.size(); i++)
        // {
        //     Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
        //     RotatedRect_ploy[i].points(pot);
        //     for(int j=0; j<4; j++)
        //     {
        //         line(draw_rotateRect, pot[j], pot[(j+1)%4], color);
        //     }
        // }

      
        //imshow("rotateRect",draw_rotateRect);
        //Update GUI Window
        //imshow("Contours Image",imageContours); //轮廓
        //imshow("gray Image",img_gray);
        imshow("hsv Image",img_hsv);
        //imshow("red Image",img_red);//hsv提取到的红色区域
        imshow("red Image_gauss",img_red_median);//高斯滤波后的红色区域
        //imshow("binary Image",img_binary);
        //imshow("poly Image",poly_image); //凸包的图像
        //imshow("draw_rotateRect", draw_rotateRect); 
	    //imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
        cv::imshow("OPENCV_WINDOW_color", img);
        t_now = cv::getTickCount();
        t_during = (t_now-t_last)/cv::getTickFrequency();
        //std::cout <<"Time of image = "<< t_during<<endl;
        t_last = t_now;
        //写入文件日志
        time_t now;
        struct tm *timenow;
        time(&now);
        timenow = localtime(&now);
        cout << timenow->tm_year+1900<<"_"<<timenow->tm_mon+1<<"_"<<timenow->tm_mday
        <<"_"<<timenow->tm_hour<<":"<<timenow->tm_min<<":"<<timenow->tm_sec
                << endl;
        FILE *flog = fopen("log.txt","a");
        fprintf(flog,"Time =  ");
        fprintf(flog,"%d",timenow->tm_year+1900);
        fprintf(flog,"_");
        fprintf(flog,"%d",timenow->tm_mon+1);
        fprintf(flog,"_");
        fprintf(flog,"%d",timenow->tm_mday);
        fprintf(flog,"_");
        fprintf(flog,"%d",timenow->tm_hour);
        fprintf(flog,":");
        fprintf(flog,"%d",timenow->tm_min);
        fprintf(flog,":");
        fprintf(flog,"%d",timenow->tm_sec);
        fprintf(flog,"     ");
        fprintf(flog,"x_pos =  ");
        fprintf(flog,"%d",info_pub.x_pos);
        fprintf(flog,"    ");
        fprintf(flog,"y_pos =  ");
        fprintf(flog,"%d",info_pub.y_pos);
        fprintf(flog,"    ");
        fprintf(flog,"distance =  ");
        fprintf(flog,"%f\n",info_pub.dis);
        fclose(flog);
        cv::waitKey(3);
    }
     
};
//调整阈值时使用
void on_trackbar_add( int, void* )
{
    red_offset_add = alpha_slider_add;
}
void on_trackbar_sub( int, void* )
{
    red_offset_sub = alpha_slider_sub;
} 
void on_trackbar_line( int, void* )
{
    red_offset_line = alpha_slider_line;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    int alpha_slider_line_max = 100;
    int alpha_slider_max = 50;

    // namedWindow("Linear Offset_add", 1);
    // namedWindow("Linear Offset_sub", 1);
    //namedWindow("Linear Offset_line", 1);
    // createTrackbar( "Trackbar", "Linear Offset_add", &alpha_slider_add, alpha_slider_max, on_trackbar_add );
    // createTrackbar( "Trackbar", "Linear Offset_sub", &alpha_slider_sub, alpha_slider_max, on_trackbar_sub );
    //createTrackbar( "Trackbar", "Linear Offset_line", &alpha_slider_line, alpha_slider_line_max, on_trackbar_line );
    ros::spin();
    return 0;
}

