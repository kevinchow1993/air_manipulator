#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;


#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5  

//#define WIDTH 2560
#define WIDTH 1344
//#define HEIGHT 720
#define HEIGHT 376

#define FRAME_RATE 30

cv::Mat frame;

Mat XR, XT, Q, P1, P2;
Mat R1, R2, K1, K2, D1, D2, R;
Vec3d T;
Mat lmapx, lmapy, rmapx, rmapy;
FileStorage calib_file;
Size out_img_size(672, 376);
Size calib_img_size(672,376);


image_transport::Publisher pub_img_left;
image_transport::Publisher pub_img_right;

static int id = 0;

bool getImages(cv::Mat& left_image, cv::Mat& right_image) 
{
    cv::Rect left_rect(0, 0, WIDTH/2, HEIGHT);
    cv::Rect right_rect(WIDTH/2, 0, WIDTH/2, HEIGHT);
	left_image = frame(left_rect);
	right_image = frame(right_rect);	
}

void undistortRectifyImage(Mat& src, Mat& dst, FileStorage& calib_file, int left = 1) {
  if (left == 1) {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } else {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}

void findRectificationMap(FileStorage& calib_file, Size finalSize)
{
  Rect validRoi[2];
  cout << "starting rectification" << endl;
  stereoRectify(K1, D1, K2, D2, calib_img_size, R, Mat(T), R1, R2, P1, P2, Q,
                CV_CALIB_ZERO_DISPARITY, 0, finalSize, &validRoi[0], &validRoi[1]);
  cout << "done rectification" << endl;

  cv::initUndistortRectifyMap(K1, D1, R1, P1, finalSize, CV_32F, lmapx,lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, finalSize, CV_32F, rmapx,rmapy);
}

void imgLeftRectify(cv::Mat& img) {
  try
  {
    if (img.empty()) return;
    Mat dst;
    undistortRectifyImage(img, dst, calib_file, 1);
    sensor_msgs::ImagePtr img_left;
    img_left = cv_bridge::CvImage(std_msgs::Header(),  sensor_msgs::image_encodings::TYPE_8UC1, dst).toImageMsg();
    img_left->header.seq = id;
    pub_img_left.publish(img_left);
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

void imgRightRectify(cv::Mat& img) {
  try
  {
    // Mat tmp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    //Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_COLOR);
    if (img.empty()) return;
    Mat dst;
    undistortRectifyImage(img, dst, calib_file, 0);
    sensor_msgs::ImagePtr img_right;
    img_right = cv_bridge::CvImage(std_msgs::Header(),  sensor_msgs::image_encodings::TYPE_8UC1, dst).toImageMsg();
    img_right->header.seq = id;
    pub_img_right.publish(img_right);
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "zed_image_pub");
	ros::NodeHandle nh;

	//打开摄像头的参数
	cv::VideoCapture cap(1);
	if(!cap.isOpened())
	{
		ROS_ERROR("Open camera error");
		return -1;
	}

    cap.set(WIDTH_ID,WIDTH);
    cap.set(HEIGHT_ID,HEIGHT);
	cap.set(FPS_ID,FRAME_RATE);

	ros::Rate rate(40);

	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher l_pub = it.advertiseCamera("/camera/left/image_raw",10);
	image_transport::CameraPublisher r_pub = it.advertiseCamera("/camera/right/image_raw",10);

	sensor_msgs::ImagePtr l_image;
	sensor_msgs::ImagePtr r_image;

	sensor_msgs::CameraInfoPtr  l_info(new sensor_msgs::CameraInfo);
	sensor_msgs::CameraInfoPtr  r_info(new sensor_msgs::CameraInfo);

	//摄像头的内外参数
	boost::array<double,9> l_k = {{351.823991, 0.000000, 339.146036, 0.000000, 351.622611, 187.307216, 0.000000, 0.000000, 1.000000}};
	double l_d[5] = {-0.162344, 0.012386, -0.000308, -0.000355, 0.000000};
	std::vector<double> l_distor(l_d,l_d + 5);

	l_info->K =  l_k;
    l_info->width  = WIDTH/2;
    l_info->height = HEIGHT;
	l_info->D = l_distor;

	boost::array<double,9> r_k = {{350.865210, 0.000000, 352.924309, 0.000000, 350.913964, 189.654471, 0.000000, 0.000000, 1.000000}};
	double r_d[5] = {-0.157345, 0.010738, -0.000150, 0.000674, 0.000000};
	std::vector<double> r_distor(r_d,r_d + 5);

	r_info->K =  r_k;
    r_info->width  = WIDTH/2;
    r_info->height = HEIGHT;
	r_info->D = r_distor;

	cv::Mat left, right;

	// calib_file = FileStorage("/home/xiaopeng/catkin_ws/src/stereo-dense-reconstruction/calibration_files/stereo_calibrate.yaml", FileStorage::READ);
	// calib_file["K1"] >> K1;
	// calib_file["K2"] >> K2;
	// calib_file["D1"] >> D1;
	// calib_file["D2"] >> D2;
	// calib_file["R"] >> R;
	// calib_file["T"] >> T;
	// findRectificationMap(calib_file, out_img_size);

	// pub_img_left = it.advertise("/camera_left_rect/image_grey", 10);
	// pub_img_right = it.advertise("/camera_right_rect/image_grey", 10);

	while(nh.ok())
	{
		cap >>frame;
		//ROS_WARN("%d---%d",frame.cols, frame.rows);

		if(!frame.empty())
		{
            cv::cvtColor(frame,frame,cv::COLOR_BGR2GRAY);
			getImages(left,right);

            l_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, left).toImageMsg();
            l_image->header.stamp =  ros::Time::now();
            l_image->header.seq = id;
			l_info->header.stamp = ros::Time::now();
            l_info->header.seq = id;
			l_pub.publish(l_image,l_info);
			// imgLeftRectify(left);

            r_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, right).toImageMsg();
			r_image->header.stamp = ros::Time::now();
            r_image->header.seq = id;
			r_info->header.stamp = ros::Time::now();
            r_info->header.seq = id;
			r_pub.publish(r_image,r_info);
			// imgRightRectify(right);

            id++;
		}

		rate.sleep();
	}
	return 0;
}
