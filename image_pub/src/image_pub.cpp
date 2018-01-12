#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>


#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5


int main(int argc, char **argv)
{
	ros::init(argc,argv, "image_pub");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("/image_rect",1);

	//打开摄像头的参数
	cv::VideoCapture cap(1);
	cap.set(WIDTH_ID,640);
    cap.set(HEIGHT_ID,480);
    cap.set(FPS_ID,20);
	if(!cap.isOpened())
	{
		ROS_ERROR("Open camera error");
		return -1;
	}

    ros::Rate rate(10);
	sensor_msgs::ImagePtr image;

	sensor_msgs::CameraInfoPtr  info(new sensor_msgs::CameraInfo);
	//摄像头的内外参数
	boost::array<double,9> param= {{806.9, 0, 319.5, 0, 806.9, 239.5,0, 0, 1}};
	double d[5] = {-0.0778, -0.7795, 0, 0, -2.649};
	std::vector<double> distor(d,d+5);

	info->K =  param;
	info->width  = 640;
	info->height = 480;
	info->D = distor;

	cv::Mat frame;



	while(nh.ok())
	{
		cap >>frame;
		//ROS_WARN("%d---%d",frame.cols, frame.rows);

		if(!frame.empty())
		{
			image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			image->header.stamp =  ros::Time::now();
			info->header.stamp = ros::Time::now();
			pub.publish(image,info);
		}

		ros::spinOnce();
		rate.sleep();
	}

	cap.release();
	return 0;
}
