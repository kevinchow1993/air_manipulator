#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>



int main(int argc, char **argv)
{
	ros::init(argc,argv, "stereo_image_pub");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub_l = it.advertiseCamera("/camera/left/image_raw",20);
	image_transport::CameraPublisher pub_r = it.advertiseCamera("/camera/right/image_raw",20);

	//打开摄像头的参数
	cv::VideoCapture cap_l(1);
	cv::VideoCapture cap_r(2);

	cap_l.set(CV_CAP_PROP_FPS,20);
	cap_r.set(CV_CAP_PROP_FPS,20);

	if(!cap_l.isOpened() || !cap_r.isOpened())
	{
		ROS_ERROR("Open camera error");
		return -1;
	}

	ros::Rate rate(10);
	sensor_msgs::ImagePtr image_l;
	sensor_msgs::ImagePtr image_r;

	sensor_msgs::CameraInfoPtr  info_l(new sensor_msgs::CameraInfo);
	sensor_msgs::CameraInfoPtr  info_r(new sensor_msgs::CameraInfo);

	//左摄像头的内外参数
	boost::array<double,9> camera_l= {{6.7880112433177135e+02, 0., 3.2896948173318629e+02, 0.,
		       6.7880112433177135e+02, 2.6358122417326939e+02, 0., 0., 1. }};
	double dist_l[8] = {-4.6489257325349426e-01, 2.8888440852262587e-01, 0., 0., 0.,
		       0., 0., 2.9833754990810957e-01 };
	std::vector<double> distor_l(dist_l,dist_l + 8);

	info_l->K =  camera_l;
	info_l->width  = 640;
	info_l->height = 480;
	info_l->D = distor_l;

	//右摄像头的内外参数
	boost::array<double, 9> camera_r = { { 6.7880112433177135e+02, 0.,
			3.2896948173318629e+02, 0., 6.7880112433177135e+02,
			2.6358122417326939e+02, 0., 0., 1. } };
	double dist_r[8] = {-4.6489257325349426e-01, 2.8888440852262587e-01, 0.,
			0., 0., 0., 0., 2.9833754990810957e-01  };
	std::vector<double> distor_r(dist_r, dist_r + 8);

	info_r->K = camera_r;
	info_r->width = 640;
	info_r->height = 480;
	info_r->D = distor_r;

	cv::Mat frame_l,M1l,M2l;
	cv::Mat frame_r,M1r,M2r;
//
//	cv::Mat K_l =
//			(cv::Mat_<double>(3, 3) << 6.7880112433177135e+02, 0., 3.2896948173318629e+02, 0., 6.7880112433177135e+02, 2.6358122417326939e+02, 0., 0., 1.);
//	cv::Mat D_l =
//			(cv::Mat_<double>(1, 8) << -4.6489257325349426e-01, 2.8888440852262587e-01, 0., 0., 0., 0., 0., 2.9833754990810957e-01);
//	cv::Mat R_l =
//			(cv::Mat_<double>(3, 3) << 9.9788907265851434e-01, 4.6700785908429292e-02, -4.5126879620304219e-02, -4.7479146695907484e-02, 9.9873869276566118e-01, -1.6332611602761105e-02, 4.4307214962789915e-02, 1.8440720383791957e-02, 9.9884774141706312e-01);
//	cv::Mat P_l =
//			(cv::Mat_<double>(3, 4) << 4.8567251388371176e+02, 0., 3.6891242980957031e+02, 0., 0., 4.8567251388371176e+02, 2.6787726020812988e+02, 0., 0., 0., 1., 0.);
//
//	cv::Mat K_r =
//			(cv::Mat_<double>(3, 3) << 6.7880112433177135e+02, 0., 3.2836642577233590e+02, 0., 6.7880112433177135e+02, 2.6379233713242672e+02, 0., 0., 1.);
//	cv::Mat D_r =
//			(cv::Mat_<double>(1, 8) << -4.6695808086931778e-01, 2.3909580344715009e-01, 0., 0., 0., 0., 0., -1.4496562779886069e-01);
//	cv::Mat R_r =
//			(cv::Mat_<double>(3, 3) << 9.9771142310569905e-01, 4.8350569134962065e-02, -4.7266676091365048e-02, -4.7520529589897942e-02, 9.9869836899515041e-01, 1.8530165512534027e-02, 4.8101096368964417e-02, -1.6241620324078637e-02, 9.9871041563475860e-01);
//	cv::Mat P_r =
//			(cv::Mat_<double>(3, 4) << 4.8567251388371176e+02, 0., 3.6891242980957031e+02, -2.9695756588689051e+01, 0., 4.8567251388371176e+02, 2.6787726020812988e+02, 0., 0., 0., 1., 0.);
//
//
//    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(640,480),CV_32F,M1l,M2l);
//    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(640,480),CV_32F,M1r,M2r);

	while(nh.ok())
	{
		cap_l >> frame_l;
		cap_r >> frame_r;

		//ROS_WARN("%d---%d",frame_l.cols, frame_l.rows);
		//ROS_WARN("%d---%d",frame_r.cols, frame_r.rows);

		if(!frame_l.empty() && !frame_r.empty())
		{
//			cv::Mat img_l_recify, img_r_recify;
//
//			cv::remap(frame_l,img_l_recify, M1l, M2l, cv::INTER_LINEAR);
//			cv::remap(frame_r,img_r_recify, M1r, M2r, cv::INTER_LINEAR);

			image_l = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_l).toImageMsg();
			image_r = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_r).toImageMsg();


			pub_l.publish(image_l,info_l);
			pub_r.publish(image_r,info_r);

		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
