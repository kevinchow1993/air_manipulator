#include "ros/ros.h"
#include "ros/time.h"
#include "string"
#include <iostream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>
#include <algorithm>  
#include <Eigen/Eigen> 
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/*********************************
	注意！！！！：：：
	
	标定的时候，link必须水平，以这个时候的坐标系为准，得到的角度就是link的坐标系，到时候在舵机标定的时候需要与这个坐标系一致。
**********************************/


using namespace Eigen; 
using namespace std;

#define pi 3.1415926

class ffcalib_waper
{
	public:
	Vector3d Owa,Owb,Oca,Owl,Obc,Obl,Olc;
	Matrix3d Rwa,Rwb,Rca,Rwl,Rbc,Rbl,Rlc;
	ros::Time time_wa1,time_wa2,time_wb,time_ca,time_wl1,time_wl2;
	vector<Quaterniond> QData;
	vector<Vector3d> OData;
	Quaterniond avg_Rlc;
	Vector3d avg_Olc;

	ffcalib_waper()
	{
		QData.clear();
	}
	~ffcalib_waper()
	{
		
	}
	void reload_wa_pose(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		time_wa1=ros::Time::now();
        //Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
        //Rwa = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
        Owa<< msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
	}
	void reload_wa_ori(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		time_wa2=ros::Time::now();
        Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
        Rwa = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
        //Owa<< msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
	}
	void reload_wb(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		time_wb=ros::Time::now();
        Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
        Rwb = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
        Owb<< msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
	}
	void reload_wl_pose(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		time_wl1=ros::Time::now();
        //Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
        //Rwl = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
        Owl<< msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
	}
	void reload_wl_ori(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		time_wl2=ros::Time::now();
        Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
        Rwl = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
        //Owl<< msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
	}
	void reload_ca(tf::StampedTransform &transform)
	{
		time_ca=ros::Time::now();
		Quaterniond q(transform.getRotation().w(),transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z());
		Rca = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
		Oca<< transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z();
		
	}
	bool check_time_data_valid(void)
	{
		unsigned int t1=time_wa1.toSec(),t2=time_wa2.toSec(),t3=time_wb.toSec(),t4=time_wl1.toSec(),t5=time_wl2.toSec(),t6=time_ca.toSec();
		int time_outcnt=100;
		if (abs(t6-t1)<time_outcnt&&abs(t6-t2)<time_outcnt&&abs(t6-t3)<time_outcnt&&abs(t6-t4)<time_outcnt&&abs(t6-t5)<time_outcnt)return true;
		else return false;
	}
	void Calculate_data_add(void)
	{
		if (check_time_data_valid())
		{
			Obc=Rwb.transpose()*(Owa-Owb)-Rwb.transpose()*Rwa*Rca.transpose()*Oca;//(Owa-Owb)-Rwb.transpose()*Rwa*Rca.transpose()*Oca
			Rbc=Rwb.transpose()*Rwa*Rca.transpose();

			Rbl=Rwb.transpose()*Rwl;
			Obl=-Rwb.transpose()*Owb+Rwb.transpose()*Owl;//-Owb+Rwb.transpose()*Owl
			//------//
			Rlc=Rbl.transpose()*Rbc;
			Olc=-Rbl.transpose()*Obl+Rbl.transpose()*Obc;

			// cout << "Rlc=\n" << Rlc << endl;
			// cout << "Olc=\n" << Olc << endl;

			Quaterniond qRlc(Rlc);
			//cout << "qRlc :\n" << qRlc.coeffs() << endl;
			QData.push_back(qRlc);
			OData.push_back(Olc);
			

		}
	}
	void Average_Qdata(void)
	{

		MatrixXd A=MatrixXd::Zero(4,4);
		for (int i = 0; i < QData.size(); ++i)
		{
			MatrixXd qV=QData[i].coeffs();
			A=qV*qV.transpose()+A;
		}
		A=(1.0)/QData.size()*A;
		EigenSolver<MatrixXd> es(A);
		MatrixXd D = es.pseudoEigenvalueMatrix();
		MatrixXd V = es.pseudoEigenvectors();
		int max_pose=0;
		double max_eigenvalue=D(0,0);
		for (int i = 1; i < 4; ++i)
		{

			if (max_eigenvalue<D(i,i))
			{
				max_eigenvalue=D(i,i);
				max_pose=i;
			}
		}
		Vector4d avgv=V.col(max_pose);
		avg_Rlc=Quaterniond(avgv);
		avg_Olc(0)=0.0;avg_Olc(1)=0.0;avg_Olc(2)=0.0;
		for (int i = 0; i < OData.size(); ++i)
		{
			avg_Olc(0)=avg_Olc(0)+OData[i](0);
			avg_Olc(1)=avg_Olc(1)+OData[i](1);
			avg_Olc(2)=avg_Olc(2)+OData[i](2);
		}
		avg_Olc(0)=avg_Olc(0)/OData.size();
		avg_Olc(1)=avg_Olc(1)/OData.size();
		avg_Olc(2)=avg_Olc(2)/OData.size();

	}




}ffcalib_handler;


void wa_Callback_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 ffcalib_handler.reload_wa_pose(msg);
}

void wa_Callback_ori(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 ffcalib_handler.reload_wa_ori(msg);
}

void wb_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 ffcalib_handler.reload_wb(msg);
}

void wl_Callback_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 ffcalib_handler.reload_wl_pose(msg);
}

void wl_Callback_ori(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 ffcalib_handler.reload_wl_ori(msg);
}

// struct MyFunctor
// {
//     int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
//     {
//         double tmp1, tmp2, tmp3;
//         static const double y[15] = { 1.4e-1, 1.8e-1, 2.2e-1, 2.5e-1, 2.9e-1, 3.2e-1, 3.5e-1,
//             3.9e-1, 3.7e-1, 5.8e-1, 7.3e-1, 9.6e-1, 1.34, 2.1, 4.39 };

//         for (int i = 0; i < values(); i++)
//         {
//             tmp1 = i + 1;
//             tmp2 = 16 - i - 1;
//             tmp3 = (i >= 8) ? tmp2 : tmp1;
//             fvec[i] = y[i] - (x[0] + tmp1 / (x[1] * tmp2 + x[2] * tmp3));
//         }

//         return 0;
//     }


//     int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
//     {
//         double tmp1, tmp2, tmp3, tmp4;
//         for (int i = 0; i < values(); i++)
//         {
//             tmp1 = i + 1;
//             tmp2 = 16 - i - 1;
//             tmp3 = (i >= 8) ? tmp2 : tmp1;
//             tmp4 = (x[1] * tmp2 + x[2] * tmp3); tmp4 = tmp4*tmp4;
//             fjac(i, 0) = -1;
//             fjac(i, 1) = tmp1*tmp2 / tmp4;
//             fjac(i, 2) = tmp1*tmp3 / tmp4;
//         }
//         return 0;
//     }

//     int inputs() const { return 3; }
//     int values() const { return 3; } // number of constraints
// };


void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	cv::Mat image=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::imshow("source_camera_image", image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	cv::Mat image=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::imshow("image_proc_remove_distortion_imgae", image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void test_local(void)
{

	Quaterniond qRlc(-0.020868,0.889458,-0.456539,0.001319);//w x y z
	for (int i = 0; i < 2; ++i)
	{
		ffcalib_handler.QData.push_back(qRlc);
	}
	ffcalib_handler.Average_Qdata();
	cout<<"-------------"<<endl;
	cout<<ffcalib_handler.avg_Rlc.coeffs()<<endl;
	cout<<"--"<<endl;
	cout<<ffcalib_handler.avg_Olc<<endl;

}



int main(int argc, char* argv[])
{

	// test_local();
	// getchar();


	// Eigen::VectorXf x(3);
 //    x(0) = 10;
 //    x(1) = -2;
 //    x(2) = 3;
 //    //x(2) = 0;

 //    std::cout << "x: " << x << std::endl;

 //    MyFunctor functor;
 //    Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);

 //    lm.minimize(x);

 //    std::cout << "x that minimizes the function: " << x << std::endl;



	ros::init(argc, argv, "ffcalib");
  	ros::NodeHandle n;
  	ros::Subscriber wa_pose_feedback=n.subscribe("/world_to_apriltags_pose/pose",1,wa_Callback_pose);
  	ros::Subscriber wa_ori_feedback=n.subscribe("/world_to_apriltags_ori/pose",1,wa_Callback_ori);

  	ros::Subscriber wl_feedback_pose=n.subscribe("/world_to_link_pose/pose",1,wl_Callback_pose);
  	ros::Subscriber wl_feedback_ori=n.subscribe("/world_to_link_ori/pose",1,wl_Callback_ori);


  	ros::Subscriber wb_feedback=n.subscribe("/mavros/vision_pose/pose",1,wb_Callback);


  	//cv::namedWindow("source_camera_image");
  	//cv::namedWindow("image_proc_remove_distortion_imgae");
  	//cv::startWindowThread();
	image_transport::ImageTransport it(n);
	//image_transport::Subscriber sub1 = it.subscribe("/f_camera/image_raw", 1, imageCallback1);
	//image_transport::Subscriber sub2 = it.subscribe("/f_camera/image_rect_color", 1, imageCallback2);

  	ros::Rate loop_rate(100);

  	tf::TransformListener listener;

  	while (ros::ok())
	{
		tf::StampedTransform transform;
	    try
	    {
	      listener.lookupTransform("/camera", "/target1", ros::Time(0), transform);
	      ffcalib_handler.reload_ca(transform);
	    }
	    catch (tf::TransformException ex)
	    {
	      ROS_ERROR("%s",ex.what());
	    }

	    if (ffcalib_handler.check_time_data_valid())
	    {
	    	ffcalib_handler.Calculate_data_add();
	    }
	    if (ffcalib_handler.QData.size()>200)
	    {
	    	ffcalib_handler.Average_Qdata();
	    	cout<<"-------------"<<endl;
	    	cout<<ffcalib_handler.avg_Rlc.coeffs()<<endl;
	    	double rol,pit,yaw,x,y,z,w;
	    	x=ffcalib_handler.avg_Rlc.x();y=ffcalib_handler.avg_Rlc.y();z=ffcalib_handler.avg_Rlc.z();w=ffcalib_handler.avg_Rlc.w();
	    	rol=atan2(2*w*x+2*y*z,1-2*x*x-2*y*y);
	    	pit=asin(2*w*y-2*z*x);
	    	yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
	    	cout<<rol*180.0/pi<<" "<<pit*180.0/pi<<" "<<yaw*180.0/pi<<endl;
	    	cout<<"--"<<endl;
	    	cout<<ffcalib_handler.avg_Olc<<endl;
	    	ffcalib_handler.QData.clear();
	    	ffcalib_handler.OData.clear();
	    }




		ros::spinOnce();
		loop_rate.sleep();
	}

	//cv::destroyWindow("source_camera_image");
	//cv::destroyWindow("image_proc_remove_distortion_imgae");

}