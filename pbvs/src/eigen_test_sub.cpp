#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cmath"

#include <eigen_conversions/eigen_msg.h>//tf库中和eigen相互转换的库
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include "Eigen/Core"
#include <Eigen/Geometry>
#include "Eigen/Dense"
#include "iostream"
#define pi  3.1415926535898

using namespace std;

double sinc(double x){
	if(x==0){
		return 0;
	}else{
		return sin(x)/x;
	}
}

void vector_hat(Eigen::Vector3d v, Eigen::Matrix3d &v_hat){
	v_hat <<0,-v(2),v(1),
			v(2),0,-v(0),
			-v(1),v(0),0;

}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "eigen_test");
	ros::NodeHandle n;
	Eigen::Matrix<double,6,6> L_e;
	Eigen::Vector3d t_c_o;
	t_c_o << 1,2,3;
	Eigen::Matrix3d t_c_o_hat;
	vector_hat(t_c_o,t_c_o_hat);
	double theta = pi/2;
	Eigen::Vector3d U;
	U << 0,0,1;
	Eigen::Matrix3d U_hat;
	vector_hat(U,U_hat);
	Eigen::AngleAxisd thetaU(theta,U);
	cout<<thetaU.angle()<<"tt"<<thetaU.axis()<<endl;
	Eigen::Matrix3d L_thetaU = Eigen::MatrixXd::Identity(3,3)-U_hat*(theta/2)+(1-sinc(theta)/(sinc(theta/2)*sinc(theta/2)))*(U_hat*U_hat);

	L_e.block<3,3>(0,0) = -1.0*Eigen::MatrixXd::Identity(3,3);
	L_e.block<3,3>(0,3) = t_c_o_hat;
	L_e.block<3,3>(3,0) = Eigen::MatrixXd::Zero(3,3);
	L_e.block<3,3>(3,3) = L_thetaU;
	cout<<L_e<<endl;
	//cout<<"sinc(0)="<<sinc(0)<<"\t"<<"sinc(pi/2)="<<sinc(pi/2)<<endl;
	cout<<"L_e inverse =\n"<<L_e.inverse()<<endl;
	cout<< "angle_axis to rotation matrix \n"<<thetaU.toRotationMatrix()<<endl;

	Eigen::Quaterniond eigen_q(-0.484592,0.533285,-0.517225,0.461797);
    cout<<"----Tlc_test----"<<endl;
	cout<<eigen_q.toRotationMatrix()<<endl;
	

	



	return 0;
}