// FormationAlgorithm.cpp: implementation of the FlockingAlgo class.
// Formation Algorithms: 
// 1. Region-based formation
// 2. OTHER Algorithms
//////////////////////////////////////////////////////////////////////

#include "fflocking/flocking_algo.hpp"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


FlockingAlgo::FlockingAlgo()
{
  afai_ = 70.0f;
  gama_ = 150.0f;
  Mi_ = 1.0f;
  betai_ = 0.0f;
  k1_ = 1.0f;
  kl_ = 0.1f;
  kij_ = 1.0f;
  obstacle_radius_ = 0.5f;

  Ksi_ << 30.0f, 0.0f,
         0.0f, 30.0f;
  incre << 0., 0.;

  kp_ = 1.0f;
  T_ = 0.08f;

  default_radius_ = 1.0f;
  d_ = 1.5f;

  satu_thres_ = 2.0f;

  ///-------------------Circle Region-------------------------//
  m_radius_=0.5;
  m_ringWidth_ =0.2;

}

void FlockingAlgo::updatePara(const double & alpha, const double & gama,
                              const double & Mi,    const double & betai,
                              const double & k1,    const double & kl,
                              const double & kij,   const double & obstacle_radius,
                              const double & kp,    const double & T,
                              const double & d,     const double & default_radius,
                              const double & satu_thres,const double & m_radius,
                              const double & m_ringWidth){
  afai_ = alpha;
  gama_ = gama;// need to distinguish local variables and input para.
  Mi_ = Mi;
  betai_ = betai;
  k1_ = k1;
  kl_ = kl;
  kij_ = kij;
  obstacle_radius_ = obstacle_radius;
  kp_ = kp;
  T_ = T;
  d_ = d;
  default_radius_ = default_radius;
  satu_thres_ = satu_thres;
  m_radius_ = m_radius;
  m_ringWidth_ = m_ringWidth;

}

double FlockingAlgo::fgCircle(const Vector2f& xi,
                              const Vector2f& x0,
                              int flag)
{/*the global function of Circle Region
 @param[in] flag flag=0, inside the circle; flag=1, outside the circle
 */
	double temp;
	if(!flag)
    temp = (xi(0)-x0(0))*(xi(0)-x0(0))+(xi(1)-x0(1))*(xi(1)-x0(1))-m_radius_*m_radius_;
	else
    temp = m_radius_*m_radius_-((xi(0)-x0(0))*(xi(0)-x0(0))+(xi(1)-x0(1))*(xi(1)-x0(1)));
	return temp;
}

double FlockingAlgo::fgCircle(const Vector2f& xi,
                              const Vector2f& x0,
                              double radius,
                              int flag)
{/*the global function of Circle Region
 @param[in] flag flag=0, inside the circle; flag=1, outside the circle
 */
	double temp;
	if(!flag)
    temp = (xi(0)-x0(0))*(xi(0)-x0(0))+(xi(1)-x0(1))*(xi(1)-x0(1))-radius*radius;
	else
    temp = radius*radius-((xi(0)-x0(0))*(xi(0)-x0(0))+(xi(1)-x0(1))*(xi(1)-x0(1)));
	return temp;
}

Vector2f FlockingAlgo::DfgCircle_xi(const Vector2f& xi,
                                    const Vector2f& x0,
                                    int flag)
{/*the first derivative of the function Circle Region
 @param[in] flag flag=0, inside the circle; flag=1, outside the circle
 */
  Vector2f temp;
	if(!flag){
    temp(0) = 2*(xi(0)-x0(0));
    temp(1) = 2*(xi(1)-x0(1));
	}
	else
	{
    temp(0) = -2*(xi(0)-x0(0));
    temp(1) = -2*(xi(1)-x0(1));
	}
	return temp;
}

Matrix2f FlockingAlgo::D2fgCircle_xi(const Vector2f& xi,
                                     const Vector2f& x0,
                                     int flag)
{//calculate the second order derivative if fgRect1 w.r.t. xi
  Matrix2f temp;
	if(!flag){
    temp << 2,0,
            0,2;
	}
	else{
    temp << -2,0,
            0,-2;
	}
	return temp;
}

double FlockingAlgo::glij(const Vector2f& x1,
                          const Vector2f& x2,
                          double cst = 0.){
  //calculate gl function
  //@parameter[in] cst : if cst == 0., use default region metric; else use user defined region.
  //@return scalar
  if (cst != 0.){
    return pow(cst,2.0) - (pow((x1(0) - x2(0)),2.0) +pow((x1(1) - x2(1)), 2.0));
  }
  else{
    return pow(default_radius_,2.0) - (pow((x1(0) - x2(0)),2.0) +pow((x1(1) - x2(1)), 2.0));
  }

}

Vector2f FlockingAlgo::Dglij_xij(const Vector2f& xi,
                                 const Vector2f& xj)
{//calculate the first order derivative of gl wrt xij
  Vector2f tempPT;
  tempPT(0)=xi(0)-xj(0);
  tempPT(1)=xi(1)-xj(1);
  tempPT(0)*=-2;
  tempPT(1)*=-2;
	return tempPT;
}

Matrix2f FlockingAlgo::D2glij_xij(const Vector2f& xi,
                                  const Vector2f& xj)
{//calculate the second order derivative of gl wrt xij
  Matrix2f tempMat;
  tempMat<< -2, 0,
            0, -2;

	return tempMat;
}

double FlockingAlgo::caldistance(const Vector2f & xi,
                                 const Vector2f & xj)
{//calculate the Euclidian distance between xi and xj
  return sqrt(pow((xi(0)-xj(0)),2.0) + pow((xi(1) - xj(1)), 2.0));
}

Vector2f FlockingAlgo::calInputKinemicRing(const Vector2f& Xi,
                                           const Vector2f& Vi,
                                           const Vector2f& X0,
                                           const Vector2f& V0,
                                           const Vector2f& Xz,
                                           const Vector2f& Vz)
{/*!A new method for calculating the kinematic input of Circular Region. 
    The local potential function has been changed to sum(max(0,gij)^3(k/abs(delta qij))
	An imitation of macro robot about the maximum velocity available. 
  @param[in]  Xi  the position of the ith uav.
  @param[in]  Vi  the velocity of the ith uav.

  @param[in]  X0  the position of the target.
  @param[in]  V0  the velocity of the target.

  @param[in]  Xz  the position of the obstacle.
  @param[in]  Vz  the velocity of the obstacle.
 */
  Vector2f first_order_inside_circlc, first_order_outside_circle;
  Matrix2f second_order_inside_circle, second_order_outside_circle;

  Vector2f tempPT4, iput;
  double dst, Aij;
  double tmp1;//, tmp2, tmp3, tmp4; //for the calculation of ddelta_roij
	double tempDist;
  Vector2f delta_roij, ddelta_roij, delta_kexi, ddelta_kexi;

  delta_kexi(0)=0.;
  delta_kexi(1)=0.;

  ddelta_kexi(0) = 0.;
  ddelta_kexi(1) = 0.;

  //first_order_inside_circlc inside circle; first_order_outside_circle outside circle.
  first_order_inside_circlc = DfgCircle_xi(Xi,X0,0);
  first_order_outside_circle = DfgCircle_xi(Xi,X0,1);
  //attractive region first order derivative
  //m_radius_ is user defined radius, here m_radius_ == 100; And the ring width is 5
  delta_kexi(0)=k1_*max(0., fgCircle(Xi,X0,m_radius_,0))*first_order_inside_circlc(0)+k1_*max(0., fgCircle(Xi,X0,m_radius_-m_ringWidth_,1))*first_order_outside_circle(0); //Dfi_x is the derivative of fi with respect to x;
  delta_kexi(1)=k1_*max(0., fgCircle(Xi,X0,m_radius_,0))*first_order_inside_circlc(1)+k1_*max(0., fgCircle(Xi,X0,m_radius_-m_ringWidth_,1))*first_order_outside_circle(1);

  //second_order_inside_circle inside circle; second_order_outside_circle outside circle.
  second_order_inside_circle = D2fgCircle_xi(Xi,X0,0);
  second_order_outside_circle = D2fgCircle_xi(Xi,X0,1);
  //if outside the out circle
  if(fgCircle(Xi,X0,m_radius_,0)>0){
    ddelta_kexi(0)=k1_*(fgCircle(Xi,X0,m_radius_,0)*(second_order_inside_circle(0,0)*(Vi(0)-V0(0))+second_order_inside_circle(0,1)*(Vi(1)-V0(1)))
      +first_order_inside_circlc(0)*first_order_inside_circlc(0)*(Vi(0)-V0(0))+first_order_inside_circlc(0)*first_order_inside_circlc(1)*(Vi(1)-V0(1)));
    //	ddelta_kexi(0)=second_order_inside_circle(0,0)*k1_*fgRect1(Xi,X0)*(Vi(0)-V0(0))-4*k1_*(Xi(0)-X0(0))*(Xi(0)-X0(0))*(Vi(0)-V0(0));
    ddelta_kexi(1)=k1_*(fgCircle(Xi,X0,m_radius_,0)*(second_order_inside_circle(1,0)*(Vi(0)-V0(0))+second_order_inside_circle(1,1)*(Vi(1)-V0(1)))
      +first_order_inside_circlc(1)*first_order_inside_circlc(1)*(Vi(1)-V0(1))+first_order_inside_circlc(0)*first_order_inside_circlc(1)*(Vi(0)-V0(0)));
	}
  //if inside the out circle
	else{
    ddelta_kexi(0)=0;
    ddelta_kexi(1)=0;
	}
  //if inside the internal circle;Attention: the ring width is 15 here
  if(fgCircle(Xi,X0,m_radius_-m_ringWidth_,1)>0){
    ddelta_kexi(0)+=k1_*(fgCircle(Xi,X0,m_radius_-5,1)*(second_order_outside_circle(0,0)*(Vi(0)-V0(0))+second_order_outside_circle(0,1)*(Vi(1)-V0(1)))
      +first_order_outside_circle(0)*first_order_outside_circle(0)*(Vi(0)-V0(0))+first_order_outside_circle(0)*first_order_outside_circle(1)*(Vi(1)-V0(1)));  //calculate Ddelta_kexi with respect to time;
    ddelta_kexi(1)+=k1_*(fgCircle(Xi,X0,m_radius_-5,1)*(second_order_outside_circle(1,0)*(Vi(0)-V0(0))+second_order_outside_circle(1,1)*(Vi(1)-V0(1)))
      +first_order_outside_circle(1)*first_order_outside_circle(1)*(Vi(1)-V0(1))+first_order_outside_circle(0)*first_order_outside_circle(1)*(Vi(0)-V0(0)));
	}
  //if outside the internal circle
	else{
    ddelta_kexi(0)=0;
    ddelta_kexi(1)=0;
	}


  //Here need to find neighbour obstacles.
//	neighbour_num=findNeighbour(AMatrix, Neighbour, index);
  delta_roij(0)=0;
  delta_roij(1)=0;

  ddelta_roij(0)=0;
  ddelta_roij(1)=0;
	
  //load obstacle_position from obstacle_cur_pose;
	//obstacles avoidance--------------------------------------------//

  //-----------------------------------------//
  //Here need to determine obstacle_radius
  //-----------------------------------------//

  tempDist = glij(Xi,Xz,obstacle_radius_+d_/2);

  Vector2f position_error(0.,0.);
  Matrix2f second_order_obstacle;
  second_order_obstacle << 0., 0., 0., 0.;

//  V0(0)=0; //X0 velocity x
//  V0(1)=0; //X0 velocity y
  position_error(0)=Xi(0)-Xz(0);  //relative position error between Xi and obstacle in x
  position_error(1)=Xi(1)-Xz(1);  //relative position error between Xi and obstacle in y
  second_order_obstacle=D2glij_xij(Xi,Xz); //second order derivative of glij

  //dst need to calculate relative distance between obstacle and current pose
  dst = caldistance(Xi, Xz);
  Aij=6*max(0., tempDist)*max(0., tempDist)*(kl_/dst)
    +max(0., tempDist)*max(0., tempDist)*max(0., tempDist)*kl_/dst/dst/dst;

  delta_roij(0)=delta_roij(0)-kij_*Aij*position_error(0);
  delta_roij(1)=delta_roij(1)-kij_*Aij*position_error(1);

  //----------what does the tmp1 refer to ?---------------//
  tmp1=24*max(0.,tempDist)*(kl_/dst)
    +12*max(0.,tempDist)*max(0.,tempDist)*kl_/dst/dst/dst
    +3*max(0., tempDist)*max(0., tempDist)*max(0., tempDist)*kl_/dst/dst/dst/dst/dst;

  tempPT4(0)=position_error(0)*position_error(0)*(Vi(0)-Vz(0))+position_error(0)*position_error(1)*(Vi(1)-Vz(1));
  tempPT4(1)=position_error(1)*position_error(1)*(Vi(1)-Vz(1))+position_error(0)*position_error(1)*(Vi(0)-Vz(0));

  tempPT4(0)=kij_*(tmp1*tempPT4(0)-Aij*(Vi(0)-Vz(0)));
  tempPT4(1)=kij_*(tmp1*tempPT4(1)-Aij*(Vi(0)-Vz(0)));

  ddelta_roij(0)=ddelta_roij(0)+tempPT4(0);  //calculate Ddelta_roij with respect to time
  ddelta_roij(1)=ddelta_roij(1)+tempPT4(1);
//		}
//	}

  Vector2f delta_ei(0., 0.), ddelta_ei(0., 0.), dXri(0., 0.), d2Xri(0., 0.),
      si(0., 0.), Yi(0., 0.), thetai(0., 0.);
  Vector2f A0(0., 0.);//here set A0 to zero to nullify acceleration item

  delta_ei(0)=afai_*delta_kexi(0)+gama_*delta_roij(0);
  delta_ei(1)=afai_*delta_kexi(1)+gama_*delta_roij(1);
  ddelta_ei(0)=afai_*ddelta_kexi(0)+gama_*ddelta_roij(0);
  ddelta_ei(1)=afai_*ddelta_kexi(1)+gama_*ddelta_roij(1);

  dXri(0)=Vz(0)-delta_ei(0);
  dXri(1)=Vz(1)-delta_ei(1);
  d2Xri(0)=A0(0)-ddelta_ei(0);
  d2Xri(1)=A0(1)-ddelta_ei(1);

  si(0)=Vi(0)-dXri(0);
  si(1)=Vi(1)-dXri(1);

  Yi(0,0)=d2Xri(0);
  Yi(1,0)=d2Xri(1);
  Yi(0,1)=dXri(0);
  Yi(1,1)=dXri(1);

  thetai(0)=Mi_;
  thetai(1)=betai_;
    //%thetai_est(:,i)=-Li*Yi'*si*T+thetai_est(:,i)
	//calculate the input
  iput(0)=-Ksi_(0,0)*si(0)-kp_*delta_ei(0)+Yi(0,0)*thetai(0)+Yi(0,1)*thetai(1);
  iput(1)=-Ksi_(1,1)*si(1)-kp_*delta_ei(1)+Yi(1,0)*thetai(0)+Yi(1,1)*thetai(1);
	
	float norm;
    norm=sqrt(iput(0)*iput(0)+iput(1)*iput(1));
	if(norm==0)
    return Vector2f(0., 0.);
   if(norm>satu_thres_){
    iput(0)=iput(0)/norm*satu_thres_;
    iput(1)=iput(1)/norm*satu_thres_;
	}

  Vector2f incre_v, incre_x;
  incre_v(0)=Vi(0)+iput(0)*T_;
  incre_v(1)=Vi(1)+iput(1)*T_;
  incre_x(0)=Xi(0)+0.5*iput(0)*T_*T_+Vi(0)*T_;
  incre_x(1)=Xi(1)+0.5*iput(1)*T_*T_+Vi(1)*T_;

  incre << incre_x(0), incre_x(1);

  return incre;
}
