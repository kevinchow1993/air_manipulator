// FormationAlgorithmRegion.h: interface for the CFFlockingAlgorithm class.
//
//////////////////////////////////////////////////////////////////////

#ifndef FLOCKING_ALGO
#define FLOCKING_ALGO

#include <iostream>
#include <algorithm>
#include <math.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class FlockingAlgo
{
public:

  FlockingAlgo();
  virtual ~FlockingAlgo(){}

//---------------------Form a circle: Functions and Parameters-----------------//
  Vector2f  calInputKinemicRing(const Vector2f& Xi, const Vector2f& Vi,
                            const Vector2f& X0, const Vector2f& V0,
                            const Vector2f& Xz, const Vector2f& Vz);

  Vector2f DfgCircle_xi(const Vector2f& xi, const Vector2f& x0, int flag);
  Matrix2f D2fgCircle_xi(const Vector2f& xi, const Vector2f& x0,int flag);
  double   fgCircle(const Vector2f& xi, const Vector2f& x0, int flag);
  double   fgCircle(const Vector2f& xi, const Vector2f& x0, double radius, int flag);

//  double fgForward(const Vector2f& xi,
//                   const double& theta,
//                   const double t_x,
//                   const double t_y);
//  double DfgForward(const Vector2f& xi,
//                    const double & theta,
//                    const double t_x,
//                    const double t_y);
//  double D2fgForward(const Vector2f& xi,
//                    const double & theta,
//                    const double t_x,
//                    const double t_y);

  double glij(const Vector2f& , const Vector2f&, double cst);
  Matrix2f D2glij_xij(const Vector2f& xi, const Vector2f& xj);
  Vector2f Dglij_xij(const Vector2f& xi, const Vector2f& xj);
  double caldistance(const Vector2f&, const Vector2f&);
  void updatePara(const double &, const double &, const double &, const double &,
                  const double &, const double &, const double &, const double &,
                  const double &, const double &, const double &, const double &,
                  const double &, const double &, const double &
                  );

  Vector2f incre;

private:
  Vector2f thetai_;
  float satu_thres_;   //saturation parameter
  float kp_,kij_,k1_,kl_,gama_,afai_;
  float default_radius_,d_;
  double T_; //period time
  double obstacle_radius_;
  float Mi_;    //the inertia mass of the agent
  float betai_; //all the particles have the same Mi and betai
  int m_regionNum_;  //number of regions
  double   m_radius_; //the radius of circle/ring region
  double   m_ringWidth_; //the width of the ring

  Matrix2f Ksi_;

};

#endif
