#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   predict the state
  **/
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
     update the state by using Kalman Filter equations
  **/

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.rows();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*
   * update the state by using Extended Kalman Filter equations
  */
	float rho_pred    =  pow(pow(double(x_[0]),2) + pow(double(x_[1]),2),0.5);
	float phi_pred    =  0.0;
	if (fabs(double(x_[0])) > 0.01) {
	   phi_pred  = atan2(double(x_[1]),double(x_[0]));
	}		float rhodot_pred = 0.0;
	if (fabs(rho_pred) > 0.01) {
		 rhodot_pred = (double(x_[0])*double(x_[2]) + double(x_[1])*double(x_[3])) / rho_pred;
	}
	VectorXd z_pred(3);
	z_pred << rho_pred, phi_pred, rhodot_pred;

	VectorXd y = z - z_pred;
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.rows();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
