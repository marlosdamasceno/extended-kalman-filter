#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
    /** predict the state */
    //Lesson 6. Section 13.
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /** update the state by using Kalman Filter equations */
    //Lesson 6. Section 7.
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    //New state
    x_ = x_ + (K * y);
    MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /** update the state by using Extended Kalman Filter equations*/
    //Lesson 6. Section 14.
    double x_value = x_(0);
    double y_value = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    double rho = sqrt(x_value*x_value + y_value*y_value);
    double theta = atan2(y_value,x_value);
    double ro_dot = (x_value*vx+y_value*vy)/rho;
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, theta, ro_dot;

    //Lesson 6. Section 7.
    VectorXd y = z - z_pred;
    //Normalizing the angle
    while(y(1) > M_PI || y(1) < -M_PI) {
        if (y(1) > M_PI) {
            y(1) -= M_PI;
        }
        else {
            y(1) += M_PI;
        }
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    //New state
    x_ = x_ + (K * y);
    MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I_ - K * H_) * P_;
}
