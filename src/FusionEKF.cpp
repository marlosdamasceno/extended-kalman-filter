#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    /** * Finish initializing the FusionEKF.
        * Set the process and measurement noises */
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.F_= MatrixXd(4, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    //H matrix for laser. Lesson 6. Section 10.
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    //noises, in the class is indicated to use 9
    noise_ax = 9.0;
    noise_ay = 9.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /** * Initialize the state ekf_.x_ with the first measurement.
            * Create the covariance matrix.
            * Remember: you'll need to convert radar from polar to cartesian coordinates.*/
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /** Convert radar from polar to cartesian coordinates and initialize state.*/
            ekf_.x_(0) = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]); //rho * cos(phi)
            ekf_.x_(1) = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]); //rho * sin(phi)
            ekf_.x_(2) = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]); //rho's velocity * cos(phi)
            ekf_.x_(3) = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]); //rho's velocity * sin(phi)
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /** Initialize state.*/
            //Lesson 6. Section 13.
            ekf_.x_(0) = measurement_pack.raw_measurements_[0]; //x
            ekf_.x_(1) = measurement_pack.raw_measurements_[1]; //y
            ekf_.x_(2) = 0;
            ekf_.x_(3) = 0;
        }
        ekf_.F_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
	//Changing the P, from 1 to 0.01 and from 1000 to 1, generates a much lower RMSE
        ekf_.P_ << 0.01, 0, 0, 0,
                0, 0.01, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        previous_timestamp_ =  measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    /** * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
        * Update the process noise covariance matrix.
        * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    //Lesson 6. Section 13.
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ =  measurement_pack.timestamp_;
    double dt2 = dt*dt;
    double dt3 = dt2*dt;
    double dt4 = dt3*dt;
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
            0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
            dt3/2*noise_ax, 0, dt2*noise_ax, 0,
            0, dt3/2*noise_ay, 0, dt2*noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    /** * Use the sensor type to perform the update step.
        * Update the state and covariance matrices. */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
