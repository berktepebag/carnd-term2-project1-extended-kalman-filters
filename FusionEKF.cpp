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
	is_initialized_ = false;

	previous_timestamp_ = 0;

  // initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
							0, 0.0225;

  //measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
							0, 0.0009, 0,
							0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	H_laser_ << 1 , 0 , 0 , 0,
							0, 1 , 0 ,0 ;

	ekf_.F_ = MatrixXd(4,4);
	ekf_.F_ << 1 , 0 , 1 , 0,
						0, 1, 0 , 1,
						0, 0 , 1, 0,
						0, 0 ,0 , 1;

	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 1,0,0,0,
						0,1,0,0,
						0,0,1000,0,
						0,0,0,1000;

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
		//cout << "not initialized, initializing" << endl;
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
		//cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1; //Importan for RMSE, can cause high RMSE at the beginning.
    //cout << "x assigned" << endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */     
    	//cout << "calculating ro, theta" << endl; 
    	float ro = measurement_pack.raw_measurements_(0);
    	float theta = measurement_pack.raw_measurements_(1);

    	ekf_.x_(0) = ro * cos(theta);
    	ekf_.x_(1) = ro * sin(theta);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	//cout << "calculating x,y" << endl; 
    	ekf_.x_(0) = measurement_pack.raw_measurements_(0);
    	//cout << "ekf_.x_(0): " << ekf_.x_(0) << endl; 
    	ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    	//cout << "ekf_.x_(1): " << ekf_.x_(1) << endl;
    }
    //cout << "calculating previous_timestamp_" << endl; 
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.F_ << 1 , 0 , 0 , 0,
						    0, 1, 0 , 0,
						    0, 0 , 1, 0,
						    0, 0 ,0 , 1;
    //cout << "calculated previous_timestamp_: " << previous_timestamp_ << endl; 
    //cout << "ekf_.F_: " << ekf_.F_ << endl;

    // done initializing, no need to predict or update
    //cout << "initialized " << endl;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //cout << "Prediction Started" << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  MatrixXd G_ = MatrixXd(4, 2);
  G_ << (dt*dt)/2 , 0,
  			0, (dt*dt)/2,
  			dt, 0,
  			0, dt;

  MatrixXd G_t = G_.transpose();
	////cout << G_ << endl; 
  MatrixXd Qv = MatrixXd(2,2);
	Qv << noise_ax , 0,    
	        0 , noise_ay;

	ekf_.Q_ = G_* Qv * G_t;
	////cout << kf_.Q_ << endl;


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "Radar Update Started" << endl;

  	//Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    //cout << "Radar Update Hj_" << endl;
    ekf_.H_ = Hj_;
    //cout << "Radar Update R_radar_" << endl;
    ekf_.R_ = R_radar_;
    //cout << "ekf_.R_ updated." << endl;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //cout << "Radar Update Ends" << endl;

  } else {
    // Laser updates
    //cout << "Laser Update" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);

  }
  //cout << "Prediction Ends" << endl;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
