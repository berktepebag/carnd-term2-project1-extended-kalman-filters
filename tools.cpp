#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	//cout << "est size: "<< estimations.size() << endl;
	assert(estimations.size() > 0 && "Estimation vector cannot be zero");
	assert(estimations.size() == ground_truth.size()  && "Estimation and Ground Truth vector cannot be different");

	VectorXd residual;
	VectorXd residual_square;
	VectorXd total;

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		
		//cout << "est: " << estimations[i] << endl;
		residual = (estimations[i] - ground_truth[i]);
		//cout << "residuals " << i <<" : " << residual << endl;
		residual_square = residual.array()*residual.array();
		//cout << "residual_square " << i <<" : " << residual_square << endl;
		rmse += residual_square;
		//cout << "total " << i <<" : " << rmse << endl;
	}	
	//calculate the mean
	rmse /= estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	//cout <<"We are in CalculateJacobian" << endl;
	MatrixXd Hj(3,4);
	//recover state parameters
	//cout << "x_state size: " << x_state.size() << endl;
	float px = x_state(0);
	//cout << "px: " << px << endl;
	float py = x_state(1);
	//cout << "py: " << py << endl;
	float vx = x_state(2);
	//cout << "vx: " << vx << endl;
	float vy = x_state(3);
	//cout << "vy: " << vy << endl;

	//TODO: YOUR CODE HERE 
	//check division by zero
	assert(px != 0 && py != 0 && "Division by zero!");
	//cout <<"No assert!" << endl;
	//compute the Jacobian matrix
	Hj << (px/sqrt(pow(px,2)+pow(py,2))) , (py/sqrt(pow(px,2)+pow(py,2))) , 0 ,0 ,
	(-py/(pow(px,2)+pow(py,2))) , (px/(pow(px,2)+pow(py,2))) , 0 , 0 ,
	(py*(vx*py-vy*px))/((pow(pow(px,2)+pow(py,2),(3.0/2.0)))) , (px*(-vx*py+vy*px))/((pow(pow(px,2)+pow(py,2),(3.0/2.0)))),
	(px/sqrt(pow(px,2)+pow(py,2))) , (py/sqrt(pow(px,2)+pow(py,2))); 
	//cout <<"We are in CalculateJacobian Hj: "<<Hj << endl;

	return Hj;
}
