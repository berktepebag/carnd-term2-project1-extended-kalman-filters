## Extended Kalman Filter Project Code

### Self-Driving Car Engineer Nanodegree Program

System:
DELL XPS 13 9350, Windows 10 Bash on Ubuntu (Linux Subsystem)

The goals / steps of this project are the following:

Utilize a kalman filter to find the state of a moving object according to radar and lidar measurements. Calculate RMSE according to ground-truth data. Project steps:

* Code the Predict and Update functions to successfully build a Kalman filter.
* Code the RMSE and Jacobian functions.
* Code the KalmanFilter and ExtendedKalmanFilter functions to calculate predict and update states.
* Initialize the state vectors and covariance matrices using the first measurements.
* Predict the object position to current timestep and after recieving measurement update the prediction.
* Call correct measurment function (linear (laser), non-linear (radar)) according to sensor type.

### Project Setup: 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Step 1: Install Windows 10 Bash on Ubuntu. Follow the link for a nice [guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) 

Step 2: Install [mobaxterm](https://mobaxterm.mobatek.net/) so you can use Sublime (or any other text editor) inside the Bash. 

Step 3: Instead of Step 1 and 2 you can follow this [thread](https://nickjanetakis.com/blog/using-wsl-and-mobaxterm-to-create-a-linux-dev-environment-on-windows#wsl-conemu-and-mobaxterm-to-the-rescue) which also explains how to install Sublime, a nice text editor to be used on Ubuntu.

#### Data Set 1 Full Run 
<img width="500" alt="Data Set 1: Full Run" src="/images/extended%20kalman%20filter%20result.JPG">

#### Data Set 1 Detail
<img width="500" alt="Data Set 1: Detail" src="/images/extended%20kalman%20filter%20detail.JPG">

#### Data Set 1 Fail
<img width="500" alt="What Is Going On?" src="/images/What_is_going_on.jpg">
Why is this happening? 

Altough we have proper radar and lidar data at the curvature we are exceeding 2xPi at Y direction. So we should be limiting the maximum angle between -Pi and Pi with:

```
if (Y(1) *180/pi > 360){
		Y(1) = 0.001;
	}
```

#### Data Set 2 Full Run 
<img width="500" alt="Data Set 2: Full Run" src="/images/extended%20kalman%20filter%20detail%20data2.JPG">
