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

[//]: # (Image References)
[image1]: ./images/extended kalman filter result.JPG "Data Set 1: Full Run"
[image2]: ./images/extended kalman filter detail.JPG "Data Set 1: Detail"
[image3]: ./images/extended kalman filter detail data2.JPG "Data Set 2: Full Run"

![alt text][image1]

In the third cell defined convert_color() to find out which color space fits best with the example images. Defined get_hog_features() using skimage.hog() and bin_spatial(), color_hist() definitions which we will feed into extract_features() to obtain hog features from it.



 


