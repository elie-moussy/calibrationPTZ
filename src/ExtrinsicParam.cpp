/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#include "calibrationPTZ/ExtrinsicParam.hh"

#define PI 3.14159265

//\fn ExtrinsicParam::ExtrinsicParam(int cam);
///\brief The constructor of the class "ExtrinsicParam".
///\param cam Indicates the camera number (1 or 2).
ExtrinsicParam::ExtrinsicParam(int cam):
  pan(0.),
  tilt(0.),
  rotation_computed(false)
{
  this->cam = cam;
  calib = new Calibration(cam);
  dist.resize(5);
  K.setZero();
  if (cam == 2)
    {
      K(0,0) = 800;
      K(1,1) = 600;
      K(2,2) = 1;
      K(0,2) = 399.92;
      K(1,2) = 234.501;
      dist << -0.113323,
	-0.023496,
	-0.013891,
	-0.003838,
	0.267853;
    } else if (cam == 1)
    {
      K(0,0) = 800;
      K(1,1) = 600;
      K(2,2) = 1;
      K(0,2) = 322.114;
      K(1,2) = 217.173;
      dist << -0.133553,
	0.078593,
	0.001123,
	0.001457,
	-0.043746;
    }
  rotation.setZero();
  translation.setZero();
}

//\fn ExtrinsicParam::~ExtrinsicParam();
///\brief The destructor of the class "ExtrinsicParam".
ExtrinsicParam::~ExtrinsicParam()
{
  delete calib;
}

//\fn Eigen::Vector3d ExtrinsicParam::getTranslationVector();
///\brief A getter function of the translation vector.
///\return The 3D vector of translation.
Eigen::Vector3d ExtrinsicParam::getTranslationVector()
{
  return translation;
}

//\fn Eigen::Matrix3d ExtrinsicParam::getRotationMatrix();
///\brief A getter function of the rotation matrix.
///\return The 3x3 matrix of rotation.
Eigen::Matrix3d ExtrinsicParam::getRotationMatrix()
{
  if (rotation_computed)
    return rotation;

  Eigen::Matrix3d zero;
  zero.fill(0.);
  return zero;
}

//\fn Eigen::Matrix3d ExtrinsicParam::getCameraMatrix();
///\brief A getter function of the camera matrix (the intrinsic parameters).
///\return The a 3x3 matrix of the camera calibration.
Eigen::Matrix3d ExtrinsicParam::getCameraMatrix()
{
  return K;
}

//\fn void ExtrinsicParam::computeRtMatrix(double pan, double tilt, cv::Mat image);
///\brief This function computes the rotation matrix and the translation vector of the extrinsic parameters.
///\param pan Value of the camera panoramique when the image has been taken.
///\param tilt Value of the camera tilt when the image has been taken.
///\param image The image from which the rotation matrix and the translation vector should be computed.
void ExtrinsicParam::computeRtMatrix(double pan, double tilt, cv::Mat image)
{
  // Save the value of the pan and the tilt in order to modify the extrinsic parameters when the camera position change
  this->pan = pan;
  this->tilt = tilt;
  cv::Mat img = image.clone();
  cv::Mat initImg = image.clone();
  int cnt = 0;

  double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
  double u, v, x, y;
  double dist_to_point;

  pp.cnt = 0;

  // Create a window of the scene
  cv::namedWindow("Extrinsic Image",CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback("Extrinsic Image",On_Mouse,0);
  cv::waitKey(10);
  cv::imshow("Extrinsic Image", img);
  cv::waitKey(10);

  // We need 4 points to compute the translation vector and the rotation matrix
  while (pp.cnt <= 4)
    {
      if (cnt > pp.cnt) // Case where we do a right click in order to remove the last point inserted
	{
	  switch (cnt)
	    {
	    case 1:
	      {
		img = image.clone();
		cnt = pp.cnt;
	      }
	      break;
	      
	    case 2:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 3:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0.5,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 4:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[0],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0.5,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[2],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0.5,0)",pp.p1[2],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    default:
	      break;
	    }
	  cv::imshow("Extrinsic Image", img);
	  cv::waitKey(10);
	}
      if (pp.clicked) // Case where we do a left click in order to insert a point
	{
	  cv::circle(img,pp.p1[pp.cnt-1],3,cv::Scalar(255,0,0));
	  
	  if (pp.cnt == 1) // First point to insert (0,0,0) (on the mocap basis)
	    {
	      cv::putText(img,"(0,0,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      // Get the image coordinates
	      u = (pp.p1[0].x - cx)/fx;
	      v = (pp.p1[0].y - cy)/fy;
	      
	      // Get the distance between the camera and the 3d point (the scale s)
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_000_CAM1;
		}
	      else if (cam == 2)
		{
		  dist_to_point= DIST_TO_000_CAM2;
		}
	      
	      // The first point inserted will help to compute the translation vector
	      translation(0) = (u)*(dist_to_point);
	      translation(1) = (v)*(dist_to_point);
	      translation(2) = dist_to_point;

	    } else if (pp.cnt == 2) // Second point to insert (500mm,0,0) (on the mocap basis)
	    {
	      cv::putText(img,"(0.5,0,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = (pp.p1[1].x - cx)/fx;
	      v = (pp.p1[1].y - cy)/fy;
	      
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_0500_CAM1;
		}
	      if (cam == 2)
		{
		  dist_to_point = DIST_TO_0500_CAM2;
		}

	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      rotation(0,0) = (x - translation(0)) / 500.;
	      rotation(1,0) = (y - translation(1)) / 500.;
	      rotation(2,0) = (dist_to_point - translation(2)) / 500.;

	    } else if (pp.cnt == 3) // Third point to insert (0,500mm,0) (on the mocap basis)
	    {
	      cv::putText(img,"(0,0.5,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = (pp.p1[2].x - cx)/fx;
	      v = (pp.p1[2].y - cy)/fy;
	      
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_0050_CAM1;
		}
	      if (cam == 2)
		{
		  dist_to_point = DIST_TO_0050_CAM2;
		}
	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      rotation(0,1) = (x - translation(0)) / 500.;
	      rotation(1,1) = (y - translation(1)) / 500.;
	      rotation(2,1) = (dist_to_point - translation(2)) / 500.;

	    } else if (pp.cnt == 4) // Fourth point to insert (0,0,1000mm) (on the mocap basis)
	    {
	      cv::putText(img,"(0,0,1)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = (pp.p1[3].x - cx)/fx;
	      v = (pp.p1[3].y - cy)/fy;
	      
	      if (cam == 1)
		{
		  dist_to_point = DIST_TO_001_CAM1;
		}
	      if (cam == 2)
		{
		  dist_to_point = DIST_TO_001_CAM2;
		}

	      x = (u)*(dist_to_point);
	      y = (v)*(dist_to_point);

	      rotation(0,2) = (x - translation(0)) / 1000.;
	      rotation(1,2) = (y - translation(1)) / 1000.;
	      rotation(2,2) = ((dist_to_point) - translation(2)) / 1000.;
	    }
	  cnt = pp.cnt;
	  pp.clicked = false;
	}
      // Keep showing the image and the modification on it
      cv::imshow("Extrinsic Image", img);
      cv::waitKey(10);
    }

  rotation_computed = true;

  // Destroy the window
  cv::destroyWindow("Extrinsic Image");
}

//\fn void ExtrinsicParam::changePanTilt(double pan, double tilt);
///\brief This function computes the new rotation matrix and the new translation vector of the extrinsic parameters when the camera has changed its position.
///\param pan Value of the new camera panoramique.
///\param tilt Value of the new camera tilt.
void ExtrinsicParam::changePanTilt(double pan, double tilt)
{
  // Compute the rotation matrices with the new values of pan and tilt
  Eigen::Matrix3d Rx, Ry;
  Rx.setZero();
  Ry.setZero();
  Rx(0,0) = 1;
  Rx(1,1) = cos((-(tilt-this->tilt))*PI/180.);
  Rx(1,2) = -sin((-(tilt-this->tilt))*PI/180.);
  Rx(2,1) = sin((-(tilt-this->tilt))*PI/180.);
  Rx(2,2) = cos((-(tilt-this->tilt))*PI/180.);
  Ry(0,0) = cos((-(pan-this->pan))*PI/180.);
  Ry(0,2) = sin((-(pan-this->pan))*PI/180.);
  Ry(1,1) = 1;
  Ry(2,0) = -sin((-(pan-this->pan))*PI/180.);
  Ry(2,2) = cos((-(pan-this->pan))*PI/180.);

  // Compute the new values of the extrinsic parameters
  Eigen::Matrix4d Rx1, Ry1, Rt;
  Rt << rotation, translation,
    0,0,0,1;
  Rx1 << Rx, Eigen::Vector3d::Zero(),
    0,0,0,1;
  Ry1 << Ry, Eigen::Vector3d::Zero(),
    0,0,0,1;
  Rt.noalias() = Rx1*Ry1*Rt;
  rotation(0,0) = Rt(0,0);rotation(0,1) = Rt(0,1);rotation(0,2) = Rt(0,2);
  rotation(1,0) = Rt(1,0);rotation(1,1) = Rt(1,1);rotation(1,2) = Rt(1,2);
  rotation(2,0) = Rt(2,0);rotation(2,1) = Rt(2,1);rotation(2,2) = Rt(2,2);
  translation(0) = Rt(0,3);
  translation(1) = Rt(1,3);
}

//\fn void ExtrinsicParam::getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z);
///\brief This function computes the coordinates of a 3D point from the real landmark to the one of the camera.
///\param realP Value of the 3D point in the real landmark.
///\param x Value of the 3D point in the camera landmark and on the x axis.
///\param y Value of the 3D point in the camera landmark and on the y axis.
///\param z Value of the distance between the camera and the 3D point (the scale).
void ExtrinsicParam::getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z)
{
  Eigen::Vector4d real;
  real << realP,
          1;
  double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);  
  Eigen::Vector3d imgP;
  Eigen::MatrixXd Rt;
  Rt.resize(3,4);
  Rt << rotation,translation;
  imgP.noalias() = K*Rt*real;
  z = imgP(2);
  imgP.noalias() = imgP/imgP(2);
  imgP.noalias() = undistortPoint(imgP);
  imgP(0) = imgP(0)*fx + cx;
  imgP(1) = imgP(1)*fy + cy;
  x = imgP(0);
  y = imgP(1);
}

//\fn Eigen::Vector3d ExtrinsicParam::undistortPoint(Eigen::Vector3d distortedPoint);
///\brief This function undistorts a distorted point on the image coordinates. After calling this function, the first value of the returned vector should be multiplied by fx and added to cx and the second value should be multiplied by fy and added to cy.
///\param distortedPoint the distorted point to undistort.
///\return The value of the undistorted point in the image coordinates.
Eigen::Vector3d ExtrinsicParam::undistortPoint(Eigen::Vector3d distortedPoint)
{
  Eigen::Vector3d res;
  
  // Convert the camera matrix to an OpenCV matrix
  cv::Mat K1(3,3,CV_64F);
  K1.at<double>(0,0) = K(0,0);
  K1.at<double>(0,1) = K(0,1);
  K1.at<double>(0,2) = K(0,2);
  K1.at<double>(1,0) = K(1,0);
  K1.at<double>(1,1) = K(1,1);
  K1.at<double>(1,2) = K(1,2);
  K1.at<double>(2,0) = K(2,0);
  K1.at<double>(2,1) = K(2,1);
  K1.at<double>(2,2) = K(2,2);

  // Convert the distortion vector to an OpenCV vector
  cv::Mat distCoeffs(5,1,CV_64F);
  distCoeffs.at<double>(0,0) = dist(0);
  distCoeffs.at<double>(1,0) = dist(1);
  distCoeffs.at<double>(2,0) = dist(2);
  distCoeffs.at<double>(3,0) = dist(3);
  distCoeffs.at<double>(4,0) = dist(4);

  // Convert the distorted point vector to an OpenCV vector
  cv::Mat src(1,1,CV_64FC2);
  src.at<double>(0,0) = distortedPoint(0);
  src.at<double>(0,1) = distortedPoint(1);
  cv::Mat dst;

  // Undistort the point
  cv::undistortPoints(src, dst, K1, distCoeffs);

  // Fill the undistorted point vector and return it
  res(0) = dst.at<double>(0,0);
  res(1) = dst.at<double>(0,1);
  res(2) = 1.;
  return res;
}

//\fn Eigen::Vector3d ExtrinsicParam::distortPoint(Eigen::Vector3d undistortedPoint);
///\brief This function distorts an undistorted point on the image coordinates. Before calling this function, the first value of "undistortedPoint" vector should be minused by cx and the whole divided by fx. Besides, the second value of this vector should be minused by cy and the whole divided by fy.
///\param undistortedPoint The undistorted point to distort.
///\return The value of the distorted point in the image coordinates.
Eigen::Vector3d ExtrinsicParam::distortPoint(Eigen::Vector3d undistortedPoint)
{
  Eigen::Vector3d res = undistortedPoint;
  double r2 = res(0)*res(0) + res(1)*res(1);
  double k1 = dist(0), k2 = dist(1), p1 = dist(2), p2 = dist(3), k3 = dist(4);

  // radial distorsion
  res(0) = undistortedPoint(0) * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);
  res(1) = undistortedPoint(1) * (1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2);

  // tangential distorsion
  res(0) += 2.*p1*undistortedPoint(1)*undistortedPoint(0) + p2*(r2 + 2.*undistortedPoint(0)*undistortedPoint(0));
  res(1) += p1*(r2 + 2.*undistortedPoint(1)*undistortedPoint(1)) + 2.*p2*undistortedPoint(0)*undistortedPoint(1);

  // ideal coordinates => actual coordinates
  res(0) = res(0)*K(0,0) + K(0,2);
  res(1) = res(1)*K(1,1) + K(1,2);
  
  return res;
}
