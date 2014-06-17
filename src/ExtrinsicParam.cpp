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
  //K = calib->getCameraMatrix();
  K.setZero();
  if (cam == 2)
    {
      K(0,0) = 821;
      K(1,1) = 688;
      K(2,2) = 1;
      K(0,2) = 399.92;
      K(1,2) = 234.501;
    } else if (cam == 1)
    {
      K(0,0) = 800;//455;
      K(1,1) = 600;//355;
      K(2,2) = 1;
      K(0,2) = 322.114;
      K(1,2) = 217.173;
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
  this->pan = pan;
  this->tilt = tilt;
  cv::Mat img = image.clone();
  cv::Mat initImg = image.clone();
  int cnt = 0;

  //Eigen::Matrix3d K = calib->getCameraMatrix();
  double fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
  double k0;//= calib->getLensDistorsion();
  if (cam == 1)
    k0 = -0.08;
  if (cam == 2)
    k0 = 0.08;
  double u, v, x, y;

  pp.cnt = 0;

  cv::namedWindow("Extrinsic Image",CV_WINDOW_AUTOSIZE);
  cvSetMouseCallback("Extrinsic Image",On_Mouse,0);
  cv::waitKey(10);
  cv::imshow("Extrinsic Image", img);
  cv::waitKey(10);

  while (pp.cnt <= 4)
    {
      if (cnt > pp.cnt)
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
		cv::putText(img,"(0,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 3:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0.5,0,0)",pp.p1[2],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    case 4:
	      {
		img = image.clone();
		cv::circle(img,pp.p1[0],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0,0)",pp.p1[1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[1],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0.5,0,0)",pp.p1[2],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cv::circle(img,pp.p1[2],3,cv::Scalar(255,0,0));
		cv::putText(img,"(0,0.5,0)",pp.p1[3],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
		cnt = pp.cnt;
	      }
	      break;

	    default:
	      break;
	    }
	  cv::imshow("Extrinsic Image", img);
	  cv::waitKey(10);
	}
      if (pp.clicked)
	{
	  cv::circle(img,pp.p1[pp.cnt-1],3,cv::Scalar(255,0,0));
	  
	  if (pp.cnt == 1)
	    {
	      cv::putText(img,"(0,0,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[0].x;
	      v = pp.p1[0].y;
	      
	      if (cam == 1)
		{
		  double dist_to_point = DIST_TO_000_CAM1;//= getDistanceToPoint(pp.p1[0]);
		  translation(0) = (u-cx)*/*DIST_TO_000_CAM1*/(dist_to_point)/fx;
		  translation(1) = (v-cy)*/*DIST_TO_000_CAM1*/(dist_to_point)/fy;
		  translation(2) = dist_to_point;//DIST_TO_000_CAM1;
		}
	      if (cam == 2)
		{
		  double dist_to_point= DIST_TO_000_CAM2;//= getDistanceToPoint(pp.p1[0]);
		  translation(0) = (u-cx)*/*DIST_TO_000_CAM2*/dist_to_point/(fx);
		  translation(1) = (v-cy)*/*DIST_TO_000_CAM2*/dist_to_point/(fy);
		  translation(2) = dist_to_point;//DIST_TO_000_CAM2;
		}
	    } else if (pp.cnt == 2)
	    {
	      cv::putText(img,"(0.5,0,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[1].x - cx;
	      v = pp.p1[1].y - cy;
	      double uu, vu;
	      uu = u/(1. + k0);
	      vu = v/(1. + k0);
	      
	      if (cam == 1)
		{
		  double dist_to_point = DIST_TO_0500_CAM1;//getDistanceToPoint(pp.p1[1]);
		  x = (uu)*(dist_to_point)/*DIST_TO_0500_CAM1*//fx;
		  y = (vu)*(dist_to_point)/*DIST_TO_0500_CAM1*//fy;
		  rotation(0,0) = (x - translation(0)) / 500.;
		  rotation(1,0) = (y - translation(1)) / 500.;
		  rotation(2,0) = (/*DIST_TO_0500_CAM1*/dist_to_point - translation(2)) / 500.;
		}
	      if (cam ==2)
		{
		  double dist_to_point = DIST_TO_0500_CAM2;//getDistanceToPoint(pp.p1[1]);
		  x = (uu)*/*DIST_TO_0500_CAM2*/dist_to_point/fx;
		  y = (vu)*/*DIST_TO_0500_CAM2*/dist_to_point/fy;
		  rotation(0,0) = (x - translation(0)) / 500.;
		  rotation(1,0) = (y - translation(1)) / 500.;
		  rotation(2,0) = (/*DIST_TO_0500_CAM2*/dist_to_point - translation(2)) / 500.;
		}
	    } else if (pp.cnt == 3)
	    {
	      cv::putText(img,"(0,0.5,0)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[2].x - cx;
	      v = pp.p1[2].y -cy;
	      double uu, vu;
	      uu = u/(1. + k0);
	      vu = v/(1. + k0);
	      
	      if (cam == 1)
		{
		  double dist_to_point = DIST_TO_0050_CAM1;//getDistanceToPoint(pp.p1[2]);
		  x = (uu)*/*DIST_TO_0050_CAM1*/(dist_to_point)/fx;
		  y = (vu)*/*DIST_TO_0050_CAM1*/(dist_to_point)/fy;
		  rotation(0,1) = (x - translation(0)) / 500.;
		  rotation(1,1) = (y - translation(1)) / 500.;
		  rotation(2,1) = (/*DIST_TO_0050_CAM1*/dist_to_point - translation(2)) / 500.;
		}
	      if (cam ==2)
		{
		  double dist_to_point = DIST_TO_0050_CAM2;//getDistanceToPoint(pp.p1[2]);
		  x = (uu)*/*DIST_TO_0050_CAM2*/dist_to_point/fx;
		  y = (vu)*/*DIST_TO_0050_CAM2*/dist_to_point/fy;
		  rotation(0,1) = (x - translation(0)) / 500.;
		  rotation(1,1) = (y - translation(1)) / 500.;
		  rotation(2,1) = (/*DIST_TO_0050_CAM2*/dist_to_point - translation(2)) / 500.;
		}
	    } else if (pp.cnt == 4)
	    {
	      cv::putText(img,"(0,0,1)",pp.p1[pp.cnt-1],CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,cv::Scalar(255,0,0));
	      u = pp.p1[3].x - cx;
	      v = pp.p1[3].y - cy;
	      double uu, vu;
	      uu = u/(1. + k0);
	      vu = v/(1. + k0);
	      
	      if (cam == 1)
		{
		  double dist_to_point = DIST_TO_001_CAM1;//getDistanceToPoint(pp.p1[3]);
		  x = (uu)*/*DIST_TO_001_CAM1*/(dist_to_point)/fx;
		  y = (vu)*/*DIST_TO_001_CAM1*/(dist_to_point)/fy;
		  rotation(0,2) = (x - translation(0)) / 1000.;
		  rotation(1,2) = (y - translation(1)) / 1000.;
		  rotation(2,2) = (/*DIST_TO_001_CAM1*/(dist_to_point) - translation(2)) / 1000.;
		}
	      if (cam == 2)
		{
		  double dist_to_point = DIST_TO_001_CAM2;//getDistanceToPoint(pp.p1[3]);
		  x = (uu)*/*DIST_TO_001_CAM2*/dist_to_point/fx;
		  y = (vu)*/*DIST_TO_001_CAM2*/dist_to_point/fy;
		  rotation(0,2) = (x - translation(0)) / 1000.;
		  rotation(1,2) = (y - translation(1)) / 1000.;
		  rotation(2,2) = (/*DIST_TO_001_CAM2*/dist_to_point - translation(2)) / 1000.;
		}
	    }
	  cnt = pp.cnt;
	  pp.clicked = false;
	}
      cv::imshow("Extrinsic Image", img);
      cv::waitKey(10);
    }
  rotation_computed = true;

  std::cout << "rotation = " << rotation << std::endl;
  std::cout << "translation = " << translation << std::endl;
}

//\fn void ExtrinsicParam::changePanTilt(double pan, double tilt);
///\brief This function computes the new rotation matrix and the new translation vector of the extrinsic parameters when the camera has changed its position.
///\param pan Value of the new camera panoramique.
///\param tilt Value of the new camera tilt.
void ExtrinsicParam::changePanTilt(double pan, double tilt)
{
  Eigen::Matrix3d Rx, Ry;
  Rx.setZero();
  Ry.setZero();
  Rx(0,0) = 1;
  Rx(1,1) = cos((tilt - this->tilt)*PI/180.);
  Rx(1,2) = -sin((tilt - this->tilt)*PI/180.);
  Rx(2,1) = sin((tilt - this->tilt)*PI/180.);
  Rx(2,2) = cos((tilt - this->tilt)*PI/180.);
  Ry(0,0) = cos((pan - this->pan)*PI/180.);
  Ry(0,2) = sin((pan - this->pan)*PI/180.);
  Ry(1,1) = 1;
  Ry(2,0) = -sin((pan - this->pan)*PI/180.);
  Ry(2,2) = cos((pan - this->pan)*PI/180.);

  //Eigen::Matrix3d K = calib->getCameraMatrix();
  double dist_to_point = getDistanceToPoint(pp.p1[3]);
  std::cout << "t before =\n" << translation << std::endl;
  if (cam == 1)
    {
      translation(0) = translation(0)*K(0,0)/dist_to_point;//DIST_TO_000_CAM1;
      translation(0) = (translation(0) - 10*(pan - this->pan))*/*DIST_TO_000_CAM1*/dist_to_point/K(0,0);
      translation(1) = translation(1)*K(1,1)/dist_to_point;//DIST_TO_000_CAM1;
      translation(1) = (translation(1) + 12*(tilt - this->tilt))*/*DIST_TO_000_CAM1*/dist_to_point/K(1,1);
    }
  if (cam == 2)
    {
      translation(0) = translation(0)*K(0,0)/dist_to_point;//DIST_TO_000_CAM2;
      translation(0) = (translation(0) - 10.5*(pan - this->pan))*/*DIST_TO_000_CAM2*/dist_to_point/K(0,0);
      translation(1) = translation(1)*K(1,1)/dist_to_point;//DIST_TO_000_CAM2;
      translation(1) = (translation(1) + 10*(tilt - this->tilt))*/*DIST_TO_000_CAM2*/dist_to_point/K(1,1);
    }
  std::cout << "t after =\n" << translation << std::endl;

  rotation = Rx*Ry*rotation;

  this->pan = pan;
  this->tilt = tilt;
}

//\fn double ExtrinsicParam::getDistanceToPoint(cv::Point2f p);
///\brief This function is depricated.
double ExtrinsicParam::getDistanceToPoint(cv::Point2f p)
{
  ControlPTZ ctrlPTZ;
  double pan, tilt;
  double imgPz = 0.;
  int z;
  const double dist_c1_c2 = 8490.;

  if (cam == 1)
    {
      ctrlPTZ.refreshPosition(pan,tilt,z,1);
      pan += ((p.x)-704/2)/10;
      tilt += -((p.y)-576/2)/12;
      imgPz = sqrt((dist_c1_c2*cos((pan)*3.14/180.)*dist_c1_c2*cos((pan)*3.14/180.)) + (dist_c1_c2*sin((tilt)*3.14/180.)*dist_c1_c2*sin((tilt)*3.14/180.)));
    }
  else if (cam == 2)
    {
      ctrlPTZ.refreshPosition(pan,tilt,z,2);
      pan += ((p.x)-704/2)/11.5 + 35;
      tilt += -((p.y)-576/2)/10;
      imgPz = sqrt((dist_c1_c2*cos((pan)*3.14/180.)*dist_c1_c2*cos((pan)*3.14/180.)) + (dist_c1_c2*sin((tilt)*3.14/180.)*dist_c1_c2*sin((tilt)*3.14/180.)));
    }
  
  return imgPz;
}

//\fn void ExtrinsicParam::getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z);
///\brief This function computes the coordinates of a 3D point from the real landmark to the one of the camera.
///\param realP Value of the 3D point in the real landmark.
///\param x Value of the 3D point in the camera landmark and on the x axis.
///\param y Value of the 3D point in the camera landmark and on the y axis.
///\param z Value of the 3D point in the camera landmark and on the z axis.
void ExtrinsicParam::getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z)
{
  Eigen::Vector4d real;
  real << realP,
          1;
  
  Eigen::Vector3d imgP;
  Eigen::MatrixXd Rt;
  Rt.resize(3,4);
  Rt << rotation,translation;
  imgP = K*Rt*real;
  if (cam == 1)
    {
      x = imgP(0)/imgP(2) + (imgP(0)/imgP(2) - K(0,2))*(-0.08);
      y = imgP(1)/imgP(2) + (imgP(1)/imgP(2) - K(1,2))*(-0.08);
    } else if (cam == 2)
    {
      x = imgP(0)/imgP(2) + (imgP(0)/imgP(2) - K(0,2))*(0.08);
      y = imgP(1)/imgP(2) + (imgP(1)/imgP(2) - K(1,2))*(0.08);
    }
  z = imgP(2);
}
