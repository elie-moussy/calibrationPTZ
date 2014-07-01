/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#ifndef EXTRINSICPARAM_H_INCLUDED
#define EXTRINSICPARAM_H_INCLUDED

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "Calibration.hh"
#include "controlPTZ/ControlPTZ.hh"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <sstream>

#define FRAME_WIDTH 704
#define FRAME_HEIGHT 576
#define DIST_TO_000_CAM1 -7282.35
#define DIST_TO_000_CAM2 5447.48
#define DIST_TO_0500_CAM1 -7106.97
#define DIST_TO_0500_CAM2 6180.59
#define DIST_TO_0050_CAM1 -7704.14
#define DIST_TO_0050_CAM2 5179.29
#define DIST_TO_001_CAM1 -6731.05
#define DIST_TO_001_CAM2 5000.91

static struct mousedata
{
  int cnt;
  bool clicked;
  cv::Point2f p1[4];
} pp = {0,false,{}};

static void On_Mouse(int event, int x, int y, int flags, void* param)
{
  switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
      {
	if (pp.cnt <= 4)
	  {
	    if (pp.cnt < 4)
	      {
		cv::Point2f ptr(x,y);
		pp.p1[pp.cnt] = ptr;	
	      }
	    pp.cnt = pp.cnt+1;
	    pp.clicked =true;
	  }
      }
      break;

    case CV_EVENT_RBUTTONDOWN:
      {
	if (pp.cnt > 0)
	  {
	    pp.cnt = pp.cnt-1;
	    pp.clicked = false;
	  }
      }
      break;
    }
}

class ExtrinsicParam
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ExtrinsicParam(int cam=1);
  ~ExtrinsicParam();
  Eigen::Vector3d getTranslationVector();
  Eigen::Matrix3d getRotationMatrix();
  Eigen::Matrix3d getCameraMatrix();
  void computeRtMatrix(double pan, double tilt, cv::Mat image);
  void changePanTilt(double pan, double tilt);
  void getCameraPointFrom3d(Eigen::Vector3d realP, double &x, double &y, double &z);
  Eigen::Vector3d undistortPoint(Eigen::Vector3d distortedPoint);
  Eigen::Vector3d distortPoint(Eigen::Vector3d undistortedPoint);

private:
  int cam;
  double pan;
  double tilt;
  bool rotation_computed;
  Calibration *calib;
  Eigen::Vector3d translation;
  Eigen::Matrix3d K;
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d H;
  Eigen::VectorXd dist;
};

#endif //EXTRINSICPARAM_H_INCLUDED
