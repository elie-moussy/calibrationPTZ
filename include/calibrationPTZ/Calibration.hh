/***************************************
 * Copyright (C) LAAS-CNRS
 * Author : Elie MOUSSY
 ***************************************/

#ifndef CALIBRATION_H
#define CALIBRATION_H

///\file Calibration.h
///\author Elie MOUSSY
///\brief This file contains the class of calibration

#include "controlPTZ/ControlPTZ.hh"
#include "controlPTZ/StreamRTSP.hh"
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "levmar/levmar.h"
#include "matrix.hh"
#include <limits>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <sstream>
#include <pthread.h>

#define SETS_SIZE 10

///\struct SStream Calibration.h
///\brief This structure allows to grab a frame from a source (in this case the camera)
typedef struct
{
  StreamRTSP *stream;
  cv::Mat imgBuf;
  pthread_mutex_t mutex_stock;
}SStream;

///\fn static void* pthread_img(void* stream);
///\brief This function is a thread that grabs an image from a camera and storesit in an SStream structure.
///\param stream A pointer to the SStream in which we want to store the image from the camera.
static void* pthread_img(void* stream)
{
  SStream *str = (SStream *) stream;
  for(;;)
    {
      pthread_mutex_lock(&(str->mutex_stock));
      str->imgBuf=str->stream->grabFrame();
      pthread_mutex_unlock(&(str->mutex_stock));
    }
  return NULL;
}

///\class Calibration Calibration.h
class Calibration
{
public :
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Calibration(int c, SStream *stream);
  void computePrincipalPoint(double &cx, double &cy);
  void computeLensDistortion(double *k);
  void computeFocalLength(Eigen::Matrix3d* K0, double* fx, double* fy);
  void computeZoomScaleDependence(double &af, double &bf, double &ak, double &bk, double &alpha, int stepZoom);
  void nonLinearOptimization();
  void computePTSet(cv::Mat img, int i);
  void computeZoomSet(cv::Mat img, int i);
  void computeCalibration();
  ~Calibration();

private :
  void MatchFeaturesZoom(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, int i, int j);
  void MatchFeaturesPT(std::vector<KeyPoint> &kp1, std::vector<KeyPoint> &kp2, int i, int j);
  Mat* zoomSet;
  Mat* ptSet;
  Eigen::Vector2d c;
  Eigen::Matrix3d *K;
  double *fx;
  double *fy;
  double *kz;
  double af, bf, ak, bk, alpha, pan_error, tilt_error;
  int cam;
  SStream *str;
};

#endif
