/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

///\file matrix.h
///\author Elie MOUSSY
///\brief This file contains the signatures of some usefull functions in order to manipulate matrices with Eigen.

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

void polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd A2, Eigen::VectorXcd *H, double *lambda);
void polyeig(Eigen::Matrix3d A0, Eigen::Matrix3d A1, Eigen::Vector3cd *H, double *lambda);

#endif // MATRIX_H_INCLUDED
