/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#ifndef MATRIX_H_INCLUDED
#define MATRIX_H_INCLUDED

///\file matrix.h
///\author Elie MOUSSY
///\brief This file contains the signatures of some usefull functions in order to manipulate matrices with Eigen.

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <iostream>
#include <vector>

#define ERROR_PSM 1 // polyeig squared matrices
#define ERROR_PSDM 2 // polyeig same dimension matrices
#define ERROR_PGS 3 // polyeig failed to compute generalized eigenvalue problem
#define ERROR_PNRE 4 // polyeig non real eigenvalues computed

typedef double doublereal;
typedef int integer ;

extern "C" {

  void dgesvd_(char const* jobu, char const* jobvt,
	       int const* m, int const* n, double* a, int const* lda,
	       double* s, double* u, int const* ldu,
	       double* vt, int const* ldvt,
	       double* work, int const* lwork, int* info);
  void dgegv_(char *, char *, integer *, doublereal *, integer *,
	      doublereal *, integer *, doublereal *, doublereal *,
	      doublereal *, doublereal *, integer *, doublereal *,
	      integer *, doublereal *, integer *, integer *);
}

double fRand(double fMin, double fMax);
bool GES(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::VectorXd &lambda, Eigen::MatrixXd &H);
int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd A2, Eigen::MatrixXd &H, Eigen::VectorXd &lambda);
int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd &H, Eigen::VectorXd &lambda);
void pseudoInverse(Eigen::MatrixXd A, Eigen::MatrixXd& inv, double threshold = 1e-4);

#endif // MATRIX_H_INCLUDED
