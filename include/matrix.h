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

#define ERROR_PSM 1 // polyeig squared matrices
#define ERROR_PSDM 2 // polyeig same dimension matrices
#define ERROR_PGS 3 // polyeig failed to compute generalized schur
#define ERROR_PNRE 4 // polyeig non real eigenvalues computed

typedef long int logical;
typedef double doublereal;
typedef logical(* L_fp)(...);
typedef int integer ;

extern "C" {
  extern doublereal dlapy2_(doublereal *, doublereal *); 
  extern double dlamch_ (char *);
  extern /* Subroutine */ int dgges_(char *, char *, char *, L_fp, integer *
				     , doublereal *, integer *, doublereal *, integer *, integer *, 
				     doublereal *, doublereal *, doublereal *, doublereal *, integer *,
				     doublereal *, integer *, doublereal *, integer *, logical *, 
				     integer *);
  extern /* Subroutine */ int dtgevc_(const char* side, const char* howmny,
				      const int* select, const int* n,
				      const double* a, const int* lda,
				      const double* b, const int* ldb,
				      double* vl, const int* ldvl,
				      double* vr, const int* ldvr,
				      const int* mm, int* m, double* work, int* info);
}

bool GeneralizedSchur(Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::VectorXd &alphar, Eigen::VectorXd &alphai, Eigen::VectorXd &beta, Eigen::MatrixXd &L, Eigen::MatrixXd &R);
bool QZFactorization(Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::VectorXd &alphar, Eigen::VectorXd &alphai, Eigen::VectorXd &beta, Eigen::MatrixXd &L, Eigen::MatrixXd &R, Eigen::MatrixXcd &H);
int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd A2, Eigen::MatrixXd &H, Eigen::VectorXd &lambda);

#endif // MATRIX_H_INCLUDED
