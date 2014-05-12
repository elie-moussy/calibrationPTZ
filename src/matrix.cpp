/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#include "matrix.h"

using namespace Eigen;
using namespace std;

///\fn logical sb02ox (double *_alphar, double * _alphai, double *_beta)
///\brief This function is used in the generalizedSchur function in order to order the eigenvalues and the eigenvectors.
logical sb02ox (double *_alphar, double * _alphai, double *_beta)
{
  logical r;
  double w =dlapy2_(_alphar,_alphai);
  double w2 = fabs(*_beta);
  r = w < w2;
  return r;
}

///\fn bool GeneralizedSchur(Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::VectorXd &alphar, Eigen::VectorXd &alphai, Eigen::VectorXd &beta, Eigen::MatrixXd &L, Eigen::MatrixXd &R);
///\brief This function computes a generalized Schur for two squared matrices A and B in order to get the eigenvalues and the left and right eigenvectors.
///\param A Input squared matrix.
///\param B Input squared matrix with the same dimensions of A.
///\param alphar Output vector that will be resized to the dimension of A. For each element i of alphar, alphar(i)/beta(i) is the real part of the eigenvalue.
///\param alphai Output vector that will be resized to the dimension of A. For each element i of alphai, alphai(i)/beta(i) is the imaginary part of the eigenvalue.
///\param beta Output vector that will be resized to the dimension of A. cf. parameters alphar and alphai for more details.
///\param L Output matrix that will be resized to the dimension of A. On the output of this function, this matrix will contain the left Schur vectors on its columns.
///\param R Output matrix that will be resized to the dimension of A. On the output of this function, this matrix will contain the right Schur vectors on its columns.
bool GeneralizedSchur(Eigen::MatrixXd &A, Eigen::MatrixXd &B, Eigen::VectorXd &alphar, Eigen::VectorXd &alphai, Eigen::VectorXd &beta, Eigen::MatrixXd &L, Eigen::MatrixXd &R)
{
  int n = A.rows();

  alphar.resize(n);
  alphar.setZero();
  alphai.resize(n);
  alphai.setZero();
  beta.resize(n);
  beta.setZero();
  L.resize(n,n);
  L.setZero();
  R.resize(n,n);
  R.setZero();

  int sdim = 0;
  int lwork = 1000+ (8*n + 16);
  double *work = new double[lwork]; //std::vector<double> work(lwork);
  for(int i=0;i<lwork;i++)
    work[i] = 0.0;
  int info = 0;
  logical *bwork=new logical[2*n];
  for(int i=0;i<2*n;i++)
    bwork[i] = 0;

  char lV[2]="V";
  char lS[2]="S";

  dgges_ (lV, lV,
          lS,
	  (logical (*)(...))sb02ox,
          &n,
          A.data(), &n,
          B.data(), &n,
          &sdim,
          alphar.data(),
          alphai.data(),
          beta.data(),
          L.data(), &n,
          R.data(), &n,
          &work[0], &lwork,
          bwork,
          &info);

  delete [] work;
  delete [] bwork;
  if (info != 0) {
    std::cout << ": info = " << info << " n = " << n << std::endl;
    return false;
  }
  
  return true;
}

///\fn int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd A2, Eigen::MatrixXd &H, Eigen::VectorXd &lambda);
///\brief This function computes the real eigenvalues lambda and eigenvectors H in the following polynomial equation : (A0 + lambda*A1 + lambda²*A2)*H = 0
///\param A0 Input squared matrix.
///\param A1 Input squared matrix with the same dimensions of A0.
///\param A2 Input squared matrix with the same dimensions of A0.
///\param H Output matrix that will be resized to the number of real eigenvectors found. On the output of this function, it will contain the real eigenvectors on its columns.
///\param lambda Output vector that will be resized to the number of real eigenvalues found. On the output of this function, it will contain the real eigenvalues.
///\return This function will return 0 if it is well executed. Otherwise, it will return:\n-ERROR_PSM if the matrices A0, A1 and/or A2 are not squared.\n-ERROR_PSDM if the matrices A0, A1 and A2 are not of the same dimensions.\n-ERROR_PGS if the function failed to cumpute the generalized Schur.\n-ERROR_PNRE if there is only complexe eigenvalues and eigenvectors computed.
int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd A2, Eigen::MatrixXd &H, Eigen::VectorXd &lambda)
{
  int n = A0.rows();
  
  if (A0.rows()!=A0.cols() && A1.rows()!=A1.cols() && A2.rows()!=A2.cols())
    {
      cout << "A0, A1 and A2 should be squared matrices" << endl;
      return ERROR_PSM;
    }

  if (A1.rows()!=n && A2.rows()!=n)
    {
      cout << "A0, A1 and A2 should be the same size" << endl;
      return ERROR_PSDM;
    }

  // Use the JACOBI-DAVIDSON conversion
  MatrixXd A, B;
  A.resize(2*n,2*n);
  A.setZero();
  B.resize(2*n,2*n);
  B.setZero();

  for (int i=0; i<n; i++)
    for (int j=0; j<n; j++)
      A(i,j) = 0.-A0(i,j);

  for (int i=n; i<2*n; i++)
    A(i,i) = 1.;

  for (int i=0; i<n; i++)
    for (int j=0; j<n; j++)
      B(i,j) = A1(i,j);  

  for (int i=0; i<n; i++)
    for (int j=n; j<2*n; j++)
      B(i,j) = A2(i,j-n);

  for (int i=n; i<2*n; i++)
    B(i,i-n) = 1.;

  // Resolve Ax = lBx with x a vector and l a real scalar
  VectorXd alphar, alphai, beta;
  Eigen::MatrixXd L, R;
  
  if(!GeneralizedSchur(A,B,alphar,alphai,beta,L,R))
    {
      cout << "Failed to compute generalizedShur in polyeig" << endl;
      return ERROR_PGS;
    }

  // Chose only the real eigenvalues
  int cnt = 0;
  for (int i=0; i<alphai.size(); i++)
    if (alphai(i)/beta(i) == 0)
      cnt++;
  
  if (cnt == 0)
    {
      cout << "Non real eigenvalues in polyeig" << endl;
      return ERROR_PNRE;
    }

  lambda.resize(cnt);
  H.resize(n, cnt);
  cnt = 0;
  for (int i=0; i<alphai.size(); i++)
    {
      if (alphai(i)/beta(i) == 0)
	{
	  lambda(cnt) = alphar(i)/beta(i);
	  for (int j = 0; j < n; j++)
	    H(j,cnt) = R(j,i);
	}
    }
  
  return 0;
}
