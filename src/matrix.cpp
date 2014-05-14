/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#include "calibration/matrix.h"

using namespace Eigen;
using namespace std;

///\fn bool GES(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::VectorXd &lambda, Eigen::MatrixXd &H);
///\brief This function computes a generalized eigenvalue problem for two squared matrices A and B in order to get the real eigenvalues and the right eigenvectors.
///\param A Input squared matrix.
///\param B Input squared matrix with the same dimensions of A.
///\param lambda Output vector that will be resized to the number of real eigenvalues computed. On the exit, this vector will contain the real eigenvalues
///\param H Output matrix that will be resized to the dimension of A on its rows and the number of real eigenvalues on its culumns. On the exit, this matrix will contain the eigenvectors on its columns.
bool GES(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::VectorXd &lambda, Eigen::MatrixXd &H)
{
  int n = A.rows();
  lambda.resize(n);
  lambda.setZero();
  H.resize(n,n);
  H.setZero();

  MatrixXd A0 = A;

  int lwork = 1000+(8*n);
  double *work = new double[lwork];

  char jobvl[2] = "N";
  char jobvr[2] = "V";
  int info = 0;

  VectorXd alphar, alphai, beta;
  alphar.resize(n);
  alphai.resize(n);
  beta.resize(n);
  MatrixXd vl, vr;
  vl.resize(n,n);
  vr.resize(n,n);

  dgegv_(jobvl, jobvr,
	 &n, A0.data(), &n,
	 B.data(), &n,
	 alphar.data(),
	 alphai.data(),
	 beta.data(),
	 vl.data(), &n,
	 vr.data(), &n,
	 work, &lwork,
	 &info);

  delete [] work;
  if (info != 0) {
    std::cout << ": info = " << info << " n = " << n << std::endl;
    return false;
  }

  int cnt = 0;
  for (int i=0; i<alphar.size(); i++)
    if (alphai(i)/beta(i) == 0.)
      cnt++;

  lambda.resize(cnt);
  H.resize(n,cnt);
  cnt = 0;
  int imag = 0;
  for (int i=0; i<alphar.size(); i++)
    {
      if (alphai(i)/beta(i) == 0.)
	{
	  lambda(cnt) = alphar(i)/beta(i);
	  for (int j=0; j<n; j++)
	    H(j,cnt) = vr(j,i+imag);
	  cnt++;
	} else
	{
	  imag++;
	}
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
///\return This function will return 0 if it is well executed. Otherwise, it will return:\n-ERROR_PSM if the matrices A0, A1 and/or A2 are not squared.\n-ERROR_PSDM if the matrices A0, A1 and A2 are not of the same dimensions.\n-ERROR_PGS if the function failed to cumpute the generalized eigenvalue problem.\n-ERROR_PNRE if there is only complexe eigenvalues and eigenvectors computed.
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
  if (!GES(A,B,lambda,H))
    {
      cout << "Failed to compute generalized eigen solver in polyeig" << endl;
      return ERROR_PGS;
    }
  MatrixXd tempH = H;
  H.resize(n,H.cols());

  for (int i=0; i<n; i++)
    for (int j=0; j<tempH.cols(); j++)
      H(i,j) = tempH(i,j);

  VectorXd templ = lambda;
  MatrixXd temp;
  bool results;
  vector<int> v;

  for (int i=0; i<templ.size(); i++)
    {
      bool nonzero =true;
      for (int j=0; j<H.rows(); j++)
	if (H(j,i)==0.)
	  nonzero = false;

      if (nonzero)
	{
	  temp.noalias() = (A0 + templ(i)*A1 + templ(i)*templ(i)*A2)*(H.col(i));

	  double eps = 1e-6;
	  for (int j = 0; j < temp.rows(); j++)
	    for (int k = 0; k < temp.cols(); k++)
	      if (temp(j,k)<0)
		temp(j,k) = 0.-temp(j,k);
	  for (int k = 0; k < temp.cols(); k++)
	    {
	      results = true;
	      for (int j = 0; j < temp.rows(); j++)
		if (temp(j,k) > eps)
		  results = false;
	      if (results)
		v.push_back(i);
	    }
	}
    }

  if (v.size()==0)
    {
      cout << "Only complex eigenvalues computed" << endl;
      return ERROR_PNRE;
    }

  lambda.resize(v.size());
  lambda.setZero();
  H.resize(n,v.size());
  H.setZero();
  for (int i=0; i < lambda.size(); i++)
    {
      lambda(i) = templ(v.at(i));
      for (int j=0; j<H.rows(); j++)
	H(j,i) = tempH(j,v.at(i));
    }
  
  return 0;
}

///\fn int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd &H, Eigen::VectorXd &lambda);
///\brief This function computes the real eigenvalues lambda and eigenvectors H in the following polynomial equation : (A0 + lambda*A1)*H = 0
///\param A0 Input squared matrix.
///\param A1 Input squared matrix with the same dimensions of A0.
///\param H Output matrix that will be resized to the number of real eigenvectors found. On the output of this function, it will contain the real eigenvectors on its columns.
///\param lambda Output vector that will be resized to the number of real eigenvalues found. On the output of this function, it will contain the real eigenvalues.
///\return This function will return 0 if it is well executed. Otherwise, it will return:\n-ERROR_PSM if the matrices A0, A1 and/or A2 are not squared.\n-ERROR_PSDM if the matrices A0, A1 and A2 are not of the same dimensions.\n-ERROR_PGS if the function failed to cumpute the generalized eigenvalue problem.\n-ERROR_PNRE if there is only complexe eigenvalues and eigenvectors computed.
int polyeig(Eigen::MatrixXd A0, Eigen::MatrixXd A1, Eigen::MatrixXd &H, Eigen::VectorXd &lambda)
{
  int n = A0.rows();

  if (A0.rows()!=A0.cols() && A1.rows()!=A1.cols())
    {
      cout << "A0 and A1 should be squared matrices" << endl;
      return ERROR_PSM;
    }

  if (A1.rows()!=n)
    {
      cout << "A0 and A1 should be the same size" << endl;
      return ERROR_PSDM;
    }

  // Use the JACOBI-DAVIDSON conversion
  MatrixXd A, B;
  A.resize(n,n);
  A.setZero();
  B.resize(n,n);
  B.setZero();

  A = -A0;
  B = A1;

   // Resolve Ax = lBx with x a vector and l a real scalar
  if (!GES(A,B,lambda,H))
    {
      cout << "Failed to compute generalized eigen solver in polyeig" << endl;
      return ERROR_PGS;
    }
  MatrixXd tempH = H;
  H.resize(n,H.cols());

  for (int i=0; i<n; i++)
    for (int j=0; j<tempH.cols(); j++)
      H(i,j) = tempH(i,j);

  VectorXd templ = lambda;
  MatrixXd temp;
  bool results;
  vector<int> v;

  for (int i=0; i<templ.size(); i++)
    {
      bool nonzero =true;
      for (int j=0; j<H.rows(); j++)
	if (H(j,i)==0.)
	  nonzero = false;
      if (nonzero)
	{
	  temp.noalias() = (A0 + templ(i)*A1)*(H.col(i));

	  double eps = 1e-6;
	  for (int j = 0; j < temp.rows(); j++)
	    for (int k = 0; k < temp.cols(); k++)
	      if (temp(j,k)<0)
		temp(j,k) = 0.-temp(j,k);
	  for (int k = 0; k < temp.cols(); k++)
	    {
	      results = true;
	      for (int j = 0; j < temp.rows(); j++)
		if (temp(j,k) > eps)
		  results = false;
	      if (results)
		v.push_back(i);
	    }
	}
    }

  if (v.size()==0)
    {
      cout << "Only complex eigenvalues computed" << endl;
      return ERROR_PNRE;
    }

  lambda.resize(v.size());
  lambda.setZero();
  H.resize(n,v.size());
  H.setZero();
  for (int i=0; i < lambda.size(); i++)
    {
      lambda(i) = templ(v.at(i));
      for (int j=0; j<H.rows(); j++)
	H(j,i) = tempH(j,v.at(i));
	}
  
  return 0;
}

///\func void pseudoInverse(Eigen::MatrixXd A, Eigen::MatrixXd &inv, double threshold = 1e-6);
///\brief This function return the pseudo inverse of the matrix A.
///\param A Input matrix from which this function computes the pseudo inverse.
///\param inv Output matrix in which is stored the pseudo inverse of A.
///\param threshold Input value that contains the value of the minimal eigenvalue accepted.
void pseudoInverse(Eigen::MatrixXd A, Eigen::MatrixXd& inv, double threshold)
{
  bool toTranspose = false;

  MatrixXd mat = A;

  if (mat.rows() < mat.cols())
    {
      mat.transposeInPlace();
      toTranspose = true;
    }

  const unsigned int NR = mat.rows();
  const unsigned int NC = mat.rows();

  MatrixXd U, VT;
  U.resize(NR,NR);
  U.setZero();
  VT.resize(NC,NC);
  VT.setZero();
  VectorXd s;
  s.resize(min(NR,NC));
  s.setZero();
  char Jobu='A';
  char Jobvt='A';
  const int m = NR;  
  const int n = NC;
  int linfo;
  int lda = std::max(m,n);
  int lw=-1;
  {
    double vw;
    dgesvd_(&Jobu, &Jobvt, &m, &n,
	    mat.data(), &lda,
	    0, 0, &m, 0, &n, &vw, &lw, &linfo);
    lw = int(vw)+5;
  }
  VectorXd w;
  w.resize(lw);
  w.setZero();
  int lu = U.rows();
  int lvt = VT.rows();
  dgesvd_(&Jobu, &Jobvt, &m, &n,
	  mat.data(), &lda,
	  s.data(),
	  U.data(),
	  &lu,
	  VT.data(), &lvt,
	  w.data(), &lw, &linfo);

  MatrixXd S;
  S.resize(mat.cols(), mat.rows());
  S.setZero();
  for (int i=0;i<S.rows();i++)
    {
    for (int j=0; j<S.cols();j++)
      {
	if ((i==j) && (fabs(s(i))>threshold))
	  S(i,i) = 1./s(i);
	else
	  S(i,j) = 0;
      }
    }
  MatrixXd tmp1;
  tmp1 = S*(U.transpose());
  inv = (VT.transpose())*tmp1;
  if (toTranspose)
    inv.transposeInPlace();
}
