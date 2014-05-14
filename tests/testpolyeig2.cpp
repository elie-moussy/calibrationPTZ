/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#define BOOST_TEST_MODULE polyeig2
#include <boost/test/unit_test.hpp>
#include "calibration/matrix.h"

BOOST_AUTO_TEST_CASE (main_test)
{
  Eigen::MatrixXd A0, A1, H;
  A0.resize(5,5);
  A0.setRandom();
  A1.resize(5,5);
  A1.setRandom();
  
  Eigen::VectorXd lambda;

  polyeig(A0, A1, H, lambda);

  Eigen::MatrixXd temp, zero;
  zero.resize(5,1);
  zero.setZero();
  bool results = true;
  for (int i=0; i<lambda.size(); i++)
    {
      temp.noalias() = (A0 + lambda(i)*A1)*(H.col(i));

      double eps = 1e-6;
      for (int i = 0; i < temp.rows(); i++)
	for (int j = 0; j < temp.cols(); j++)
	  if (temp(i,j)<0)
	    temp(i,j) = 0.-temp(i,j);
      for (int i = 0; i < temp.rows(); i++)
	for (int j = 0; j < temp.cols(); j++)
	  if (temp(i,j) > eps)
	    results = false;
    }

  BOOST_CHECK_EQUAL(results,true);
}
