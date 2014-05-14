/***************************************
  * Copyright (C) LAAS-CNRS
  * Author : Elie MOUSSY
***************************************/

#define BOOST_TEST_MODULE pseudo-inverse
#include <boost/test/unit_test.hpp>
#include "calibration/matrix.h"

BOOST_AUTO_TEST_CASE (main_test)
{
  Eigen::MatrixXd M;
  M.resize(5,2);
  M.setRandom();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5,5);

  Eigen::MatrixXd inv;
  pseudoInverse(M,inv);

  std::cout << "inv=\n" << inv << std::endl << std::endl;
  Eigen::MatrixXd temp;
  temp = M*inv;
  temp.noalias() -= I;

  std::cout << temp << std::endl;
  bool results = true;
  double eps = 1e-6;
  for (int i = 0; i < temp.rows(); i++)
    for (int j = 0; j < temp.cols(); j++)
      if (temp(i,j)<0)
	temp(i,j) = 0.-temp(i,j);
  for (int i = 0; i < temp.rows(); i++)
    for (int j = 0; j < temp.cols(); j++)
      if (temp(i,j) > eps)
	results = false;

  BOOST_CHECK_EQUAL(results,true);
}
