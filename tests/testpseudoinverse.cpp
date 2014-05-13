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
  M.resize(10,10);
  M.setRandom();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(10,10);

  Eigen::MatrixXd inv;
  ::pseudoInverse(M,inv);

  Eigen::MatrixXd temp;
  temp = M*inv;
  temp.noalias() -= I;

  bool results = true;
  if (!temp.isApprox(Eigen::MatrixXd::Zero(10,10)))
    results = false;

  std::cout << results;
}
