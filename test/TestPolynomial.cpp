/*
 * TestPolynomial.cpp
 *
 *  Created on: 09.06.2017
 *      Author: christian@inf-schaefer.de
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../src/Polynomial.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPolynomial, TestCoeffivientsGrad1)
{
  Eigen::VectorXd x_vals(4);
  x_vals << 0, 1, 2, 3;

  Eigen::VectorXd y_vals(4);
  y_vals << 0, 1, 2, 3;

  Polynomial sut(x_vals, y_vals, 1);

  Eigen::VectorXd coeff = sut.getCoefficients();
  ASSERT_EQ(coeff.size(), 2);
  ASSERT_NEAR(coeff[0], 0, 1e-6);
  ASSERT_NEAR(coeff[1], 1, 1e-6);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPolynomial, TestCoeffivientsGrad2)
{
  Eigen::VectorXd x_vals(4);
  x_vals << -3, -1, 0, 2;

  Eigen::VectorXd y_vals(4);
  y_vals << 9, 1, 0, 4;

  Polynomial sut(x_vals, y_vals, 2);

  Eigen::VectorXd coeff = sut.getCoefficients();
  ASSERT_EQ(coeff.size(), 3);
  ASSERT_NEAR(coeff[0], 0, 1e-6);
  ASSERT_NEAR(coeff[1], 0, 1e-6);
  ASSERT_NEAR(coeff[2], 1, 1e-6);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPolynomial, TestPolyEval)
{
  Eigen::VectorXd x_vals(4);
  x_vals << -3, -1, 0, 2;

  Eigen::VectorXd y_vals(4);
  y_vals << 9, 1, 0, 4;

  Polynomial sut(x_vals, y_vals, 2);
  ASSERT_NEAR(sut.polyeval(0), 0, 1e-6);
  ASSERT_NEAR(sut.polyeval(0.5), 0.25, 1e-6);
  ASSERT_NEAR(sut.polyeval(-2), 4, 1e-6);
  ASSERT_NEAR(sut.polyeval(-3), 9, 1e-6);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


