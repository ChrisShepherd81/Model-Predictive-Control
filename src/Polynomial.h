/*
 * Polynomial.h
 *
 *  Created on: 09.06.2017
 *      Author: christian@inf-schaefer.de
 */

#ifndef SRC_POLYNOMIAL_H_
#define SRC_POLYNOMIAL_H_

#include <cassert>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <iostream>

class Polynomial
{
  Eigen::VectorXd _coeffs;
  const double epsilon = 1e-6;

  // Fit a polynomial.
  void polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, size_t order);

 public:
  Polynomial(Eigen::VectorXd xvals, Eigen::VectorXd yvals, size_t order);

  // Evaluate a polynomial.
  double polyeval(double x);

  // Calculate the slope at x.
  double slopeAt(double x);

  Eigen::VectorXd getCoefficients() const;

};

#endif /* SRC_POLYNOMIAL_H_ */
