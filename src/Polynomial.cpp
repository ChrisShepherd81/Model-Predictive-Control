/*
 * Polynomial.cpp
 *
 *  Created on: 09.06.2017
 *      Author: christian@inf-schaefer.de
 */

#include "Polynomial.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Polynomial::Polynomial(Eigen::VectorXd xvals, Eigen::VectorXd yvals, size_t order)
{
  this->polyfit(xvals, yvals, order);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Polynomial::polyeval(double x) {
  double result = 0.0;
  for (int i = 0; i < this->_coeffs.size(); i++) {
    result += this->_coeffs[i] * pow(x, i);
  }
  return result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Polynomial::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, size_t order)
{
  // Adapted from
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  this->_coeffs = result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Polynomial::getCoefficients() const
{
  return this->_coeffs;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double Polynomial::slopeAt(double x)
{
  double y_x = this->polyeval(x);
  double y_x_eps = this->polyeval(x+this->epsilon);
  return (y_x_eps-y_x)/this->epsilon;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
