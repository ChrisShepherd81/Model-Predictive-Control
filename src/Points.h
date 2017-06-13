/*
 * Path.h
 *
 *  Created on: 08.06.2017
 */

#ifndef SRC_POINTS_H_
#define SRC_POINTS_H_

#include <vector>
#include <iostream>
#include <tuple>

#include "Eigen-3.3/Eigen/Dense"


class Points
{
  const size_t IDX_X = 0;
  const size_t IDX_Y = 1;

  Eigen::MatrixXd _points;

 public:
  Points(std::vector<double> x, std::vector<double> y);
  Points(double x, double y) : Points(std::vector<double>{x}, std::vector<double>{y}) {}

  void translation(double x, double y);
  void rotation(double angle);

  std::vector<double> getXStdVector() const;
  std::vector<double> getYStdVector() const;

  Eigen::VectorXd getXVector() const;
  Eigen::VectorXd getYVector() const;

  std::vector<double> operator[](size_t index);

  virtual ~Points();
};

#endif /* SRC_POINTS_H_ */
