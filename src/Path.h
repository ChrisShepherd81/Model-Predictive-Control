/*
 * Path.h
 *
 *  Created on: 08.06.2017
 *      Author: christian@inf-schaefer.de
 */

#ifndef SRC_PATH_H_
#define SRC_PATH_H_

#include <vector>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"


class Path
{
  const size_t IDX_X = 0;
  const size_t IDX_Y = 1;

  Eigen::MatrixXd _pathPoints;

 public:
  Path(std::vector<double> &x, std::vector<double> &y);

  void translation(double x, double y);
  void rotation(double angle);

  std::vector<double> getXVector() const;
  std::vector<double> getYVector() const;
  virtual ~Path();
};

#endif /* SRC_PATH_H_ */
