/*
 * Path.cpp
 *
 *  Created on: 08.06.2017
 *      Author: christian@inf-schaefer.de
 */

#include "Path.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Path::Path(std::vector<double> &x, std::vector<double> &y)
{
  assert(x.size() == y.size());

  _pathPoints = Eigen::MatrixXd::Zero(x.size(), 2);

  for(size_t i=0; i < x.size(); ++i)
  {
    _pathPoints(i,0) = x[i];
    _pathPoints(i,1) = y[i];
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Path::~Path() {
  // TODO Auto-generated destructor stub
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Path::getXVector() const
{
  return std::vector<double>(_pathPoints.col(IDX_X).data(), _pathPoints.col(IDX_X).data() + _pathPoints.col(IDX_X).size() );
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Path::getYVector() const
{
  return std::vector<double>(_pathPoints.col(IDX_Y).data(), _pathPoints.col(IDX_Y).data() + _pathPoints.col(IDX_Y).size() );
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Path::translation(double x, double y)
{
  _pathPoints.rowwise() += Eigen::Vector2d(x,y).transpose();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Path::rotation(double angle)
{
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2,2);
  R <<  std::cos(angle), -std::sin(angle),
        std::sin(angle), std::cos(angle);

  for(size_t i = 0; i < _pathPoints.rows(); ++i)
  {
    _pathPoints.row(i) =  R * _pathPoints.row(i).transpose() ;
  }
}
