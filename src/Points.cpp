/*
 * Path.cpp
 *
 *  Created on: 08.06.2017
 */

#include "Points.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Points::Points(std::vector<double> x, std::vector<double> y)
{
  assert(x.size() == y.size());

  _points = Eigen::MatrixXd::Zero(x.size(), 2);

  for(size_t i=0; i < x.size(); ++i)
  {
    _points(i,0) = x[i];
    _points(i,1) = y[i];
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Points::~Points() {}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Points::getXStdVector() const
{
  return std::vector<double>(_points.col(IDX_X).data(), _points.col(IDX_X).data() + _points.col(IDX_X).size() );
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Points::getYStdVector() const
{
  return std::vector<double>(_points.col(IDX_Y).data(), _points.col(IDX_Y).data() + _points.col(IDX_Y).size() );
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Points::getXVector() const
{
  return this->_points.col(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Points::getYVector() const
{
  return this->_points.col(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Points::translation(double x, double y)
{
  _points.rowwise() += Eigen::Vector2d(x,y).transpose();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Points::rotation(double angle)
{
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2,2);
  R <<  std::cos(angle), -std::sin(angle),
        std::sin(angle), std::cos(angle);

  for(size_t i = 0; i < _points.rows(); ++i)
  {
    _points.row(i) =  R * _points.row(i).transpose() ;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> Points::operator[](size_t index)
{
  return std::vector<double>{_points.row(index).data(), _points.row(index).data() + _points.row(index).size()};
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
