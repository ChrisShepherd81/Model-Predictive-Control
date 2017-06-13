/*
 * FileWriter.cpp
 *
 *  Created on: 24.05.2017
 *      Author: christian@inf-schaefer.de
 */

#include "FileWriter.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FileWriter::FileWriter(std::string filename, size_t polynomialGrade, size_t noOfWaypoints, size_t noOfPredictions) {
  this->filename_ = filename;
  this->writeHeader(polynomialGrade, noOfWaypoints, noOfPredictions);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FileWriter::writeHeader(size_t polynomialGrade, size_t noOfWaypoints, size_t noOfPredictions)
{
  std::stringstream ss;
  ss << "x,y,psi,v,cte,epsi,";
  auto fillHeader = [&](std::string prefix, size_t n) -> void
  {
    for(size_t i=0; i < n; ++i)
      ss << prefix << "_" << i << ",";
  };
  fillHeader("coeff_", polynomialGrade+1);
  fillHeader("w_x", noOfWaypoints);
  fillHeader("w_y", noOfWaypoints);
  fillHeader("p_x", noOfPredictions-1);
  fillHeader("p_y", noOfPredictions-1);

  this->writeLine(ss.str());
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FileWriter::writeData(const std::vector<double> &state, const std::vector<double> &coeffs,
                           const std::vector<double> &wayPointsX, const std::vector<double> &wayPointsY,
                           const std::vector<double> &predictionsX,const std::vector<double> &predictionsY)
{
  std::stringstream ss;

  auto fillStringStream = [&ss] (const std::vector<double> &data) -> void
  {
    for(size_t i=0; i < data.size(); ++i )
      ss << data[i] << ",";
  };

  fillStringStream(state);
  fillStringStream(coeffs);
  fillStringStream(wayPointsX);
  fillStringStream(wayPointsY);
  fillStringStream(predictionsX);
  fillStringStream(predictionsY);

  this->writeLine(ss.str());
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FileWriter::writeLine(std::string line) {
  //Remove last ',' if exist.
  if(line[line.size()-1] == ',')
    line = line.substr(0, line.size()-1);
  this->write(line + "\n");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FileWriter::write(std::string data) {
  std::ofstream dataFile;
  dataFile.open(this->filename_, std::ios::app);
  dataFile << data;
  dataFile.close();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
