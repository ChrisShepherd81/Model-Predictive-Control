/*
 * FileWriter.h
 *
 *  Created on: 24.05.2017
 *      Author: christian@inf-schaefer.de
 */

#ifndef SRC_FILEWRITER_H_
#define SRC_FILEWRITER_H_

#include <fstream>
#include <sstream>
#include <vector>

class FileWriter {
 public:
  FileWriter(std::string filename, size_t polynomialGrade, size_t noOfWaypoints, size_t noOfPredictions);

  void writeData(const std::vector<double> &state, const std::vector<double> &coeffs,
                 const std::vector<double> &wayPointsX, const std::vector<double> &wayPointsY,
                 const std::vector<double> &predictionsX,const std::vector<double> &predictionsY);

 private:
  std::string filename_;
  void writeHeader(size_t polynomialGrade, size_t noOfWaypoints, size_t noOfPredictions);
  void writeLine(std::string line);
  void write(std::string data);
};

#endif /* SRC_FILEWRITER_H_ */
