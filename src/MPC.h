#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
  std::vector<double> _pathX;
  std::vector<double> _pathY;

 public:

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  std::vector<double> getPathX() const { return this->_pathX; }
  std::vector<double> getPathY() const { return this->_pathY; }
};

#endif /* MPC_H */
