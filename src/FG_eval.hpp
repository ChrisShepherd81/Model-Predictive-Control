/*
 * FG_eval.hpp
 *
 *  Created on: 09.06.2017
 *      Author: christian@inf-schaefer.de
 */
#ifndef SRC_FG_EVAL_HPP_
#define SRC_FG_EVAL_HPP_

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//Values setted in main.cpp
extern size_t N;
extern double dt;

extern AD<double> cte_eval_weigth;
extern AD<double> epsi_eval_weigth;
extern AD<double> delta_eval_weigth;
extern AD<double> delta_diff_eval_weigth;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
extern double ref_v;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval
{
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

 public:
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void operator()(ADvector& fg, const ADvector& vars)
  {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`.
    fg[0] = this->calculateCost(vars);

    // Setup Constraints
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * (x0*x0) + coeffs[3] * (x0*x0*x0);
      AD<double> psides0 = CppAD::atan(coeffs[1]+(2*coeffs[2] * x0)+(3*coeffs[3] * (x0*x0)));

      //Calculate next constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + ( ( v0/ Lf) * delta0 * dt));
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + delta0 * (v0 / Lf) * dt);
    }
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  AD<double> calculateCost(const ADvector& vars)
  {
    AD<double> cost = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      cost += cte_eval_weigth*CppAD::pow(vars[cte_start + t], 2); //Cross track errors
      cost += epsi_eval_weigth*CppAD::pow(vars[epsi_start + t], 2); //error psi
      cost += CppAD::pow(vars[v_start + t] - ref_v, 2); // speed
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      cost += delta_eval_weigth*CppAD::pow(vars[delta_start + t], 2);
      cost += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      cost += delta_diff_eval_weigth*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      cost += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    return cost;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
};

#endif /* SRC_FG_EVAL_HPP_ */
