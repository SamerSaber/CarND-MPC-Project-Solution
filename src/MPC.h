#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  static void init(size_t N, double dt, double Lf, double ref_v);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  static size_t N;
  static double dt;
  static double Lf;
  static double ref_v;
  static bool is_initialized;

};

#endif /* MPC_H */
