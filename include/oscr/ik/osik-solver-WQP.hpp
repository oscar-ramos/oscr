/*
 * Copyright 2017, Oscar Ramos
 *
 * This file is part of oscr.
 *
 * oscr is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * oscr is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details. You should
 * have received a copy of the GNU Lesser General Public License along
 * with oscr. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OSIK_SOLVER_WQP_HPP
#define OSIK_SOLVER_WQP_HPP

#include <oscr/ik/osik-solver.hpp>
#include <qpOASES.hpp>


namespace oscr
{

/**
 * Whole-body Operational Space Inverse Kinematics (OSIK) solver using
 * a weighted scheme for tasks
 */
class OSIKSolverWQP :
  public OSIKSolver
{
public:

  /**
   * Constructor
   *
   * @param[in] rmodel Pointer to the robot model
   * @param[in] qinit Initial joint configuration
   * @param[in] dt control time (default is 10ms)
   */
  OSIKSolverWQP(RobotModel* rmodel,
                const Eigen::VectorXd& qinit,
                const double& dt=0.010);

  /**
   * Solve for the kinematics using a weighted scheme for tasks. The tasks are
   * weighted, based on QPs, and the desired position control for the given
   * configuration is obtained.
   *
   * @param[in] q current joint generalized configuration
   * @param[out] qdes joint position control (including free floating base, if
   *                  there is one)
   */
  void getPositionControl(const Eigen::VectorXd& q,
                          Eigen::VectorXd& qdes);

  /**
   * Set the angular joint limits and velocity joint limits
   *
   * @param[in] qmin angular lower bounds
   * @param[in] qmax angular upper bounds
   * @param[in] dqmax velocity limits
   */
  void setJointLimits(const Eigen::VectorXd& qmin,
                      const Eigen::VectorXd& qmax,
                      const Eigen::VectorXd& dqmax);

  /**
   * Set the regularization term. This is the coefficient of the
   * \f$\Vert\dot{\mathbf{q}}\Vert\f$ term in the QP problem.
   *
   * @param[in] regterm regularization term
   */
  void setRegularizationTerm(const double& regterm);

  /**
   * Get the regularization term. This is the coefficient of the
   * \f$\Vert\dot{\mathbf{q}}\Vert\f$ term in the QP problem.
   *
   * @return current regularization term
   */
  double getRegularizationTerm();

private:

  /// Lower bound for the joints
  Eigen::VectorXd qmin_;
  /// Upper bound for the joints
  Eigen::VectorXd qmax_;
  /// Maximum joint velocity
  Eigen::VectorXd dqmax_;
  /// Options for QP
  qpOASES::Options qpOptions_;
  /// Constant for the regularization term
  double lambda_q_;

};

}

#endif
