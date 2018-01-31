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

#ifndef OSIK_SOLVER_HQP_HPP
#define OSIK_SOLVER_HQP_HPP

#include <oscr/ik/osik-solver.hpp>
#include <qpOASES.hpp>


namespace oscr
{

/**
 * Whole-body Operational Space Inverse Kinematics (OSIK) solver using
 * a hierarchical QP for tasks (HQP)
 */
class OSIKSolverHQP :
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
  OSIKSolverHQP(RobotModel* rmodel,
                const Eigen::VectorXd& qinit,
                const double& dt=0.010);

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
   * Indicate if the solver has joint limits
   *
   * @return true if the solver is using joint limits (position and velocity)
   */
  bool hasJointLimits();

  /**
   * Solve for the kinematics using a Hierarchical QP. The system is solved in
   * a prioritized way getting the desired position control for the given
   * configuration using a cascade of QPs, usually called a hierarchical QP or
   * HQP.
   *
   * @param[in] q current joint generalized configuration
   * @param[out] qdes joint position control (including free floating base, if
   *                  there is one)
   */
  void getPositionControl(const Eigen::VectorXd& q,
                          Eigen::VectorXd& qdes);

private:

  /// Options for QP
  qpOASES::Options qpOptions_;
  /// Lower bound for the joints
  Eigen::VectorXd qmin_;
  /// Upper bound for the joints
  Eigen::VectorXd qmax_;
  /// Maximum joint velocity
  Eigen::VectorXd dqmax_;
  // Indicator of joint limits
  bool has_joint_limits_;

};

}

#endif
