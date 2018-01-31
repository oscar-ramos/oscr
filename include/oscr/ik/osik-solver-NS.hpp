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

#ifndef OSIK_SOLVER_NS_HPP
#define OSIK_SOLVER_NS_HPP


#include <iostream>
#include <oscr/ik/osik-solver.hpp>


namespace oscr
{

/**
 * Whole-body Operational Space Inverse Kinematics (OSIK) solver using
 * projections onto nullspaces.
 */
class OSIKSolverNS :
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
  OSIKSolverNS(RobotModel* rmodel,
               const Eigen::VectorXd& qinit,
               const double& dt=0.005);

  /**
   * Solve for the kinematics using nullspace projections. The system is solved
   * in a prioritized way getting the desired position control for the given
   * configuration through projections onto the nullspaces of tasks with higher
   * priorities.
   *
   * @param[in] q current joint generalized configuration
   * @param[out] qdes joint position control (including free floating base, if
   *                  there is one)
   */
  void getPositionControl(const Eigen::VectorXd& q,
                          Eigen::VectorXd& qdes);

};

}

#endif
