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

#ifndef OSIK_SOLVER_HPP
#define OSIK_SOLVER_HPP

#include <oscr/ik/kine-task.hpp>


namespace oscr{

/**
 * Base class for Operational Space Inverse Kinematic (OSIK) solvers. These
 * solvers give a velocity control law, based upon differential inverse
 * kinematics, which is then integrated to obtain a position control law.
 */
class OSIKSolver
{
public:

  /**
   * Constructor
   * @param[in] rmodel Robot model
   * @param[in] qinit Initial joint generalized configuration
   * @param[in] dt control time
   */
  OSIKSolver(RobotModel* rmodel,
             const Eigen::VectorXd& qinit,
             const double& dt);

  /**
   * Destructor
   */
  ~OSIKSolver();

  /**
   * Push a task into the stack of tasks
   *
   * @param[in] task Pointer to the kinematic task
   */
  void pushTask(KineTask* task);

  /**
   * Remove the last added task (pop) from the stack of tasks
   */
  void popTask();

  /**
   * Remove a task from the stack of tasks using the task name. If two or more
   * tasks have the same name, it removes the first one added to the stack.
   *
   * @param[in] taskName Name of the task to be removed
   */
  void removeTask(const std::string& taskName);

  /**
   * Get the stack containing pointers to the currently existing tasks
   *
   * @param stack_tasks stack of tasks
   */
  void getTaskStack(std::vector< KineTask* >& stack_tasks);

  /**
   * Solve the system and get the desired position control for the given
   * configuration
   *
   * @param[in] q current configuration
   * @param[out] qdes joint control
   */
  virtual void getPositionControl(const Eigen::VectorXd& q,
                                  Eigen::VectorXd& qdes) = 0;

  /**
   * Get the internal time. This time is increased after every call to
   * getPositionControl()
   *
   * @return internal control time
   */
  unsigned int getTime();

  void printStack();


  //KinematicTask getTaskByName(const std::string& taskName);


protected:

  /// Robot Model
  RobotModel* rmodel_;
  /// Discretization (control) time
  double dt_;
  /// Desired value for the joints
  Eigen::VectorXd qdes_;  //  (TODO: check this name)
  /// Stack of Tasks
  std::vector< KineTask* > taskStack_;
  /// Number of degrees of freedom
  unsigned int ndof_;
  /// Solver internal discrete time
  unsigned int time_;

};

}

#endif

