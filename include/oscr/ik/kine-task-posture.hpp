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

#ifndef KINEMATIC_TASK_POSTURE_HPP
#define KINEMATIC_TASK_POSTURE_HPP

#include <oscr/ik/kine-task.hpp>


namespace oscr{

/**
 * Kinematic task for the direct motion of some joints.
 */
class KineTaskPosture:
  public KineTask
{
public:

  /**
   * @brief Constructor
   * @param[in] model robot model
   * @param[in] jointNumbers indices of the joints to be controlled
   * @param[in] taskName name for the task
   */
  KineTaskPosture(RobotModel* model,
                  const std::vector<unsigned int>& jointNumbers,
                  const std::string& taskName = "PostureTask");

  /* Virtual functions */
  void getSensedValue(Eigen::VectorXd& xsensed) const;

  // TODO: Check this eigen or eigen ref????
  void getTaskJacobian(Eigen::MatrixXd& Jacobian) const;

  /**
   * Get the derivative of the task error. This task uses: \f$\dot e=-\lambda
   * e\f$ where \f$e=x-x^*\f$
   *
   * @param[out] de derivative of the task error
   */
  void getDerivError(Eigen::VectorXd& de);

  /**
   * @brief Set the indices of the joints to be directly controlled.
   *
   * @param[in] jointNumbers Indices of the desired joints
   */
  void setJointNumbers(const std::vector<unsigned int>& jointNumbers);

  /**
   * Get the indices of the joints that are controlled
   *
   * @return joint indices
   */
  std::vector<unsigned int> getJointNumbers() const;

protected:

  /// Indices of the selected elements
  std::vector<unsigned int> jointIndices_;
};

}

#endif
