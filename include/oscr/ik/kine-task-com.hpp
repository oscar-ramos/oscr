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

#ifndef KINEMATIC_TASK_COM_HPP
#define KINEMATIC_TASK_COM_HPP

#include <oscr/ik/kine-task.hpp>


namespace oscr{

/**
 * Kinematic task for the motion of the Center of Mass. This task can control
 * the position of the CoM.
 */
class KineTaskCoM:
  public KineTask
{
public:

  /**
   * @brief Constructor
   * @param[in] model robot model
   * @param[in] taskName name for the task
   */
  KineTaskCoM(RobotModel* model,
              const std::string& taskName = "CoMTask");

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
   * @brief Set a selection mask on some task dofs.
   *
   * The selection mask selects only some degrees of freedom of the CoM task;
   * that is, it selects x, y and/or z in binary form (0/1). It will leave
   * without control the unselected elements of the task. If no mask is
   * selected, the default is to use all the available degrees of freedom.
   *
   * @param[in] mask The format is 0/1. For example: 001 (only z)
   */
  void setMask(const std::vector<unsigned int>& mask);

  /**
   * Get the selection mask
   *
   * @return mask in 0/1 format.
   */
  std::vector<unsigned int> getMask() const;

protected:

  /// Indices of the selected elements (using the mask)
  std::vector<unsigned int> selectedIndices_;
};

}

#endif
