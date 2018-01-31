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

#ifndef KINEMATIC_TASK_POSE_HPP
#define KINEMATIC_TASK_POSE_HPP

#include <oscr/ik/kine-task.hpp>


namespace oscr{

/**
 * Kinematic task for 6D placement. This task can control the position,
 * orientation and full pose of an operational point (a.k.a. a link)
 */
class KineTaskPose :
  public KineTask
{
public:

  /**
   * @brief Constructor
   * @param[in] model robot model
   * @param[in] linkNum link number to be controlled
   * @param[in] taskType it can be "position", "orientation" or "pose"
   * @param[in] taskName name for the task
   */
  KineTaskPose(RobotModel* model,
               const unsigned int& linkNum,
               const std::string& taskType,
               const std::string& taskName = "6DTask");

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
   * Get the task type (0=position, 1=orientation, 2=full pose)
   */
  unsigned int getType() const;

  /**
   * Set the task type after initialization.
   *
   * @param[in] taskType It can be "position", "orientation", or "pose"
   */
  void setType(const std::string& taskType);

  /**
   * @brief Set a selection mask (only for position task).
   *
   * The selection mask selects only some degrees of freedom when the position
   * task is used; that is, it selects x, y and/or z in binary form (0/1). It
   * will leave without control the unselected elements of the task. If no mask
   * is selected, the default is to use x, y and z.
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


  // void getIntegralError(const Eigen::VectorXd& q,
  //                       Eigen::VectorXd& eint);

private:

  /// Link number
  unsigned int linkNum_;
  /// Local position with respect to the link frame
  Eigen::Vector3d localpos_;
  /// Indicator of task type
  enum PoseType{
    POSITION=0,
    ORIENTATION,
    FULLPOSE
  };
  /// Task type (POSITION, ORIENTATION or FULLPOSE)
  PoseType type_;
  /// Indices of the selected elements (using the mask)
  std::vector<unsigned int> selectedIndices_;

};

}

#endif
