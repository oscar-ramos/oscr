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

#ifndef ROBOT_MODEL_PIN_HPP
#define ROBOT_MODEL_PIN_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <oscr/model/robot-model.hpp>


/**
 * Robot Model using Pinocchio
 *
 */

namespace oscr {

class RobotModelPin:
  public RobotModel
{

public:

  /**
   * Constructor.
   * After this, the model has to be initialized using loadURDF()
   */
  RobotModelPin();

  /**
   * Constructor initializing the URDF model
   * @param[in] model_name path to the urdf describing the robot
   * @param[in] has_floating_base indicates that the robot has a floating base
   */
  RobotModelPin(const std::string& model_name,
                const bool& has_floating_base = true);

  /**
   * Destructor
   */
  ~RobotModelPin();

  //
  // Inherited methods from RobotModel (virtual functions)
  //

  bool loadURDF(const std::string& model_name,
                const bool& has_floating_base = true,
                const bool& verbose = false);

  unsigned int ndof() const;

  unsigned int ndofActuated() const;

  std::string floatingLink() const;

  void updateJointConfig(const Eigen::VectorXd& q);

  Eigen::Vector3d linkPosition(const unsigned int& link_number,
                               const Eigen::Vector3d& local_pos =
                                     Eigen::Vector3d::Zero()) const;
  Eigen::Vector3d linkPosition(const std::string& link_name,
                               const Eigen::Vector3d& local_pos =
                               Eigen::Vector3d::Zero()) const;

  Eigen::Vector4d linkOrientation(const unsigned int& link_number) const;
  Eigen::Vector4d linkOrientation(const std::string& link_name) const;

  Eigen::VectorXd linkPose(const unsigned int& link_number,
                           const Eigen::Vector3d& local_pos =
                                 Eigen::Vector3d::Zero()) const;
  Eigen::VectorXd linkPose(const std::string& link_name,
                           const Eigen::Vector3d& local_pos =
                                 Eigen::Vector3d::Zero()) const;

  Eigen::MatrixXd linearJacobian(const unsigned int& link_number,
                                 const Eigen::Vector3d& local_pos =
                                       Eigen::Vector3d::Zero()) const;

  Eigen::MatrixXd angularJacobian(const unsigned int& link_number) const;

  Eigen::MatrixXd geometricJacobian(const unsigned int& link_number,
                                    const Eigen::Vector3d& local_pos =
                                          Eigen::Vector3d::Zero()) const;

  Eigen::VectorXd comPosition();

  Eigen::MatrixXd comJacobian() const;

private:

  /// Internal Pinocchio model
  se3::Model* pmodel_;

  se3::Data* data_;

  /**
   * Get the link names and IDs.
   * @param[out] link_names link names
   * @param[out] link_id link id (corresponding to the respective link name)
   */
  void linkNames(std::vector<std::string>& link_names,
                 std::vector<unsigned int>& link_ids) const;

};

}

#endif
