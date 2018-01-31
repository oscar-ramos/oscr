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

#ifndef ROBOT_MODEL_FF6DOF_HPP
#define ROBOT_MODEL_FF6DOF_HPP

#include <oscr/model/robot-model.hpp>

/**
 * Test of a simple robot with floating base and only 6dof
 *
 */
namespace oscr{

class RobotModelff6dof:
  public RobotModel
{
public:

  RobotModelff6dof(const bool& has_floating_base = true);

  ~RobotModelff6dof();

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

  // These lengths must coincide with the ones in urdf/ff6dof.xacro
  // const double la1_ = 0.20; // larm1_len
  // const double la2_ = 0.20; // larm2_len
  // const double la3_ = 0.05; // larm3_len
  // const double ll1_ = 0.20; // lleg1_len
  // const double ll2_ = 0.20; // lleg2_len
  // const double ll3_ = 0.05; // lleg3_len
  // const double yt_  = 0.2;  // torso_ylen
  // const double zt_  = 0.2;  // torso_zlen

  const double la1_; // larm1_len
  const double la2_; // larm2_len
  const double la3_; // larm3_len
  const double ll1_; // lleg1_len
  const double ll2_; // lleg2_len
  const double ll3_; // lleg3_len
  const double yt_;  // torso_ylen
  const double zt_;  // torso_zlen

  unsigned int ndof_act_;

};

}

#endif
