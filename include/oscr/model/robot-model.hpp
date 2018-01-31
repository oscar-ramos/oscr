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

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

/**
 * Base Robot Model.
 * All the model parsing is implemented in the derived classes.
 *
 */
namespace oscr {

  /**
   * Base class for the model of a robot
   */
class RobotModel
{

public:

  /**
   * Constructor for the base model class
   * @param[in] has_floating_base indicates that the robot has a floating base
   */
  RobotModel(const bool& has_floating_base = true);

  /**
   * Destructor
   */
  ~RobotModel();

  /**
   * Get information about whether or not a floating base exists
   * @return true if the robot model has a floating base
   */
  bool hasFloatingBase() const;

  /**
   * Get the joint names. It does not include the floating base.
   * @return joint names
   */
  std::vector<std::string> jointNames() const;

  /**
   * Get the maximum joint angluar limits. It does not include the robot
   * floating base.
   * @return joint angular limits (maximum)
   */
  Eigen::VectorXd jointMaxAngularLimits() const;

  /**
   * Get the minimum joint angluar limits. It does not include the robot
   * floating base.
   * @return joint angular limits (minimum)
   */
  Eigen::VectorXd jointMinAngularLimits() const;

  /**
   * Get the joint velocity limits. It does not include the floating base.
   * @return joint velocity limits (maximum)
   */
  Eigen::VectorXd jointVelocityLimits() const;

  /**
   * Get the map between the link names and their IDs.
   * @return map containing the link names and their respective link ids
   */
  std::map<std::string, unsigned int> mapLinkNamesIDs() const;

  /**
   * Get the current joint configuration.
   *
   * @return current joint configuration
   */
  Eigen::VectorXd getJointConfig();

  /**
   * Load the URDF model
   * @param[in] model_name path to the urdf describing the robot
   * @param[in] has_floating_base indicates that the robot has a floating base
   * @return true if successfully loaded
   */
  virtual bool loadURDF(const std::string& model_name,
                        const bool& has_floating_base = true,
                        const bool& verbose = false) = 0;

  /**
   * Get total number of degrees of freedom. It includes the floating base, if
   * the robot has one, otherwise, it is equivalent to ndofActuated().
   * @return degrees of freedom
   */
  virtual unsigned int ndof() const = 0;

  /**
   * Get number of actuated degrees of freedom. It excludes the floating base.
   * @return actuated degrees of freedom
   */
  virtual unsigned int ndofActuated() const = 0;

  /**
   * Get the link corresponding to the floating base (link number 0)
   * @return floating link
   */
  virtual std::string floatingLink() const = 0;


  /**
   * Update the joint configuration.
   *
   * @param[in] q generalized joint configuration (it must include the floating
   *              base configuration, if the robot has one)
   */
  virtual void updateJointConfig(const Eigen::VectorXd& q) = 0;


  /**
   * Get the position of a point (\f$x \in \mathbb{R}^3\f$) in a link with
   * respect to the world frame. If the robot does not have a floating base,
   * the world frame coincides with the robot base frame.
   *
   * @param[in] link_number link number for which the position will be computed
   * @param[in] local_pos point with respect to the link frame (offset).
   * @return the position \f$(x.y.z)\f$ with respect to the world frame
   */
  virtual Eigen::Vector3d linkPosition(const unsigned int& link_number,
                                       const Eigen::Vector3d& local_pos =
                                             Eigen::Vector3d::Zero())
    const = 0;

  /**
   * Get the position of a point (\f$x \in \mathbb{R}^3\f$) in a link with
   * respect to the world frame. If the robot does not have a floating base,
   * the world frame coincides with the robot base frame.
   *
   * @param[in] link_name link name for which the position will be computed
   * @param[in] local_pos point with respect to the link frame (offset).
   * @return the position \f$(x.y.z)\f$ with respect to the world frame
   */
  virtual Eigen::Vector3d linkPosition(const std::string& link_name,
                                       const Eigen::Vector3d& local_pos =
                                       Eigen::Vector3d::Zero())
  const = 0;

  /**
   * Get the orientation of a link with respect to the world frame. If the
   * robot does not have a floating base, the world frame coincides with the
   * robot base frame.
   *
   * @param[in] link_number link number for which the orientation will be
   *                        computed
   * @return the orientation (quaternion) with respect to the world frame
   */
  virtual Eigen::Vector4d linkOrientation(const unsigned int& link_number)
    const = 0;

  /**
   * Get the orientation of a link with respect to the world frame. If the
   * robot does not have a floating base, the world frame coincides with the
   * robot base frame.
   *
   * @param[in] link_name link name for which the orientation will be
   *                      computed
   * @return the orientation (quaternion) with respect to the world frame
   */
  virtual Eigen::Vector4d linkOrientation(const std::string& link_name)
    const = 0;

  /**
   * Get the pose of a link with respect to the world frame. If the robot does
   * not have a floating base, the world frame coincides with the robot base
   * frame.
   *
   * @param link_number link number for which the pose will be computed
   * @param local_pos point with respect to the link frame (offset)
   * @return the pose \f$(x,y,z,w,ex,ey,ez)\f$ which is Cartesian+quaternion
   */
  virtual Eigen::VectorXd linkPose(const unsigned int& link_number,
                                   const Eigen::Vector3d& local_pos =
                                         Eigen::Vector3d::Zero())
  const = 0;

  /**
   * Get the pose of a link with respect to the world frame. If the robot does
   * not have a floating base, the world frame coincides with the robot base
   * frame.
   *
   * @param link_name link name for which the pose will be computed
   * @param local_pos point with respect to the link frame (offset)
   * @return the pose \f$(x,y,z,w,ex,ey,ez)\f$ which is Cartesian+quaternion
   */
  virtual Eigen::VectorXd linkPose(const std::string& link_name,
                                   const Eigen::Vector3d& local_pos =
                                         Eigen::Vector3d::Zero())
    const = 0;

  /**
   * Get the Jacobian corresponding to the linear velocity.

   * This Jacobian is \f$J_{v} \in \mathbb{R}^{3 \times n}\f$ such that \f$v =
   * J_{v} \dot q\f$ where \f$v \in \mathbb{R}^3\f$ is the linear velocity,
   * \f$\dot q\f$ is the generalized joint velocity, and \f$n\f$ is the total
   * number of degrees of freedom (including the free floating base, if the
   * robot has one).
   *
   * @param[in] link_number link number for which the Jacobian will be computed
   * @param[in] local_pos point with respect to the link
   * @return linear velocity Jacobian \f$J_{v}\f$
   */
  virtual Eigen::MatrixXd linearJacobian(const unsigned int& link_number,
                                         const Eigen::Vector3d& local_pos =
                                               Eigen::Vector3d::Zero())
    const = 0;

  /**
   * Get the Jacobian corresponding to the angular velocity. That is,
   * \f$J_{\omega} \in \mathbb{R}^{3 \times n}\f$ such that \f$\omega =
   * J_{\omega} \dot q\f$ where \f$\omega \in \mathbb{R}^3\f$ is the angular
   * velocity, \f$\dot q\f$ is the generalized joint velocity, and \f$n\f$ is
   * the total number of degrees of freedom (including the free floating base,
   * if the robot has one).
   *
   * @param[in] link_number link number for which the Jacobian will be computed
   * @return angular velocity Jacobian \f$J_{\omega}\f$
   */
  virtual Eigen::MatrixXd angularJacobian(const unsigned int& link_number)
    const = 0;

  /**
   * Get the full geometric Jacobian. That is, \f$J = \begin{bmatrix}J_v \\
   * J_{\omega}\end{bmatrix} \in \mathbb{R}^{6 \times n}\f$ such that \f$
   * \begin{bmatrix}v \\ \omega\end{bmatrix} = J \dot q\f$ where \f$v \in
   * \mathbb{R}^3\f$ is the linear velocity, \f$\omega \in \mathbb{R}^3\f$ is
   * the angular velocity, \f$\dot q\f$ is the generalized joint velocity, and
   * \f$n\f$ is the total number of degrees of freedom (including the free
   * floating base, if the robot has one).
   *
   * @param[in] link_number link number for which the Jacobian will be computed
   * @param[in] local_pos point with respect to the link
   * @return geometric Jacobian \f$\begin{bmatrix}J_v \\
   *         J_{\omega}\end{bmatrix}\f$
   */
  virtual Eigen::MatrixXd geometricJacobian(const unsigned int& link_number,
                                            const Eigen::Vector3d& local_pos =
                                                  Eigen::Vector3d::Zero())
    const = 0;

  /**
   * Set floating base for both feet on the ground (robot is vertical!).
   *
   * The floating base position (not orientation) is updated to be above the
   * middle of both feet, with both feet on the ground. The height of both feet
   * is assumed to be constant (they are assumed to lie on a horizontal
   * plane). Note that the robot is assumed to be in a vertical posture.
   *
   * @param[in] foot1 name of the first foot
   * @param[in] foot2 name of the second foot
   * @param[in] sole_dist (vertical) distance between the feet and the sole
   *            (ground).
   * @return The modified generalized joint configuration
   */
  Eigen::VectorXd setFeetOnGround(const std::string& foot1,
                                  const std::string& foot2,
                                  const double& sole_dist);

  /**
   * Get the robot Center of Mass with respect to the world frame.
   *
   * @return Center of Mass
   */
  virtual Eigen::VectorXd comPosition() = 0;

  /**
   * Get the Center of Mass Jacobian.
   *
   * @return center of mass Jacobian \f$J_{com}\f$
   */
  virtual Eigen::MatrixXd comJacobian() const = 0;


protected:

  /// Indicator of floating base
  bool has_floating_base_;
  /// Joint names
  std::vector<std::string> jnames_;
  /// Joint angular limits (minimum) for revolute joints
  Eigen::VectorXd qmin_;
  /// Joint angular limits (maximum) for revolute joints
  Eigen::VectorXd qmax_;
  /// Joint velocity limits (maximum) for revolute joints
  Eigen::VectorXd dqmax_;
  // Joint configuration
  Eigen::VectorXd q_;
  /// Map between link names and link IDs
  std::map< std::string, unsigned int > link_id_;

  /**
   * Helper to get the link corresponding to the floating base
   * @param[in] flink_id ID of the floating link
   * @return floating link
   */
  std::string floatingLink(const unsigned int& flink_id) const;

};

}

#endif
