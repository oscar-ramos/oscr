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

#ifndef OSCR_MATH_UTILS_HPP
#define OSCR_MATH_UTILS_HPP

#include <Eigen/Dense>

namespace oscr{

/**
 * Compute the Moore-Penrose pseudoinverse
 * @param[in] input_mat Input matrix
 * @param[out] pseudo_inv_mat Pseudo inverse of the input matrix
 * @param[in] pinvtoler Tolerance for discarding null singular values
 */
void pinv(const Eigen::MatrixXd& input_mat,
          Eigen::MatrixXd& pseudo_inv_mat,
          const double& pinvtoler = 1.0e-6);

/**
 * Skew-symmetric matrix from vector
 * @param[in] w Input 3D vector
 * @return Skew symmetric matrix of w
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& w);

/**
 * Convert from roll pitch yaw to rotation matrix. Note that the roll is about
 * the 'x' axis, the pitch about the 'y' axis and the yaw about the 'z' axis.
 *
 * @param[in] rpy roll, pitch yaw angles \f$(\varphi_r,\varphi_p,\varphi_y)\f$
 * @return equivalent rotation matrix given by
 *         \f$R_z(\varphi_y)R_y(\varphi_p)R_x(\varphi_r)\f$
 * @see rotationToRPY()
 */
Eigen::Matrix3d RPYToRotation(const Eigen::Vector3d& rpy);

/**
 * Convert a rotation matrix to roll, pitch, yaw angles. For this
 * implementation, roll is about the 'x' axis, pitch about the 'y' axis and yaw
 * about the 'z' axis. Note that pitch is limited between \f$-\frac{\pi}{2}\f$
 * and \f$\frac{\pi}{2}\f$.
 *
 * @param[in] R rotation matrix \f$R \in SO(3)\f$
 * @return roll, pitch yaw angles \f$(\varphi_r,\varphi_p,\varphi_y)\f$ where
 *         \f$\varphi_r \in [-\pi,\pi]\f$, \f$\varphi_p \in
 *         [-\frac{\pi}{2},\frac{\pi}{2}]\f$, and \f$\varphi_y\in [-\pi,\pi]\f$
 * @see RPYToRotation()
 */
Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& R);

/**
 * Convert a rotation matrix to the axis/angle representation. This
 * representation is \f$w=(w_x,w_y,w_z)\f$ where the rotation angle is
 * \f$\theta=\Vert w \Vert\f$ and the unitary rotation axis is
 * \f$\frac{w}{\Vert w \Vert}\f$. Note that \f$R = e^{w}\f$.
 *
 * @param[in] R rotation matrix \f$R \in SO(3)\f$
 * @return axis/angle \f$w\f$.
 * @see rotationToAxisAngle()
 */
Eigen::Vector3d rotationToAxisAngle(const Eigen::Matrix3d& R);

/**
 * Convert an axis/angle representation to a rotation matrix. The axis/angle is
 * given as a vector \f$w=(w_x,w_y,w_z)\f$ where the rotation angle is
 * \f$\theta=\Vert w \Vert\f$ and the unitary axis is \f$\frac{w}{\Vert w
 * \Vert}\f$.
 *
 * @param w axis/angle vector \f$w\f$
 * @return rotation matrix
 * @see axisAngleToRotation()
 */
Eigen::Matrix3d axisAngleToRotation(const Eigen::Vector3d& w);

/**
 * Get a rotation matrix from an angle about an axis. This function is
 * specialized in rotations about 'x', 'y' and 'z' only.
 *
 * @param[in] axis axis of rotation. It can be 'x', 'y', 'z'
 * @param[in] angle angle of rotation (in radians).
 * @return rotation matrix
 */
Eigen::Matrix3d rotationMatrix(const char& axis,
                               const double& angle);

/**
 * Get a rotation matrix from an angle about an axis using the Rodrigues
 * formula. The axis is \f$w=(w_x,w_y,w_z)\f$ and it should be unitary (if it
 * is not unitary, it is internally normalized). The rotation angle about the
 * unitary axis is \f$\theta\f$.
 *
 * @param[in] axis axis (\f$w\f$) of rotation
 * @param[in] angle angle (\f$\theta\f$) of rotation [in radians].
 * @return rotation matrix
 */
Eigen::Matrix3d rotationMatrix(const Eigen::Vector3d& axis,
                               const double& angle);

/**
 * Convert a quaternion to a rotation matrix. The quaternion format is
 * \f$q=(\epsilon_w,\epsilon_x,\epsilon_y,\epsilon_z)\f$ where \f$\epsilon_w\f$
 * is the scalar component, and \f$(\epsilon_x,\epsilon_y,\epsilon_z)\f$ is the
 * vectorial component.
 *
 * @param quat quaternion \f$(\epsilon_w,\epsilon_x,\epsilon_y,\epsilon_z)\f$
 * @return rotation matrix
 * @see rotationToQuaternion()
 */
Eigen::Matrix3d quaternionToRotation(const Eigen::Vector4d& quat);

/**
 * Convert a rotation matrix to a unitary quaternion. The quaternion format is
 * \f$q=(\epsilon_w,\epsilon_x,\epsilon_y,\epsilon_z)\f$ where \f$w\f$ is the
 * scalar component, and \f$(\epsilon_x,\epsilon_y,\epsilon_z)\f$ is the
 * vectorial component.
 *
 * @param[in] R rotation matrix \f$R \in SO(3)\f$
 * @return quat quaternion \f$(\epsilon_w,\epsilon_x,\epsilon_y,\epsilon_z)\f$
 * @see quaternionToRotation()
 */
Eigen::Vector4d rotationToQuaternion(const Eigen::Matrix3d& R);

/**
 * Multiply 2 quaternions. The quaternion format is
 * \f$q=(\epsilon_w,\epsilon_x,\epsilon_y,\epsilon_z)\f$ where \f$w\f$ is the
 * scalar component, and \f$(\epsilon_x,\epsilon_y,\epsilon_z)\f$ is the
 * vectorial component.
 *
 * @param[in] q1 first quaternion \f$(\epsilon_{w_1},\epsilon_{x_1},
 *               \epsilon_{y_1},\epsilon_{z_1})\f$
 * @param[in] q2 second quaternion \f$(\epsilon_{w_2},\epsilon_{x_2},
 *               \epsilon_{y_2},\epsilon_{z_2})\f$
 * @return quaternion multiplication \f$q1 \circ q2\f$
 * @see quaternionToRotation()
 */
Eigen::Vector4d quaternionMult(const Eigen::Vector4d& q1,
                               const Eigen::Vector4d& q2);

/**
 * Sign function \f$\text{sgn}(x)\f$. If the argument is greater or equal to 0
 * (positive), the output is 1; otherwise, it is -1..
 *
 * @param[in] x input argument
 * @return sgn(x)
 */
double sgn(const double& x);


/**
 * Numerical value of infinity
 */
const double INFTY = 1.0e20;

}

#endif
