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

#include <oscr/tools/math-utils.hpp>
#include <iostream>
#include <vector>


void oscr::pinv(const Eigen::MatrixXd& matrix_in,
                Eigen::MatrixXd& pseudo_inv,
                const double& pinvtoler)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix_in, Eigen::ComputeThinU |
                                        Eigen::ComputeThinV);
  Eigen::VectorXd singular_values;
  Eigen::VectorXd singular_values_inv;
  singular_values = svd.singularValues();
  singular_values_inv.setZero(singular_values.size());

  for (int w = 0; w < singular_values.size(); ++w)
    if( singular_values(w) > pinvtoler)
      singular_values_inv(w) = 1/singular_values(w);
  pseudo_inv = svd.matrixV() * singular_values_inv.asDiagonal() *
    svd.matrixU().transpose();
  return;
}


Eigen::Matrix3d oscr::skew(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d R;
  R(0,0) = 0.0;   R(0,1) = -w(2); R(0,2) = w(1);
  R(1,0) = w(2);  R(1,1) = 0.0;   R(1,2) = -w(0);
  R(2,0) = -w(1); R(2,1) = w(0);  R(2,2) = 0.0;
  return R;
}


Eigen::Matrix3d oscr::RPYToRotation(const Eigen::Vector3d& rpy)
{
  Eigen::Matrix3d res;
  res = rotationMatrix('z',rpy(2))*rotationMatrix('y',rpy(1))
    *rotationMatrix('x',rpy(0));
  return res;
}


Eigen::Vector3d oscr::rotationToRPY(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d rpy;
  double m = sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2));
  rpy(1) = atan2(-R(2,0),m);

  if (fabs(rpy(1)-M_PI/2.0) < 0.001)
  {
    rpy(2) = 0;
    rpy(0) = atan2(R(0,1), R(1,1));
  }
  else if (fabs(rpy(1)+M_PI/2.0) < 0.001)
  {
    rpy(2) = 0;
    rpy(0) = -atan2(R(0,1), R(1,1));
  }
  else
  {
    rpy(2) = atan2(R(1,0), R(0,0));
    rpy(0) = atan2(R(2,1), R(2,2));
  }

  return rpy;
}


Eigen::Matrix3d oscr::quaternionToRotation(const Eigen::Vector4d& q)
{
  double normq = q.norm();
  if (fabs(normq-1.0)>0.001)
  {
    std::cerr << "WARNING: Input quaternion is not unitary! ... "
              << "Returning identity" << std::endl;
    return Eigen::Matrix3d::Identity();
  }
  Eigen::Matrix3d res;
  res(0,0) = 2.0*(q(0)*q(0)+q(1)*q(1))-1.0;
  res(0,1) = 2.0*(q(1)*q(2)-q(0)*q(3));
  res(0,2) = 2.0*(q(1)*q(3)+q(0)*q(2));
  res(1,0) = 2.0*(q(1)*q(2)+q(0)*q(3));
  res(1,1) = 2.0*(q(0)*q(0)+q(2)*q(2))-1.0;
  res(1,2) = 2.0*(q(2)*q(3)-q(0)*q(1));
  res(2,0) = 2.0*(q(1)*q(3)-q(0)*q(2));
  res(2,1) = 2.0*(q(2)*q(3)+q(0)*q(1));
  res(2,2) = 2.0*(q(0)*q(0)+q(3)*q(3))-1.0;

  return res;
}


Eigen::Vector4d oscr::rotationToQuaternion(const Eigen::Matrix3d& R)
{
  double dEpsilon = 1e-6;
  Eigen::Vector4d quat;

  quat(0) = 0.5*sqrt(R(0,0)+R(1,1)+R(2,2)+1.0);
  if ( fabs(R(0,0)-R(1,1)-R(2,2)+1.0) < dEpsilon )
    quat(1) = 0.0;
  else
    quat(1) = 0.5*oscr::sgn(R(2,1)-R(1,2))*sqrt(R(0,0)-R(1,1)-R(2,2)+1.0);
  if ( fabs(R(1,1)-R(2,2)-R(0,0)+1.0) < dEpsilon )
    quat(2) = 0.0;
  else
    quat(2) = 0.5*oscr::sgn(R(0,2)-R(2,0))*sqrt(R(1,1)-R(2,2)-R(0,0)+1.0);
  if ( fabs(R(2,2)-R(0,0)-R(1,1)+1.0) < dEpsilon )
    quat(3) = 0.0;
  else
    quat(3) = 0.5*oscr::sgn(R(1,0)-R(0,1))*sqrt(R(2,2)-R(0,0)-R(1,1)+1.0);

  return quat;
}


Eigen::Vector4d oscr::quaternionMult(const Eigen::Vector4d& q1,
                                     const Eigen::Vector4d& q2)
{
  Eigen::Vector4d res;
  res(0) = -q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) + q1(0)*q2(0);
  res(1) =  q1(0)*q2(1) - q1(3)*q2(2) + q1(2)*q2(3) + q1(1)*q2(0);
  res(2) =  q1(3)*q2(1) + q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0);
  res(3) = -q1(2)*q2(1) + q1(1)*q2(2) + q1(0)*q2(3) + q1(3)*q2(0);
  return res;
}


Eigen::Vector3d oscr::rotationToAxisAngle(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d res;

  double sinTh = 0.5*sqrt(  (R(1,0)-R(0,1))*(R(1,0)-R(0,1))
                           + (R(2,0)-R(0,2))*(R(2,0)-R(0,2))
                           + (R(2,1)-R(1,2))*(R(2,1)-R(1,2)) );
  double cosTh = 0.5*(R(0,0)+R(1,1)+R(2,2)-1.0);
  double theta = atan2(sinTh,cosTh);   // angle btw 0 and PI (s>0)

  const double ANGLE_MIN = 0.0001;
  const double SINC_MIN  = 1e-8;
  const double COSC_MIN  = 2.5e-4;

  if ( (sinTh>ANGLE_MIN) || (cosTh>0.0) )
  {
    double sinc = (fabs(sinTh) > SINC_MIN) ? (sinTh/theta) : 1.0;
    sinc = 1.0/(2.0*sinc);

    res(0) = sinc*(R(2,1)-R(1,2));
    res(1) = sinc*(R(0,2)-R(2,0));
    res(2) = sinc*(R(1,0)-R(0,1));
  }
  else
  {
    // Theta near PI
    res(0) = theta*(sqrt((R(0,0)-cosTh)/(1.0-cosTh)));
    if ((R(2,1)-R(1,2))<0)
      res(0) = -res(0);

    res(1) = theta*(sqrt((R(1,1)-cosTh)/(1.0-cosTh)));
    if ((R(0,2)-R(2,0))<0)
      res(1) = -res(1);

    res(2) = theta*(sqrt((R(2,2)-cosTh)/(1.0-cosTh)));
    if ((R(1,0)-R(0,1))<0)
      res(2) = -res(2);
  }
  return res;
}


Eigen::Matrix3d oscr::axisAngleToRotation(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d R, S;
  Eigen::Vector3d axis;

  double angle = w.norm();
  axis = w.normalized();
  S = skew(axis);
  R = Eigen::Matrix3d::Identity() + sin(angle)*S + (1-cos(angle))*S*S;

  return R;
}


Eigen::Matrix3d oscr::rotationMatrix(const char& axis,
                                     const double& angle)
{
  double ca = cos(angle);
  double sa = sin(angle);
  Eigen::Matrix3d res;

  if (axis=='x')
    res << 1, 0, 0, 0, ca, -sa, 0, sa, ca;
  else if (axis=='y')
    res << ca, 0, sa, 0, 1, 0, -sa, 0, ca;
  else if (axis=='z')
    res << ca, -sa, 0, sa, ca, 0, 0, 0, 1;
  else {
    std::cerr << "rotationMatrix: Valid options for axis are only x, y, z"
              << std::endl;
    res = Eigen::Matrix3d::Identity();
  }
  return res;
}


Eigen::Matrix3d oscr::rotationMatrix(const Eigen::Vector3d& axis,
                                     const double& angle)
{
  Eigen::Vector3d w;
  double normaxis = axis.norm();
  w = angle/normaxis*axis;
  return axisAngleToRotation(w);
}


double oscr::sgn(const double& x)
{
  if (x>=0)
    return 1.0;
  else
    return -1.0;
}
