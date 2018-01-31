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

#include <fstream>
#include <iostream>

#include <oscr/tools/math-utils.hpp>
#include <oscr/model/robot-model-ff6dof.hpp>


oscr::
RobotModelff6dof::RobotModelff6dof(const bool& has_floating_base)
  :
  RobotModel(has_floating_base),
  la1_(0.20),
  la2_(0.20),
  la3_(0.05),
  ll1_(0.20),
  ll2_(0.20),
  ll3_(0.05),
  yt_(0.2),
  zt_(0.2)
{
  loadURDF("simpleff6dof", has_floating_base);
}


oscr::RobotModelff6dof::~RobotModelff6dof()
{
}


bool oscr::RobotModelff6dof::loadURDF(const std::string& model_name,
                                      const bool& has_floating_base,
                                      const bool& verbose)
{
  has_floating_base_ = has_floating_base;
  ndof_act_ = 6;

  jnames_.clear();
  jnames_.resize(ndof_act_);
  // The joint names must coincide with those in model/ff6dof.xacro
  jnames_[0] = "larm_j1";
  jnames_[1] = "larm_j2";
  jnames_[2] = "larm_j3";
  jnames_[3] = "lleg_j1";
  jnames_[4] = "lleg_j2";
  jnames_[5] = "lleg_j3";

  qmin_.resize(ndof_act_); qmax_.resize(ndof_act_); dqmax_.resize(ndof_act_);
  // These values must coincide with those in model/ff6dof.xacro
  qmin_ << -1, -2.5, -2.5, -2.75, -2.5, -2.5;
  qmax_ << 2.75, 2.5, 2.5, 1, 2.5, 2.5;
  dqmax_ << 1, 1, 1, 1, 1, 1;

  // Map for links and IDs (IDs are fictitious, just for coherence with generic
  // models
  link_id_["larm1"] = 1;
  link_id_["larm2"] = 2;
  link_id_["larm3"] = 3;
  link_id_["lleg1"] = 4;
  link_id_["lleg2"] = 5;
  link_id_["lleg3"] = 6;

  return true;
}


unsigned int oscr::RobotModelff6dof::ndof()
  const
{
  if (has_floating_base_)
    return ndof_act_ + 7;
  else
    return ndof_act_;
}


unsigned int oscr::RobotModelff6dof::ndofActuated()
  const
{
  return ndof_act_;
}


std::string oscr::RobotModelff6dof::floatingLink()
  const
{
  return "TODO";
}


void oscr::RobotModelff6dof::updateJointConfig(const Eigen::VectorXd& q)
{
  q_ = q;
}


Eigen::Vector3d
oscr::RobotModelff6dof::linkPosition(const unsigned int& link_number,
                                     const Eigen::Vector3d& local_pos)
  const
{
  // TODO: there is currently no support for local_pos
  Eigen::Vector3d res; res.setZero();
  unsigned int b = has_floating_base_ ? 7 : 0;
  if (link_number == 1)
  {
    res(1) = yt_/2.0;
    res(2) = zt_/2.0;
  }
  else if (link_number == 2)
  {
    res(1) = yt_/2.0 + la1_*cos(q_(0+b));
    res(2) = zt_/2.0 + la1_*sin(q_(0+b));
  }
  else if (link_number == 3)
  {
    res(1) = yt_/2.0 + la1_*cos(q_(0+b)) + la2_*cos(q_(b+0)+q_(b+1));
    res(2) = zt_/2.0 + la1_*sin(q_(0+b)) + la2_*sin(q_(b+0)+q_(b+1));
  }
  else if (link_number == 4)
  {
    res(1) = yt_/2.0;
    res(2) = -zt_/2.0;
  }
  else if (link_number == 5)
  {
    res(1) =  yt_/2.0 + ll1_*cos(q_(3+b));
    res(2) = -zt_/2.0 + ll1_*sin(q_(3+b));
  }
  else if (link_number == 6)
  {
    res(1) =  yt_/2.0 + ll1_*cos(q_(3+b)) + ll2_*cos(q_(b+3)+q_(b+4));
    res(2) = -zt_/2.0 + ll1_*sin(q_(3+b)) + ll2_*sin(q_(b+3)+q_(b+4));
  }
  else
  {
    std::cerr << "ERROR: No link has ID " << link_number << std::endl;
  }

  if (has_floating_base_)
  {
    // TODO: add floating part
    Eigen::Matrix3d Rbase;
    Rbase = quaternionToRotation(q_.segment(3,4));
    res = Rbase*res + q_.head(3);
  }
  return res;

}


Eigen::Vector3d
oscr::RobotModelff6dof::linkPosition(const std::string& link_name,
                                     const Eigen::Vector3d& local_pos)
const
{
  try{
    return linkPosition(link_id_.at(link_name), local_pos);
  }
  catch (std::exception& e)
  {
    std::cerr << "WARNING: Link name " << link_name << " is invalid! ... "
              <<  "Returning zeros" << std::endl;
    return Eigen::Vector3d::Zero();
  }
}


Eigen::Vector4d
oscr::RobotModelff6dof::linkOrientation(const unsigned int& link_number)
  const
{
  Eigen::Vector4d res; res.setZero();
  unsigned int b = has_floating_base_ ? 7 : 0;
  if (link_number == 1)
  {
    res(0) = cos(q_(b+0)/2.0);
    res(1) = sin(q_(b+0)/2.0);
  }
  else if (link_number == 2)
  {
    res(0) = cos( (q_(b+0)+q_(b+1)) / 2.0);
    res(1) = sin( (q_(b+0)+q_(b+1)) / 2.0);
  }
  else if (link_number == 3)
  {
    res(0) = cos( (q_(b+0)+q_(b+1)+q_(b+2)) / 2.0);
    res(1) = sin( (q_(b+0)+q_(b+1)+q_(b+2)) / 2.0);
  }
  else if (link_number == 4)
  {
    res(0) = cos(q_(b+3)/2.0);
    res(1) = sin(q_(b+3)/2.0);
  }
  else if (link_number == 5)
  {
    res(0) = cos( (q_(b+3)+q_(b+4)) / 2.0);
    res(1) = sin( (q_(b+3)+q_(b+4)) / 2.0);
  }
  else if (link_number == 6)
  {
    res(0) = cos( (q_(b+3)+q_(b+4)+q_(b+5)) / 2.0);
    res(1) = sin( (q_(b+3)+q_(b+4)+q_(b+5)) / 2.0);
  }
  else
  {
    std::cerr << "ERROR: No link has ID " << link_number << std::endl;
  }

  if (has_floating_base_)
  {
    // TODO: add floating part
    return quaternionMult(q_.segment(3,4), res);
  }
  else
    return res;
}


Eigen::Vector4d
oscr::RobotModelff6dof::linkOrientation(const std::string& link_name)
  const
{
  try{
    return linkOrientation(link_id_.at(link_name));
  }
  catch (std::exception& e)
  {
    std::cerr << "WARNING: Link name " << link_name << " is invalid! ... "
              <<  "Returning zeros" << std::endl;
    return Eigen::Vector4d::Zero();
  }
}


Eigen::VectorXd
oscr::RobotModelff6dof::linkPose(const unsigned int& link_number,
                                 const Eigen::Vector3d& local_pos)
  const
{
  Eigen::VectorXd lpose(7);
  lpose.head(3) = linkPosition(link_number, local_pos);
  lpose.tail(4) = linkOrientation(link_number);
  return lpose;
}


Eigen::VectorXd
oscr::RobotModelff6dof::linkPose(const std::string& link_name,
                                 const Eigen::Vector3d& local_pos)
const
{
  try{
    return linkPose(link_id_.at(link_name), local_pos);
  }
  catch (std::exception& e)
  {
    std::cerr << "WARNING: Link name " << link_name << " is invalid! ... "
              <<  "Returning zeros" << std::endl;
    return Eigen::VectorXd::Zero(7,1);
  }
}


Eigen::MatrixXd
oscr::RobotModelff6dof::linearJacobian(const unsigned int& link_number,
                                       const Eigen::Vector3d& local_pos)
  const
{
  Eigen::MatrixXd J;
  J.resize(3, this->ndof()); J.setZero();

  unsigned int b = has_floating_base_ ? 7 : 0;
  if ( (link_number == 1) || (link_number == 4) )
  {
  }
  else if (link_number == 2)
  {
    J(1,b+0) = -la1_*sin(q_(b+0));
    J(2,b+0) = la1_*cos(q_(b+0));
  }
  else if (link_number == 3)
  {
    J(1,b+0) = -la1_*sin(q_(b+0)) - la2_*sin(q_(b+0)+q_(b+1));
    J(2,b+0) = la1_*cos(q_(b+0)) + la2_*cos(q_(b+0)+q_(b+1));
    J(1,b+1) = -la2_*sin(q_(b+0)+q_(b+1));
    J(2,b+1) = la2_*cos(q_(b+0)+q_(b+1));
  }
  else if (link_number == 5)
  {
    J(1,b+3) = -ll1_*sin(q_(b+3));
    J(2,b+3) = ll1_*cos(q_(b+3));
  }
  else if (link_number == 6)
  {
    J(1,b+3) = -ll1_*sin(q_(b+3)) - ll2_*sin(q_(b+3)+q_(b+4));
    J(2,b+3) =  ll1_*cos(q_(b+3)) + ll2_*cos(q_(b+3)+q_(b+4));
    J(1,b+4) = -ll2_*sin(q_(b+3)+q_(b+4));
    J(2,b+4) =  ll2_*cos(q_(b+3)+q_(b+4));
  }
  else
  {
    std::cerr << "ERROR: No link has ID " << link_number << std::endl;
  }

  if (has_floating_base_)
  {
    // TODO: add effect of floating base and increase size of J
    Eigen::MatrixXd T; T.resize(3,4);
    T <<
      -2.0*q_(4),  2.0*q_(3), -2.0*q_(6),  2.0*q_(5),
      -2.0*q_(5),  2.0*q_(6),  2.0*q_(3), -2.0*q_(4),
      -2.0*q_(6), -2.0*q_(5),  2.0*q_(4),  2.0*q_(3);
    Eigen::Matrix3d R;
    R = quaternionToRotation(q_.segment(3,4));

    J.leftCols(3) = Eigen::Matrix3d::Identity();
    J.middleCols(3,4) = skew(q_.head(3)-linkPosition(link_number))*T;
    J.rightCols(this->ndofActuated()) = R*J.rightCols(this->ndofActuated());
    return J;
  }
  else
  {
    return J;
  }
}


Eigen::MatrixXd
oscr::RobotModelff6dof::angularJacobian(const unsigned int& link_number)
  const
{
  Eigen::MatrixXd J;
  J.resize(3, this->ndof()); J.setZero();

  unsigned int b = has_floating_base_ ? 7 : 0;
  if (link_number == 1)
  {
    J(0,b+0) = 1.0;
  }
  else if (link_number == 2)
  {
    J(0,b+0) = 1.0;
    J(0,b+1) = 1.0;
  }
  else if (link_number == 3)
  {
    J(0,b+0) = 1.0;
    J(0,b+1) = 1.0;
    J(0,b+2) = 1.0;
  }
  else if (link_number == 4)
  {
    J(0,b+3) = 1.0;
  }
  else if (link_number == 5)
  {
    J(0,b+3) = 1.0;
    J(0,b+4) = 1.0;
  }
  else if (link_number == 6)
  {
    J(0,b+3) = 1.0;
    J(0,b+4) = 1.0;
    J(0,b+5) = 1.0;
  }
  else
  {
    std::cerr << "ERROR: No link has ID " << link_number << std::endl;
  }

  if (has_floating_base_)
  {
    // TODO: add effect of floating base and increase size of J
    return J;
  }
  else
  {
    return J;
  }
}


Eigen::MatrixXd
oscr::RobotModelff6dof::geometricJacobian(const unsigned int& link_number,
                                          const Eigen::Vector3d& local_pos)
  const
{
  Eigen::MatrixXd J;
  J.resize(6, this->ndof());
  J.topRows(3) = linearJacobian(link_number, local_pos);
  J.bottomRows(3) = angularJacobian(link_number);
  return J;
}


Eigen::VectorXd oscr::RobotModelff6dof::comPosition()
{
  Eigen::VectorXd com(3);
  return com;
}


Eigen::MatrixXd oscr::RobotModelff6dof::comJacobian()
  const
{

}
