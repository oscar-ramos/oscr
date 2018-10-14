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

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
// #include <pinocchio/algorithm/operational-frames.hpp>

#include <oscr/tools/math-utils.hpp>
#include <oscr/model/robot-model-pin.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>


oscr::RobotModelPin::RobotModelPin()
  :
  RobotModel(),
  pmodel_(new se3::Model())
{
}


oscr::RobotModelPin::RobotModelPin(const std::string& model_name,
                                   const bool& has_floating_base)
  :
  RobotModel(has_floating_base),
  pmodel_(new se3::Model())
{
  loadURDF(model_name, has_floating_base);
}


oscr::RobotModelPin::~RobotModelPin()
{
  delete pmodel_;
}


bool oscr::RobotModelPin::loadURDF(const std::string& model_name,
                                   const bool& has_floating_base,
                                   const bool& verbose)
{
  has_floating_base_ = has_floating_base;

  if (has_floating_base)
    // TODO: Check the Joint for FreeFlyer, if it is correct
    se3::urdf::buildModel(model_name, se3::JointModelFreeFlyer(),
                          *pmodel_, verbose);
  else
    se3::urdf::buildModel(model_name, *pmodel_, verbose);

  data_ = new se3::Data(*pmodel_);

  jnames_.clear();
  jnames_ = pmodel_->names;

  // Remove the first joint "universe", if it exists
  unsigned int offset = 0;
  if (jnames_[0].compare("universe")==0)
  {
    if (false)
      std::cout << "Eliminating first universe joint" << std::endl;
    jnames_.erase(jnames_.begin());
    offset = 1;
  }

  qmin_.resize(jnames_.size());
  qmax_.resize(jnames_.size());
  dqmax_.resize(jnames_.size());

  qmin_  = pmodel_->lowerPositionLimit;
  qmax_  = pmodel_->upperPositionLimit;
  dqmax_ = pmodel_->velocityLimit;

  // Get the map for links and their IDs
  std::vector<std::string> link_names;
  std::vector<unsigned int> link_ids;
  linkNames(link_names, link_ids);
  for (unsigned int i=0; i<link_names.size(); ++i)
    link_id_[link_names[i]] = link_ids[i];

  // If free-floating base, eliminate the "root_joint" when displaying
  if (has_floating_base_)
  {
    jnames_.erase(jnames_.begin());
    Eigen::VectorXd tmp;
    tmp = qmin_; qmin_ = tmp.tail(jnames_.size());
    tmp = qmax_; qmax_ = tmp.tail(jnames_.size());
    tmp = dqmax_; dqmax_ = tmp.tail(jnames_.size());
  }

  // Continuous joints are given spurious values par default, set those values
  // to arbitrary ones
  for (unsigned int i=0; i<qmin_.size(); ++i)
  {
    double d = qmax_[i]-qmin_[i];
    // If wrong values or if difference less than 0.05 deg (0.001 rad)
    if ( (d<0) || (fabs(d)<0.001) )
    {
      qmin_[i]  = -20.0;
      qmax_[i]  =  20.0;
      dqmax_[i] = 100.0;
    }
  }

  std::cout << "Model loaded: " << model_name << std::endl;
  return true;
}


unsigned int oscr::RobotModelPin::ndof()
  const
{
  return pmodel_->nq;
}


unsigned int oscr::RobotModelPin::ndofActuated()
  const
{
  if (has_floating_base_)
    // Eliminate the Cartesian position and orientation (quaternion)
    return pmodel_->nq-7;
  else
    return pmodel_->nq;
}


std::string oscr::RobotModelPin::floatingLink()
  const
{
  return RobotModel::floatingLink(1);
}


void oscr::RobotModelPin::linkNames(std::vector<std::string>& link_names,
                                    std::vector<unsigned int>& link_ids)
  const
{
  link_names.clear();
  link_ids.clear();

  se3::container::aligned_vector<se3::Frame> frames;
  frames = pmodel_->frames;

  // IDs of joints
  unsigned int jointId, jointIdprev=-1;
  std::vector<unsigned int> id_fixed_joints;
  std::vector<unsigned int> id_joints;
  for(unsigned int i=0; i<frames.size(); ++i)
  {
    // For fixed joints: this will be used to discard links that do not move
    jointId = static_cast<unsigned int>
      (pmodel_->getFrameId(frames[i].name,se3::FIXED_JOINT));
    if (jointId != frames.size())
      id_fixed_joints.push_back(jointId);
    // For movable joints: this will be used to order the link numbers (in
    // Pinocchio the link numbers and joint numbers obtained from "frames" are
    // in a particularly different order)
    jointId = static_cast<unsigned int>
      (pmodel_->getFrameId(frames[i].name,se3::JOINT));
    if (jointId != frames.size())
    {
      // Only add if the current jointId is different from the previous one
      // (due to a 'feature' in pinocchio that repeats some joints in 'frames')
      if (jointId != jointIdprev)
        id_joints.push_back(jointId);
      jointIdprev = jointId;
    }
  }

  // IDs and names of links if their parent is not a fixed joint (convention:
  // e.g. if joint ID is 5, associated link ID is 6)
  unsigned int linkId;
  bool fixed_link = false;
  // When there is floating base, an additional base_link is used, then, no '1'
  // needs to be added here ... afterall, this seems to be not needed (when
  // there is floating base the link number changes (+1) but Pinocchio
  // internally takes care of it.
  // unsigned int bias = has_floating_base_ ? 0 : 1;
  for(unsigned int i=0; i<frames.size(); ++i)
  {
    linkId = static_cast<unsigned int> (pmodel_->getBodyId(frames[i].name));
    if (linkId != frames.size())
    {
      // Discard joints attached to fixed link
      for (unsigned int k=0; k<id_fixed_joints.size(); ++k)
      {
        if ((id_fixed_joints[k]+1) == linkId)
        {
          fixed_link = true;
          break;
        }
      }
      if (!fixed_link)
      {
        link_names.push_back(frames[i].name);
        // Map the link numbers to the associated parent joints
        for (unsigned int k=0; k<id_joints.size(); ++k)
        {
          if ((id_joints[k]+1) == linkId)
          {
            //linkId = k+bias;
            linkId = k+1;
            break;
          }
        }
        link_ids.push_back(linkId);
      }
      fixed_link = false;
    }
  }
}


void oscr::RobotModelPin::updateJointConfig(const Eigen::VectorXd& q)
{
  q_ = q;
  if (has_floating_base_)
  {
    // Change quaternion order: in oscr it is (w,x,y,z) and in Pinocchio it is
    // (x,y,z,w)
    Eigen::VectorXd qpin; qpin = q_;
    qpin[3] = q_[4];
    qpin[4] = q_[5];
    qpin[5] = q_[6];
    qpin[6] = q_[3];
    se3::forwardKinematics(*pmodel_, *data_, qpin);
    se3::computeJacobians(*pmodel_, *data_, qpin);
  }
  else
  {
    se3::forwardKinematics(*pmodel_, *data_, q_);
    se3::computeJacobians(*pmodel_, *data_, q_);
  }
}


Eigen::Vector3d oscr::RobotModelPin::linkPosition(const unsigned int& link_number,
                                                  const Eigen::Vector3d& local_pos)
  const
{
  // TODO: there is currently no support for local_pos
  return data_->oMi[link_number].translation();
}


Eigen::Vector3d oscr::RobotModelPin::linkPosition(const std::string& link_name,
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


Eigen::Vector4d oscr::RobotModelPin::linkOrientation(const unsigned int& link_number)
  const
{
  return rotationToQuaternion(data_->oMi[link_number].rotation());
}


Eigen::Vector4d oscr::RobotModelPin::linkOrientation(const std::string& link_name)
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


Eigen::VectorXd oscr::RobotModelPin::linkPose(const unsigned int& link_number,
                                              const Eigen::Vector3d& local_pos)
  const
{
  // TODO: there is currently no support for local_pos
  Eigen::VectorXd lpose(7);
  lpose.head(3) = data_->oMi[link_number].translation();
  lpose.tail(4) = rotationToQuaternion(data_->oMi[link_number].rotation());

  return lpose;
}


Eigen::VectorXd oscr::RobotModelPin::linkPose(const std::string& link_name,
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
oscr::RobotModelPin::linearJacobian(const unsigned int& link_number,
                                    const Eigen::Vector3d& local_pos)
  const
{
  se3::Data::Matrix6x J(6, pmodel_->nv); J.fill(0);
  // This Jacobian is in the LOCAL frame. It has to be transformed to the
  // WORLD frame (using the joint rotation matrix).
  se3::getJacobian<se3::LOCAL>(*pmodel_, *data_, link_number, J);
  // Note: For some reason, the Jacobian in the supposedly fixed frame is
  // different from the one in rbdl. Check why???

  if (has_floating_base_)
  {
    // Structure of J is:
    // [ Rworld_wrt_link*Rbase_wrt_world |
    //   Rworld_wrt_link*skew(Pbase_wrt_world-Plink_wrt_world)*R_base_wrt_world |
    //   Jq_wrt_link]
    Eigen::MatrixXd Jlin; Jlin.resize(3, this->ndof()); Jlin.setZero();
    Eigen::Matrix3d Rbase; Rbase = quaternionToRotation(q_.segment(3,4));
    Jlin.leftCols(3) =
      Rbase.transpose()*data_->oMi[link_number].rotation()*J.block(0,0,3,3);
    Jlin.rightCols(ndofActuated()) =
      (data_->oMi[link_number].rotation())*J.block(0,6,3,ndofActuated());

    Eigen::MatrixXd T; T.resize(3,4);
    T <<
      -2.0*q_(4),  2.0*q_(3), -2.0*q_(6),  2.0*q_(5),
      -2.0*q_(5),  2.0*q_(6),  2.0*q_(3), -2.0*q_(4),
      -2.0*q_(6), -2.0*q_(5),  2.0*q_(4),  2.0*q_(3);
    Jlin.middleCols(3,4) =
      data_->oMi[link_number].rotation()*J.block(0,3,3,3)*Rbase.transpose()*T;

    return Jlin;
  }
  else
  {
    // Transform Jacobian from local frame to base frame
    return (data_->oMi[link_number].rotation())*J.topRows(3);
  }
}


Eigen::MatrixXd oscr::RobotModelPin::angularJacobian(const unsigned int& link_number)
  const
{
  se3::Data::Matrix6x J(6,pmodel_->nv); J.fill(0);

  if (has_floating_base_)
  {
    Eigen::MatrixXd Jang; Jang.resize(3, this->ndof()); Jang.setZero();
    Eigen::Vector4d q;

    // Jacobian in global frame
    se3::getJacobian<se3::LOCAL>(*pmodel_, *data_, link_number, J);

    // The structure of J is: [0 | Rot_ff_wrt_world | Jq_wrt_world]
    Jang.rightCols(ndofActuated()) = J.block(3,6,3,ndofActuated());
    q = rotationToQuaternion(J.block(3,3,3,3));
    Jang.middleCols(3,4) <<
      -2.0*q(1),  2.0*q(0), -2.0*q(3),  2.0*q(2),
      -2.0*q(2),  2.0*q(3),  2.0*q(0), -2.0*q(1),
      -2.0*q(3), -2.0*q(2),  2.0*q(1),  2.0*q(0);
    return Jang;
  }
  else
  {
    // Jacobian in local frame
    se3::getJacobian<se3::LOCAL>(*pmodel_, *data_, link_number, J);
    // Transform Jacobian from local frame to base frame
    return (data_->oMi[link_number].rotation())*J.bottomRows(3);
  }

}


Eigen::MatrixXd oscr::RobotModelPin::geometricJacobian(const unsigned int& link_number,
                                                       const Eigen::Vector3d& local_pos)
  const
{
  se3::Data::Matrix6x J(6,pmodel_->nv); J.fill(0);
  // Jacobian in local (link) frame
  se3::getJacobian<se3::LOCAL>(*pmodel_, *data_, link_number, J);

  if (has_floating_base_)
  {
    Eigen::MatrixXd Jg; Jg.resize(6, this->ndof()); Jg.setZero();
    Eigen::Matrix3d Rbase; Rbase = quaternionToRotation(q_.segment(3,4));
    Jg.topLeftCorner(3,3) =
      Rbase.transpose()*data_->oMi[link_number].rotation()*J.block(0,0,3,3);
    Jg.topRightCorner(3,ndofActuated()) =
      (data_->oMi[link_number].rotation())*J.block(0,6,3,ndofActuated());
    Jg.bottomRightCorner(3,ndofActuated()) =
      (data_->oMi[link_number].rotation())*J.block(3,6,3,ndofActuated());

    Eigen::MatrixXd T; T.resize(3,4);
    T <<
      -2.0*q_(4),  2.0*q_(3), -2.0*q_(6),  2.0*q_(5),
      -2.0*q_(5),  2.0*q_(6),  2.0*q_(3), -2.0*q_(4),
      -2.0*q_(6), -2.0*q_(5),  2.0*q_(4),  2.0*q_(3);
    Jg.block(0,3,3,4) =
      data_->oMi[link_number].rotation()*J.block(0,3,3,3)*Rbase.transpose()*T;
    Jg.block(3,3,3,4) = T;

    return Jg;
  }
  else
  {
    // Transform Jacobians from local frame to base frame
    J.topRows(3) = (data_->oMi[link_number].rotation())*J.topRows(3);
    J.bottomRows(3) = (data_->oMi[link_number].rotation())*J.bottomRows(3);
    return J;
  }
}


Eigen::VectorXd oscr::RobotModelPin::comPosition()
{
  Eigen::Vector3d com;

  if (has_floating_base_)
  {
    // Change quaternion order: in oscr it is (w,x,y,z) and in Pinocchio it is
    // (x,y,z,w)
    Eigen::VectorXd qpin;

    qpin = q_;
    qpin[3] = q_[4];
    qpin[4] = q_[5];
    qpin[5] = q_[6];
    qpin[6] = q_[3];
    //Eigen::Vector3d com = se3::centerOfMass(*pmodel_, *data_, qpin);
    //std::cout << qpin.head(7).transpose() << std::endl;
    se3::centerOfMass(*pmodel_, *data_, qpin);

    // Eigen::Matrix3d Rbase; Rbase = quaternionToRotation(q_.segment(3,4));
    // com = Rbase*data_->com[0];
    com = data_->com[0];
  }
  else
  {
    //Eigen::Vector3d com = se3::centerOfMass(*pmodel_, *data_, q_);
    se3::centerOfMass(*pmodel_, *data_, q_);
    com = data_->com[0];
  }

  return com;

}


Eigen::MatrixXd oscr::RobotModelPin::comJacobian()
  const
{
  Eigen::MatrixXd Jcom;
  if (has_floating_base_)
  {
    // Change quaternion order: in oscr it is (w,x,y,z) and in Pinocchio it is
    // (x,y,z,w)
    Eigen::VectorXd qpin; qpin = q_;
    qpin[3] = q_[4];
    qpin[4] = q_[5];
    qpin[5] = q_[6];
    qpin[6] = q_[3];
    Jcom = se3::jacobianCenterOfMass(*pmodel_, *data_, qpin);
  }
  else
  {
    Jcom = se3::jacobianCenterOfMass(*pmodel_, *data_, q_);
  }
  return Jcom;
}
