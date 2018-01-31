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

#include <iostream>
#include <oscr/ik/kine-task-pose.hpp>
#include <oscr/tools/math-utils.hpp>


namespace oscr{


KineTaskPose::KineTaskPose(RobotModel* model,
                           const unsigned int& linkNum,
                           const std::string& taskType,
                           const std::string& taskName)
  :
  KineTask(model, taskName),
  linkNum_(linkNum)
{
  localpos_ = Eigen::Vector3d::Zero();
  setType(taskType);
  selectedIndices_.resize(3);
  for (unsigned int i=0; i<3; ++i)
    selectedIndices_.at(i) = i;
}


void KineTaskPose::getSensedValue(Eigen::VectorXd& xsensed)
  const
{
  if (type_ == POSITION)
  {
    // xsensed = rmodel_->linkPosition(linkNum_, localpos_);
    Eigen::VectorXd position = rmodel_->linkPosition(linkNum_, localpos_);
    // Use only the selected degrees of freedom
    xsensed.resize(selectedIndices_.size());
    for(unsigned int i=0; i<selectedIndices_.size(); ++i)
      xsensed(i) = position(selectedIndices_.at(i));
  }
  if (type_ == ORIENTATION)
    xsensed = rmodel_->linkOrientation(linkNum_);
  if (type_ == FULLPOSE)
    xsensed = rmodel_->linkPose(linkNum_, localpos_);
}


void KineTaskPose::getTaskJacobian(Eigen::MatrixXd& Jacobian)
  const
{
  if (type_ == POSITION)
  {
    // Jacobian = rmodel_->linearJacobian(linkNum_, localpos_);
    Eigen::MatrixXd J = rmodel_->linearJacobian(linkNum_, localpos_);
    Jacobian.resize(selectedIndices_.size(), this->ndof_);
    for(unsigned int i=0; i<selectedIndices_.size(); ++i)
      Jacobian.row(i) = J.row(selectedIndices_.at(i));
  }
  else if (type_ == ORIENTATION)
  {
    // TODO: check this, right now only for position
    Eigen::MatrixXd Jang;
    Jang = rmodel_->angularJacobian(linkNum_);

    Eigen::VectorXd quat; getSensedValue(quat);
    Eigen::MatrixXd Tinv;
    Tinv.resize(4,3);
    Tinv <<
      -0.5*quat(1), -0.5*quat(2), -0.5*quat(3),
       0.5*quat(0),  0.5*quat(3), -0.5*quat(2),
      -0.5*quat(3),  0.5*quat(0),  0.5*quat(1),
       0.5*quat(2), -0.5*quat(1),  0.5*quat(0);
    Jacobian = Tinv*Jang;
  }
  else if (type_ == FULLPOSE)
  {
    // Geometric Jacobian
    Eigen::MatrixXd Jg;
    Jg = rmodel_->geometricJacobian(linkNum_, localpos_);
    // Matrix to map geometric Jacobian to task Jacobian that uses quaternion
    // for rotation
    Eigen::VectorXd pose; getSensedValue(pose);
    Eigen::MatrixXd Tinv;
    Tinv.resize(4,3);
    Tinv <<
      -0.5*pose(4), -0.5*pose(5), -0.5*pose(6),
       0.5*pose(3),  0.5*pose(6), -0.5*pose(5),
      -0.5*pose(6),  0.5*pose(3),  0.5*pose(4),
       0.5*pose(5), -0.5*pose(4),  0.5*pose(3);

    Jacobian.resize(7,ndof_);
    Jacobian.topRows(3) = Jg.topRows(3);
    Jacobian.bottomRows(4) = Tinv*Jg.bottomRows(3);
  }
}


void KineTaskPose::getDerivError(Eigen::VectorXd& de)
{
  Eigen::VectorXd xsensed;
  getSensedValue(xsensed);
  if (type_ == POSITION)
  {
    error_ = xdes_-xsensed;
    de = getGain(error_.norm())*error_;
  }
  else if (type_ == ORIENTATION)
  {
    // Here xdes and xsensed are quaternions (w,x,y,z)
    error_(0) = xdes_(0)*xsensed(0) +
               xdes_.tail(3).transpose()*xsensed.tail(3) - 1.0;
    error_.tail(3) = xsensed(0)*xdes_.tail(3) - xdes_(0)*xsensed.tail(3)
                    - skew(xdes_.tail(3))*xsensed.tail(3);
    de = getGain(error_.norm())*error_;

  }
  else if (type_ == FULLPOSE)
  {
    de.resize(7);
    error_.head(3) = xdes_.head(3)-xsensed.head(3);
    de.head(3) = getGain(error_.head(3).norm())*error_.head(3);

    error_(3) = xdes_(3)*xsensed(3) +
               xdes_.tail(3).transpose()*xsensed.tail(3) - 1.0;
    error_.tail(3) = xsensed(3)*xdes_.tail(3) - xdes_(3)*xsensed.tail(3)
                    - skew(xdes_.tail(3))*xsensed.tail(3);
    de.tail(4) = getGain(error_.tail(4).norm())*error_.tail(4);

  }
}


unsigned int KineTaskPose::getType()
  const
{
  return type_;
}


void KineTaskPose::setType(const std::string& taskType)
{
  if (!taskType.compare("position"))
  {
    type_ = POSITION;
    error_.resize(3);
  }
  else if (!taskType.compare("orientation"))
  {
    type_ = ORIENTATION;
    error_.resize(4);
  }
  else if (!taskType.compare("pose"))
  {
    type_ = FULLPOSE;
    error_.resize(7);
  }
  else
    std::cerr << "Unknown task type: " << taskType << std::endl;
}


void KineTaskPose::setMask(const std::vector<unsigned int>& mask)
{
  // Verify if mask size is 3, as it should be. If not, keep the previous
  // values.
  if (mask.size() != 3)
  {
    std::cerr << "[KineTaskPose]: Mask size for position must be 3!\n"
              << "... using the previous mask" << std::endl;
    return;
  }
  selectedIndices_.clear();
  for(unsigned int mi=0; mi<3; ++mi)
  {
    if (mask[mi]==1)
      selectedIndices_.push_back(mi);
  }
}


std::vector<unsigned int> KineTaskPose::getMask()
  const
{
  std::vector<unsigned int> mask(3,0);
  for(unsigned int i=0; i<selectedIndices_.size(); ++i)
  {
    mask[selectedIndices_.at(i)] = 1;
  }
  return mask;
}


// void KineTaskPose::getIntegralError(const Eigen::VectorXd& q,
//                                 Eigen::VectorXd& eint)
// {
//   Eigen::VectorXd xsensed, xdes;
//   getDesiredValue(xdes);
//   getSensedValue(q, xsensed);
//   esum_ = esum_ + (xsensed-xdes);
//   eint = -0.05*(esum_);
// }

}
