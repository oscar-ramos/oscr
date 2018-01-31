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
#include <oscr/ik/kine-task-posture.hpp>
//#include <oscr/tools/math-utils.hpp>


namespace oscr{


KineTaskPosture::KineTaskPosture(RobotModel* model,
                                 const std::vector<unsigned int>& jointNumbers,
                                 const std::string& taskName)
  :
  KineTask(model, taskName)
{
  setJointNumbers(jointNumbers);
  // selectedIndices_.resize(3);
  // for (unsigned int i=0; i<3; ++i)
  //   selectedIndices_.at(i) = i;
}


void KineTaskPosture::getSensedValue(Eigen::VectorXd& xsensed)
  const
{
  Eigen::VectorXd q = rmodel_->getJointConfig();

  xsensed.resize(jointIndices_.size());
  for(unsigned int i=0; i<jointIndices_.size(); ++i)
  {
    xsensed(i) = q(7+jointIndices_.at(i));
  }
}


void KineTaskPosture::getTaskJacobian(Eigen::MatrixXd& Jacobian)
  const
{
  //Eigen::MatrixXd Jcom = rmodel_->comJacobian();
  Jacobian.resize(jointIndices_.size(), this->ndof_);
  Jacobian.setZero();
  for(unsigned int i=0; i<jointIndices_.size(); ++i)
  {
    Jacobian(i,jointIndices_[i]+7) = 1;
  }
}


void KineTaskPosture::getDerivError(Eigen::VectorXd& de)
{
  Eigen::VectorXd xsensed;
  getSensedValue(xsensed);

  error_ = xdes_-xsensed;
  de = getGain(error_.norm())*error_;

}


void KineTaskPosture::setJointNumbers(const std::vector<unsigned int>& jnumbers)
{
  // Verify if mask size is 3, as it should be. If not, keep the previous
  // values.
  jointIndices_ = jnumbers;
  // jointIndices_.clear();
  // for(unsigned int mi=0; mi<3; ++mi)
  // {
  //   if (mask(mi)==1)
  //     selectedIndices_.push_back(mi);
  // }
}


std::vector<unsigned int> KineTaskPosture::getJointNumbers()
  const
{
  return jointIndices_;
  // Eigen::VectorXd mask(3); mask.setZero();
  // for(unsigned int i=0; i<selectedIndices_.size(); ++i)
  // {
  //   mask(selectedIndices_.at(i)) = 1;
  // }
  // return mask;
}


}
