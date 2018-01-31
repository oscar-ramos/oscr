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
#include <oscr/ik/kine-task-com.hpp>
#include <oscr/tools/math-utils.hpp>


namespace oscr{


KineTaskCoM::KineTaskCoM(RobotModel* model,
                          const std::string& taskName)
  :
  KineTask(model, taskName)
{
  selectedIndices_.resize(3);
  for (unsigned int i=0; i<3; ++i)
    selectedIndices_.at(i) = i;
}


void KineTaskCoM::getSensedValue(Eigen::VectorXd& xsensed)
  const
{
  // xsensed = rmodel_->comPosition();
  Eigen::VectorXd com = rmodel_->comPosition();

  xsensed.resize(selectedIndices_.size());
  for(unsigned int i=0; i<selectedIndices_.size(); ++i)
  {
    xsensed(i) = com(selectedIndices_.at(i));
  }
}


void KineTaskCoM::getTaskJacobian(Eigen::MatrixXd& Jacobian)
  const
{
  // Jacobian = rmodel_->comJacobian();
  Eigen::MatrixXd Jcom = rmodel_->comJacobian();

  Jacobian.resize(selectedIndices_.size(), this->ndof_);
  for(unsigned int i=0; i<selectedIndices_.size(); ++i)
  {
    Jacobian.row(i) = Jcom.row(selectedIndices_.at(i));
  }
}


void KineTaskCoM::getDerivError(Eigen::VectorXd& de)
{
  Eigen::VectorXd xsensed;
  getSensedValue(xsensed);

  error_ = xdes_-xsensed;
  de = getGain(error_.norm())*error_;

}


void KineTaskCoM::setMask(const std::vector<unsigned int>& mask)
{
  // Verify if mask size is 3, as it should be. If not, keep the previous
  // values.
  if (mask.size() != 3)
  {
    std::cerr << "[KineTaskCoM]: Mask size for CoM must be 3!\n"
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


std::vector<unsigned int> KineTaskCoM::getMask()
  const
{
  std::vector<unsigned int> mask(3,0);
  for(unsigned int i=0; i<selectedIndices_.size(); ++i)
  {
    mask[selectedIndices_.at(i)] = 1;
  }
  return mask;
}


}
