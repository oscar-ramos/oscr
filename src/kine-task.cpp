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
#include <oscr/ik/kine-task.hpp>


namespace oscr{

KineTask::KineTask(RobotModel* model,
                   const std::string& taskName,
                   const double& gain)
  :
  rmodel_(model),
  name_(taskName),
  gain_(gain),
  weight_(1.0),
  adaptiveGain_(false),
  lmax_(gain),
  lmin_(gain),
  k_(0.0)
{
  ndof_ = model->ndof();
  q_.resize(0);
}


KineTask::~KineTask()
{
  //delete rmodel_;
}


void KineTask::setName(const std::string& taskName)
{
  name_ = taskName;
}


std::string KineTask::getName() const
{
  return name_;
}


void KineTask::setGain(const double& gain)
{
  // Set a constant gain
  adaptiveGain_ = false;
  gain_ = gain;
}


void KineTask::setAdaptiveGain(const double& max_gain,
                               const double& min_gain,
                               const double& rate_descent)
{
  // Set an adaptive gain
  adaptiveGain_ = true;
  std::cout << "Task " << name_ << ": Using adaptive gain" << std::endl;
  // Set the gains
  lmax_ = max_gain;
  lmin_ = min_gain;
  k_ = rate_descent;
  // The gain is set to the minimum gain
  gain_ = lmin_;
}


double KineTask::getGain(const double& error)
{
  if (adaptiveGain_)
  {
    std::cout << "Using adaptive gain" << std::endl;
    gain_ = (lmax_-lmin_)*exp(-k_*error)+lmin_;
  }
  return gain_;
}


// void KineTask::setDesiredValue(const Eigen::Ref<const Eigen::VectorXd>& desiredValue)
void KineTask::setDesiredValue(const Eigen::VectorXd& desiredValue)
{
  xdes_ = desiredValue;
  // Reset the error sum
  esum_ = Eigen::VectorXd::Zero(xdes_.size());
}


// void KineTask::getDesiredValue(Eigen::VectorXd& desiredValue)
//   const
// {
//   desiredValue = xdes_;
// }
Eigen::VectorXd KineTask::getDesiredValue()
  const
{
  return xdes_;
}


Eigen::VectorXd KineTask::getError()
  const
{
  return error_;
}


void KineTask::setWeight(const double& weight)
{
  weight_ = weight;
}


double KineTask::getWeight()
  const
{
  return weight_;
}


unsigned int KineTask::getTaskDim()
  const
{
  Eigen::VectorXd sensedValue;
  this->getSensedValue(sensedValue);
  return sensedValue.rows();
}


void KineTask::updateJointConfig(const Eigen::VectorXd& q)
{
  q_ = q;
  rmodel_->updateJointConfig(q_);
}


void KineTask::keep(const double& gain)
{
  Eigen::VectorXd config;

  setGain(gain);
  getSensedValue(config);
  setDesiredValue(config);
}


void KineTask::getTaskBounds(Eigen::VectorXd& bound_min,
                             Eigen::VectorXd& bound_max)
{
  getDerivError(bound_min);
  bound_max = bound_min;
}


}
