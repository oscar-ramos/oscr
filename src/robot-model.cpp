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

#include <oscr/model/robot-model.hpp>
#include <oscr/tools/math-utils.hpp>

#include <fstream>
#include <iostream>


oscr::RobotModel::RobotModel(const bool& has_floating_base)
  :
  has_floating_base_(has_floating_base)
  ,jnames_(0)
{
}


oscr::RobotModel::~RobotModel()
{
}


std::vector<std::string> oscr::RobotModel::jointNames()
  const
{
  return jnames_;
}


Eigen::VectorXd oscr::RobotModel::jointMaxAngularLimits()
 const
{
  return qmax_;
}


Eigen::VectorXd oscr::RobotModel::jointMinAngularLimits()
  const
{
  return qmin_;
}


Eigen::VectorXd oscr::RobotModel::jointVelocityLimits()
  const
{
  return dqmax_;
}


std::map<std::string, unsigned int> oscr::RobotModel::mapLinkNamesIDs()
  const
{
  return link_id_;
}


bool oscr::RobotModel::hasFloatingBase()
  const
{
  return has_floating_base_;
}


Eigen::VectorXd oscr::RobotModel::getJointConfig()
{
  return q_;
}


Eigen::VectorXd oscr::RobotModel::setFeetOnGround(const std::string& foot1,
                                                  const std::string& foot2,
                                                  const double& sole_dist)
{
  if (has_floating_base_)
  {
    if (q_.size()==0)
    {
      std::cerr << "No joint configuration found.\n"
                << "Please set the configuration with updateJointConfig()"
                << std::endl;
      return q_;
    }
    Eigen::VectorXd position1, position2;
    // Temporarily set the robot completely vertical (although the robot is
    // assumed to be vertical!)
    double w, ex, ey, ez;
    w = q_[3]; ex = q_[4]; ey = q_[5]; ez = q_[6];
    q_[3] = 1.0; q_[4] = 0.0; q_[5] = 0.0; q_[6] = 0.0;
    this->updateJointConfig(q_);
    // Position of feet (assuming body is vertical)
    position1 = this->linkPosition(foot1);
    position2 = this->linkPosition(foot2);
    q_[1] = (position1[1]+position2[1])/2.0;
    q_[2] = fabs(position1[2])+fabs(sole_dist);
    // Return the orientation to its initial value
    q_[3] = w; q_[4] = ex; q_[5] = ey; q_[6] = ez;
    this->updateJointConfig(q_);
    return q_;
  }
  else
  {
    std::cerr << "Robot has no floating base! ... "
              << "returning the current joint configuration" << std::endl;
    return q_;
  }
}


std::string oscr::RobotModel::floatingLink(const unsigned int& flink_id)
  const
{
  if (has_floating_base_)
  {
    std::map< std::string, unsigned int >::const_iterator it;
    for (it = link_id_.begin(); it != link_id_.end(); ++it)
    {
      if (it->second == flink_id)
        return it->first;
    }
    std::cerr << "No floating link found!" << std::endl;
    return "NONE";
  }
  else
  {
    std::cerr << "Robot has no floating link (or floating base)" << std::endl;
    return "NONE";
  }
}
