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

#include <oscr/tools/model-utils.hpp>
#include <oscr/tools/math-utils.hpp>
#include <iostream>


void oscr::printJointLimits(const std::vector<std::string>& jnames,
                            const Eigen::VectorXd& qmin,
                            const Eigen::VectorXd& qmax,
                            const Eigen::VectorXd& dqmax)
{
  if (!( (jnames.size() == qmin.size()) &&
         (jnames.size() == qmax.size()) &&
         (jnames.size() == dqmax.size())) )
  {
    std::cerr << "Joint names and joint limits size do not match!"
              << std::endl;
    return;
  }
  std::cout << "\nJoint Name\t qmin \t qmax \t dqmax" << std::endl;
  for (unsigned int i=0; i<jnames.size(); ++i)
    std::cout << jnames[i] << "\t\t" << qmin[i] << "\t" << qmax[i] << "\t"
              << dqmax[i] << std::endl;
}


void oscr::printLinkID(const std::map< std::string, unsigned int >& nid)
{
  std::cout << "\nMap link names and IDs" << std::endl;
  std::map<std::string, unsigned int>::const_iterator it;
  for (it=nid.begin(); it!=nid.end(); ++it)
    std::cout << it->first << " - " << it->second << std::endl;
}


void oscr::printLinkIDshort(const std::map< std::string, unsigned int >& nid)
{
  std::cout << "\nMap link names: IDs" << std::endl;
  std::map<std::string, unsigned int>::const_iterator it;
  for (it=nid.begin(); it!=nid.end(); ++it)
    std::cout << "(" << it->first << ": " << it->second << "), ";
  std::cout << "\b\b";
  std::cout << " " << std::endl;
}


void oscr::printModelInfo(oscr::RobotModel* robot)
{
  Eigen::VectorXd qmin, qmax, dqmax;
  std::vector<std::string> jnames;
  std::map<std::string, unsigned int> maplink_name_ID;

  // Joint angular and velocity limits
  qmin  = robot->jointMinAngularLimits();
  qmax  = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();
  // Joint names
  jnames = robot->jointNames();
  // Link names and IDs
  maplink_name_ID = robot->mapLinkNamesIDs();

  // Show the previous information on the screen
  std::cout << "Total ndof: " << robot->ndof() << std::endl;
  std::cout << "Total ndof actuated: " << robot->ndofActuated() << std::endl;
  std::cout << "Has floating base: " << robot->hasFloatingBase() << std::endl;
  std::cout << "Floating link: " << robot->floatingLink() << std::endl;
  oscr::printJointLimits(jnames, qmin, qmax, dqmax);
  oscr::printLinkID(maplink_name_ID);

}


void oscr::printStrVector(const std::vector<std::string>& strvector)
{
  for (unsigned int i=0; i<strvector.size(); ++i)
    std::cout << strvector[i] << " ";
  std::cout << std::endl;
}


std::map<std::string, unsigned int>
oscr::mapJointNames(const std::vector<std::string>& jnames,
                    const bool& has_floating_base)
{
  unsigned int bias = 0;
  if (has_floating_base)
    bias = 7;
  std::map<std::string, unsigned int> mjoint;
  for (unsigned int i=0; i<jnames.size(); ++i)
    mjoint[jnames.at(i)] = i+bias;
  return mjoint;
}


Eigen::VectorXd oscr::incPoseLocal(const Eigen::VectorXd& initial_pose,
                                   const double& dx, const double& dy,
                                   const double& dz, const double& angle,
                                   const double& axisx, const double& axisy,
                                   const double& axisz)
{
  Eigen::VectorXd Pf(7);
  // Increment in position
  Pf(0) = initial_pose(0)+dx;
  Pf(1) = initial_pose(1)+dy;
  Pf(2) = initial_pose(2)+dz;
  // Increment in orientation
  double ang2 = angle/360.0*M_PI;
  Eigen::Vector3d ax;
  ax << axisx, axisy, axisz;
  ax.normalize();
  Eigen::Vector4d dQ;
  dQ << cos(ang2), ax[0]*sin(ang2), ax[1]*sin(ang2), ax[2]*sin(ang2);
  // Increment with respect to local frame
  Pf.tail(4) = quaternionMult(initial_pose.tail(4), dQ);
  return Pf;
}


Eigen::VectorXd oscr::incPosition(const Eigen::VectorXd& initial_position,
                                  const double& dx, const double& dy,
                                  const double& dz)
{
  Eigen::VectorXd Pf(3);
  Pf(0) = initial_position(0) + dx;
  Pf(1) = initial_position(1) + dy;
  Pf(2) = initial_position(2) + dz;
  return Pf;
}
