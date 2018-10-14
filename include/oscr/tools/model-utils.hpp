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

#ifndef OSCR_MODEL_UTILS_HPP
#define OSCR_MODEL_UTILS_HPP

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <oscr/model/robot-model.hpp>

namespace oscr{

/**
 * Print joint names and their limits on the screen
 *
 * @param[in] jnames joint names obtained with RobotModel::jointNames()
 * @param[in] qmin minimum joint angular limits obtained with
 *                 RobotModel::jointMinAngularLimits()
 * @param[in] qmax maximum joint angular limits obtained with
 *                 RobotModel::jointMaxAngularLimits()
 * @param[in] dqmax joint velocity limits limits obtained with
 *                  RobotModel::jointVelocityLimits()
 */
void printJointLimits(const std::vector<std::string>& jnames,
                      const Eigen::VectorXd& qmin,
                      const Eigen::VectorXd& qmax,
                      const Eigen::VectorXd& dqmax);

/**
 * Print link names and their IDs
 *
 * @param[in] nid map of link names and IDs obtained with
 *                RobotModel::mapLinkNamesIDs()
 */
void printLinkID(const std::map< std::string, unsigned int >& nid);

/**
 * Print link names and their IDs within a line (short version)
 *
 * @param[in] nid map of link names and IDs obtained with
 *                RobotModel::mapLinkNamesIDs()
 */
void printLinkIDshort(const std::map< std::string, unsigned int >& nid);

/**
 * Print information about the parsed robot model
 *
 * @param[in] robot parsed robot model
 */
void printModelInfo(oscr::RobotModel* robot);

/**
 * Print a vector of strings on the screen
 *
 * @param[in] strvector
 */
void printStrVector(const std::vector<std::string>& strvector);

/**
 * Assign every joint name a number according to the position on the vector. If
 * there is a floating base, the numbers are increased by 7 to account for the
 * base pose.
 *
 * @param[in] jnames joint names
 * @param[in] has_floating_base indication of whether a floating base exists
 * @return map that relates joint names with an ID (in sequential order)
 */
std::map<std::string, unsigned int>
mapJointNames(const std::vector<std::string>& jnames,
              const bool& has_floating_base);

/**
 * Increment the position and orientatin with respect to the local frame
 *
 * @param[in] initial_pose initial pose
 * @param[in] dx, dy, dz increments in position
 * @param[in] angle angle of rotation (with respect to local frame) in degrees
 * @param[in] axisx, axisy, axisz axis of rotation
 * @return increased pose vector
 */
Eigen::VectorXd incPoseLocal(const Eigen::VectorXd& initial_pose,
                             const double& dx, const double& dy,
                             const double& dz, const double& angle,
                             const double& axisx, const double& axisy,
                             const double& axisz);

/**
 * Increment the position
 *
 * @param[in] initial_position initial position
 * @param[in] dx, dy, dz increments in position
 * @return increased position vector
 */
Eigen::VectorXd incPosition(const Eigen::VectorXd& initial_position,
                            const double& dx, const double& dy,
                            const double& dz);

};

#endif
