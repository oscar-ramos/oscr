/*
 * Copyright 2018, Oscar Ramos
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

#ifndef OSCR_VECTOR_LOGGER_HPP
#define OSCR_VECTOR_LOGGER_HPP

#include <fstream>
#include <Eigen/Dense>

namespace oscr{

/**
 * Class to log Eigen vectors
 *
 */
class VectorLogger
{
public:

  /**
   * Constructor.
   *
   * @param[in] fileName Name for the log file
   */
  VectorLogger(const std::string& fileName = "my_log.txt");

  /**
   * Destructor.
   */
  ~VectorLogger();

  /**
   * Save the vector Get the task weight (only used with KinematicSolverWQP)
   *
   * @param[in] vector Eigen vector containing the data
   * @param[in] time log time
   */
  void save(const Eigen::VectorXd& vector,
            const double& time = 0.0);
  
private:
  /// Object containing the log
  std::ofstream ffile_;

};

}
#endif
