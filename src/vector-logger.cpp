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

#include <oscr/tools/vector-logger.hpp>


namespace oscr{

VectorLogger::VectorLogger(const std::string& fileName)
{
  ffile_.open(fileName.c_str());  
}


VectorLogger::~VectorLogger()
{
  ffile_.close();
}


void VectorLogger::save(const Eigen::VectorXd& vector,
                        const double& time)
{
  ffile_ << time << " ";
  for (unsigned int i=0; i<vector.size(); ++i)
  {
    ffile_ << vector(i) << " ";
  }
  ffile_ << "\n";
}

}
