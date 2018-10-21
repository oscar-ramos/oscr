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

#include <oscr/ik/osik-solver.hpp>
#include <iostream>


oscr::OSIKSolver::OSIKSolver(RobotModel* model,
                             const Eigen::VectorXd& qinit,
                             const double& dt)
  :
  rmodel_(model),
  dt_(dt),
  qdes_(qinit),
  time_(0)
{
  taskStack_.clear();
  ndof_ = model->ndof();
}


oscr::OSIKSolver::~OSIKSolver()
{
  //delete rmodel_;
}


void oscr::OSIKSolver::getTaskStack(std::vector< KineTask* >& stack_tasks)
{
  stack_tasks = taskStack_;
}


void oscr::OSIKSolver::pushTask(KineTask* task)
{
  taskStack_.push_back(task);
}


void oscr::OSIKSolver::popTask()
{
  taskStack_.pop_back();
}


void oscr::OSIKSolver::removeTask(const std::string& taskName)
{
  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    if (taskName.compare(taskStack_[i]->getName())==0)
    {
      taskStack_.erase(taskStack_.begin()+i);
      break;
    }
  }
}


unsigned int oscr::OSIKSolver::getTime()
{
  return time_;
}


void oscr::OSIKSolver::printStack()
{
  std::cout << "Stack at time " << time_ << ":" << std::endl;
  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    std::cout << "- Priority " << i+1 << ": " << taskStack_[i]->getName()
              << std::endl;
  }
}
