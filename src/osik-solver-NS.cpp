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

#include <oscr/ik/osik-solver-NS.hpp>
#include <oscr/tools/math-utils.hpp>


namespace oscr{


OSIKSolverNS::OSIKSolverNS(RobotModel* rmodel,
                           const Eigen::VectorXd& qinit,
                           const double& dt)
  :
  OSIKSolver(rmodel, qinit, dt)
{
}


void OSIKSolverNS::getPositionControl(const Eigen::VectorXd& q,
                                      Eigen::VectorXd& qcmd)
{
  // Fail if called without tasks
  if (taskStack_.size()==0)
  {
    std::cerr << "Cannot get control without tasks ... "
              << "keeping previous configuration" << std::endl;
    qcmd = q;
    return;
  }

  Eigen::MatrixXd J, pinvJ, P;
  Eigen::VectorXd dq, de;

  // Initialization of P and dq
  P = Eigen::MatrixXd::Identity(ndof_, ndof_);
  dq = Eigen::VectorXd::Zero(ndof_);

  Eigen::MatrixXd A1, A1p, A2, Pt;
  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    taskStack_[i]->updateJointConfig(q);
    taskStack_[i]->getTaskJacobian(J);
    // TODO: THIS IS JUST TO CHECK ... USE ASSERT INSTEAD
    if(J.cols() != dq.rows())
    {
      std::cerr << "Error of sizes" << std::endl;
      return;
    }
    taskStack_[i]->getDerivError(de);
    A1 = J*P;
    pinv(A1, A1p);
    A2 = de - J*dq;
    Pt = Eigen::MatrixXd::Identity(A1p.rows(), A1p.rows()) - A1p*A1;
    dq = dq + A1p*A2;
    P = P*Pt;
  }

  // For integration of floating base
  if (rmodel_->hasFloatingBase())
  {
    // Actuated joints
    qdes_.tail(rmodel_->ndofActuated()) =
      qdes_.tail(rmodel_->ndofActuated()) + dt_*dq.tail(rmodel_->ndofActuated());
    // Position of floating base
    qdes_.head(3) = qdes_.head(3) + dt_*dq.head(3);
    // Orientation of floating base
    Eigen::Vector3d omega;
    Eigen::MatrixXd T; T.resize(3,4);
    T <<
      -2.0*qdes_(4),  2.0*qdes_(3), -2.0*qdes_(6),  2.0*qdes_(5),
      -2.0*qdes_(5),  2.0*qdes_(6),  2.0*qdes_(3), -2.0*qdes_(4),
      -2.0*qdes_(6), -2.0*qdes_(5),  2.0*qdes_(4),  2.0*qdes_(3);
    omega = T*dq.segment(3,4);
    double angle = dt_*omega.norm()/2.0;
    Eigen::Vector3d k; k.setZero();
    if (omega.norm() > 1.0e-8)
      k = omega/omega.norm();
    else if (fabs(angle) > 1.0e-8)
      std::cout << "WARNING: axis for floating base orientation increment is zero"
                << std::endl;
    else {}// Axis is zero but angle increment is also zero ... it is okay!
    Eigen::Vector4d dQ, Qf;
    dQ << cos(angle), k(0)*sin(angle), k(1)*sin(angle), k(2)*sin(angle);
    Qf = quaternionMult(dQ, qdes_.segment(3,4));
    qdes_(3) = Qf(0);
    qdes_(4) = Qf(1);
    qdes_(5) = Qf(2);
    qdes_(6) = Qf(3);
  }
  else
  {
    //qcmd = qdes_ + dt_*dq;
    qdes_ = qdes_ + dt_*dq;
  }

  //qdes_ = qcmd;
  qcmd = qdes_;

  // Increase its internal time
  time_ ++;
}

}

