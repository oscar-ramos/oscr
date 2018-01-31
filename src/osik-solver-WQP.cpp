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

#include <oscr/ik/osik-solver-WQP.hpp>
#include <oscr/tools/math-utils.hpp>
#include <iostream>


oscr::OSIKSolverWQP::OSIKSolverWQP(RobotModel* rmodel,
                                   const Eigen::VectorXd& qinit,
                                   const double& dt)
  :
  OSIKSolver(rmodel, qinit, dt),
  lambda_q_(0.1)
{
  qpOptions_.setToMPC();
  qpOptions_.printLevel = qpOASES::PL_LOW;
}


void oscr::OSIKSolverWQP::setRegularizationTerm(const double& regterm)
{
  lambda_q_ = regterm;
}


double oscr::OSIKSolverWQP::getRegularizationTerm()
{
  return lambda_q_;
}


void oscr::OSIKSolverWQP::setJointLimits(const Eigen::VectorXd& qmin,
                                         const Eigen::VectorXd& qmax,
                                         const Eigen::VectorXd& dqmax)
{
  qmin_.resize(rmodel_->ndof());
  qmax_.resize(rmodel_->ndof());
  dqmax_.resize(rmodel_->ndof());

  unsigned int jinit=0;
  // If the robot has floating base, add high limit values for them
  if (rmodel_->hasFloatingBase())
  {
    jinit=7;
    for (unsigned int i=0; i<7; ++i)
    {
      qmin_[i] = -INFTY;
      qmax_[i] =  INFTY;
      dqmax_[i] = INFTY;
    }
  }
  // Joint limits for the actuated joints
  for (unsigned int i=jinit; i<qmin_.size(); ++i)
  {
    qmin_[i]  = qmin[i-jinit];
    qmax_[i]  = qmax[i-jinit];
    dqmax_[i] = dqmax[i-jinit];
  }
}


void oscr::OSIKSolverWQP::getPositionControl(const Eigen::VectorXd& q,
                                             Eigen::VectorXd& qcmd)
{
  // Fail if called without tasks
  if (taskStack_.size()==0){
    std::cerr << "Cannot get control without tasks ... "
              << "keeping previous configuration" << std::endl;
    qcmd = q;
    return;
  }

  // Fail if no joint limits set
  if ( (qmin_.size()==0) || (qmax_.size()==0) ){
    std::cerr << "Joint limits are not set! ..."
              << "keeping previous configuration" << std::endl;
    qcmd = q;
    return;
  }

  Eigen::VectorXd dq, de, p, eint;
  Eigen::MatrixXd J;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> W;
  double weight;

  // Adding regularization term
  W = lambda_q_*Eigen::MatrixXd::Identity(q.size(), q.size());
  p = Eigen::VectorXd::Zero(q.size());

  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    taskStack_[i]->updateJointConfig(q);
    taskStack_[i]->getTaskJacobian(J);
    taskStack_[i]->getDerivError(de);
    // std::cout << time_ << " - " << taskStack_[i]->getName() << " - de: "
    //           << de.transpose() << std::endl;
    //taskStack_[i]->getIntegralError(q, eint);
    weight = taskStack_[i]->getWeight();

    W = W + weight*J.transpose()*J;
    //p = p - 2*weight*J.transpose()*(de+eint);
    p = p - 2*weight*J.transpose()*de;
  }

  unsigned int nV = rmodel_->ndof();

  double *H = W.data();
  double *g = p.data();

  double *lb = new double[nV];
  double *ub = new double[nV];
  double ltmp, utmp;
  //for (unsigned int i=0; i<ndof_; ++i)
  for (unsigned int i=0; i<nV; ++i)
  {
    ltmp = (1.0/dt_)*(qmin_[i]-q[i]);
    utmp = (1.0/dt_)*(qmax_[i]-q[i]);
    lb[i] = ltmp > -dqmax_[i] ? ltmp : -dqmax_[i];
    ub[i] = utmp < dqmax_[i] ? utmp : dqmax_[i];
  }

  qpOASES::QProblemB qp(nV);
  qp.setOptions(qpOptions_);
  int nWSR = 10;  // Max number of working set recalculations
  if (qp.init(H, g, lb, ub, nWSR, 0))
  {
    std::cout << "OSIKSolverWQP: QP Problem ... returning previous values"
              << std::endl;
    qcmd = q;
    return;
  }
  double dqArray[nV];
  qp.getPrimalSolution(dqArray);

  //dq.resize(ndof_);
  dq.resize(nV);
  //for (unsigned int i=0; i<ndof_; ++i)
  for (unsigned int i=0; i<nV; ++i)
    dq[i] = dqArray[i];


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
