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

#include <oscr/ik/osik-solver-HQP.hpp>
#include <oscr/tools/math-utils.hpp>
#include <iostream>


oscr::OSIKSolverHQP::OSIKSolverHQP(RobotModel* rmodel,
                                   const Eigen::VectorXd& qinit,
                                   const double& dt)
  :
  OSIKSolver(rmodel, qinit, dt)
  ,has_joint_limits_(false)
{
  qpOptions_.setToMPC();
  qpOptions_.printLevel = qpOASES::PL_LOW;
}


void oscr::OSIKSolverHQP::setJointLimits(const Eigen::VectorXd& qmin,
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
  has_joint_limits_ = true;
}


bool oscr::OSIKSolverHQP::hasJointLimits()
{
  return has_joint_limits_;
}


void oscr::OSIKSolverHQP::getPositionControl(const Eigen::VectorXd& q,
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

  unsigned int m, nV, nC;
  unsigned int n = rmodel_->ndof();  // N dofs
  unsigned int mprev = 0;            // Size of tasks with higher priority
  double *dqArray;

  Eigen::MatrixXd Jprev, J;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Hm;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> Am;
  Eigen::VectorXd dq, wopt, bminprev, bmaxprev, demin, demax;
  Eigen::VectorXd lbm, ubm, gm, bminm, bmaxm;
  Jprev.resize(0,n);
  bminprev.resize(0);
  bmaxprev.resize(0);

  for (unsigned int i=0; i<taskStack_.size(); ++i)
  {
    taskStack_[i]->updateJointConfig(q);
    taskStack_[i]->getTaskJacobian(J);
    taskStack_[i]->getTaskBounds(demin, demax);
    // std::cout << "J: " << J << std::endl;
    // std::cout << "demin: " << demin.transpose() << ", demax: " << demax.transpose() << std::endl;

    m  = demin.size();         // Size of current task
    nV = m + n;                // N variables (w & dq)
    nC = m + mprev;            // N constraints (w & w_prev)
    qpOASES::QProblem qp(nV, nC);
    qp.setOptions(qpOptions_);
    int nWSR = 10;   // Max number of working set recalculations

    // Cost function:
    //       [w dq]*H*[w dq]' + [w dq]*g
    Hm.setZero(nV, nV);
    Hm.topLeftCorner(m, m) = Eigen::MatrixXd::Identity(m, m);
    gm.setZero(nV);
    double *g = gm.data();
    double *H = Hm.data();

    // Constraints:
    //     bmin <= A*[w dq] <= bmax
    //       lb <=  [w dq]  <= ub
    Am.setZero(nC, nV);
    Am.topLeftCorner(m, m) = Eigen::MatrixXd::Identity(m, m);
    Am.topRightCorner(m, n) = J;
    Am.bottomRightCorner(mprev, n) = Jprev;

    bminm.resize(m+mprev);
    bminm.head(m) = demin;
    bminm.tail(mprev) = bminprev;
    bmaxm.resize(m+mprev);
    bmaxm.head(m) = demax;
    bmaxm.tail(mprev) = bmaxprev;

    double *A = Am.data();
    double *bmin = bminm.data();
    double *bmax = bmaxm.data();

    // Lower and upper joint limits set to infinity
    lbm.setConstant(nV, -INFTY);
    ubm.setConstant(nV, INFTY);
    double *lb = lbm.data();
    double *ub = ubm.data();
    // Use joint limits when they are set
    if (has_joint_limits_)
    {
      double ltmp, utmp;
      for (unsigned int i=0; i<ndof_; ++i)
      {
        ltmp = (1.0/dt_)*(qmin_[i]-q[i]);
        utmp = (1.0/dt_)*(qmax_[i]-q[i]);
        lb[i+m] = ltmp > -dqmax_[i] ? ltmp : -dqmax_[i];
        ub[i+m] = utmp < dqmax_[i] ? utmp : dqmax_[i];
      }
    }

    if (qp.init(H, g, A, lb, ub, bmin, bmax, nWSR))
    {
      std::cout << "OSIKSolverHQP: QP Problem ... returning previous values"
                << std::endl;
      qcmd = q;
      return;
    }
    dqArray = new double[nV];
    qp.getPrimalSolution(dqArray);

    wopt.resize(m);
    for (unsigned int i=0; i<m; ++i)
      wopt[i] = dqArray[i];

    // Update task variables with higher priority for next loop
    Jprev.conservativeResize(mprev+m, n);
    Jprev.bottomRows(m) = J;
    bminprev.conservativeResize(mprev+m);
    bmaxprev.conservativeResize(mprev+m);
    bminprev.tail(m) = demin-wopt;
    bmaxprev.tail(m) = demax-wopt;
    mprev = mprev + m;

  }

  dq.resize(n);
  for (unsigned int i=0; i<n; ++i)
    dq[i] = dqArray[i+m];

  // std::cout << "Time " << time_ << " - dq: " << dq.transpose() << std::endl;
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
    // std::cout << "T: " << T << std::endl;
    // std::cout << "omega norm: " << omega.norm() << std::endl;

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
