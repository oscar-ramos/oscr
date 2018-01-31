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

#ifndef KINEMATIC_TASK_HPP
#define KINEMATIC_TASK_HPP

#include <Eigen/Dense>
#include <string>
#include <oscr/model/robot-model.hpp>


namespace oscr{

/**
 * Base class for a generic kinematic task. All the inherited kinematic tasks
 * can be used with a differential inverse kinematics solver.
 *
 */
class KineTask
{
public:

  /**
   * Constructor.
   *
   * @param[in] rmodel Robot model
   * @param[in] taskName Task Name
   * @param[in] gain Task gain
   */
  KineTask(RobotModel* rmodel,
           const std::string& taskName = "KineTask",
           const double& gain = 1.0);

  /**
   * Destructor.
   */
  ~KineTask();

  /**
   * Set the kinematic task name
   *
   * @param[in] taskName task name
   */
  void setName(const std::string& taskName);

  /**
   * Get the kinematic task name
   *
   * @return task name
   */
  std::string getName() const;

  /**
   * Set a kinematic constant task gain. It unables the adaptive gain.
   *
   * @param[in] gain task gain
   */
  void setGain(const double& gain);

  /**
   * @brief Set an adaptive kinematic gain.
   * The adaptive gain \f$\lambda\f$ is given by the decreasing exponential
   * \f$\lambda=(\lambda_{max}-\lambda_{min})e^{-k(\Vert x-x^*
   * \Vert)}+\lambda_{min}\f$ and depends upon the task error
   * \f${x-x^*}\f$. Note: it unables the constant gain.
   *
   * @param[in] max_gain maximum gain for a small error (\f$\lambda_{max}>0\f$)
   * @param[in] min_gain miminum gain for a big error (\f$\lambda_{min}>0\f$)
   * @param[in] rate_descent descent for the exponential (\f$ k>0 \f$)
   */
  void setAdaptiveGain(const double& max_gain,
                       const double& min_gain,
                       const double& rate_descent);

  /**
   * Get the kinematic task gain. Internally, it updates the gain, when the
   * gain is adaptive.
   *
   * @param[in] error scalar value of the error: \f$\Vert x-x^* \Vert\f$. Only
   *                  needed when the gain is adaptive (ignored for a constant
   *                  gain).
   * @return task gain
   */
  double getGain(const double& error=0.0);

  /**
   * Set the desired value for the kinematic task
   *
   * @param[in] desiredValue task desired value: \f$x^*\f$
   */
  void setDesiredValue(const Eigen::VectorXd& desiredValue);
  // void setDesiredValue(const Eigen::Ref<const Eigen::VectorXd>& desiredValue);

  /**
   * Get the desired value for the kinematic task
   *
   * @param[out] desiredValue task desired value \f$x^*\f$
   */
  Eigen::VectorXd getDesiredValue() const;
  //void getDesiredValue(Eigen::VectorXd& desiredValue) const;

  /**
   * Set the task weight (only used with KinematicSolverWQP)
   *
   * @param[in] weight Task weight
   */
  void setWeight(const double& weight);

  /**
   * Get the task weight (only used with KinematicSolverWQP)
   *
   * @return task weight
   */
  double getWeight() const;

  /**
   * Get the dimension of the task (this is inferred from the size of the
   * sensed value)
   *
   * @return task dimension
   */
  unsigned int getTaskDim() const;


  /**
   * Get the task error \f$e\f$. Note that the task error is only updated after
   * a call to getDerivError()
   *
   * @return task error
   */
  Eigen::VectorXd getError() const;


  /**
   * Update the joint configuration.
   *
   * @param[in] q generalized joint configuration (it must include the floating
   *              base configuration, if the robot has one)
   */
  void updateJointConfig(const Eigen::VectorXd& q);


  /**
   * Keep the task with the current values. It will set the current value as
   * the desired value.
   *
   * @param gain task gain (typically high to ensure keeping the configuration)
   */
  void keep(const double& gain);

  /**
   * Get the bounds for kinematic tasks. For equality tasks, both minimum and
   * maximum bounds correspond to the same desired derivative of the
   * error. This function must be reimplemented for inequality tasks.
   *
   * @param bound_min minimum bound
   * @param bound_max maximum bound
   */
  void getTaskBounds(Eigen::VectorXd& bound_min,
                     Eigen::VectorXd& bound_max);

  // ~~~~~~~~~~~~~~~~~~~~~~
  // PURE VIRTUAL FUNCTIONS
  // ~~~~~~~~~~~~~~~~~~~~~~

  /**
   * Get the sensed value for the kinematic task given a joint configuration
   *
   * @param[out] xsensed Sensed value for the joint configuration
   */
  virtual void getSensedValue(Eigen::VectorXd& xsensed) const = 0;

  /**
   * Get the task Jacobian for a given configuration
   *
   * @param[out] Jacobian Jacobian for the joint configuration
   */
  virtual void getTaskJacobian(Eigen::MatrixXd& Jacobian) const = 0;

  /**
   * Get the derivative of the task error \f$(\dot e)\f$
   *
   * @param[out] de derivative of the error
   */
  virtual void getDerivError(Eigen::VectorXd& de) = 0;

  // /**
  //  * Get the integral of the task error \f$(-k_i \int e dt)\f$. This term can
  //  * be added to \f$\dot e^*\f$ (check if useful!).
  //  * @param[in] q Joint configuration
  //  * @param[out] eint integral of the error multiplied by a constant gain
  //  */
  // virtual void getIntegralError(const Eigen::VectorXd& q,
  //                               Eigen::VectorXd& eint) = 0;


protected:

  /// Robot model
  RobotModel* rmodel_;
  /// Kinematic task name
  std::string name_;
  /// Kinematic task gain
  double gain_;
  /// Total number of degrees of freedom. It includes the floating base, if
  /// present.
  unsigned int ndof_;
  /// Desired value for the kinematic task
  Eigen::VectorXd xdes_;
  /// Weight for the task (only used with weighted QP)
  double weight_;
  /// Integral (sum of) error
  Eigen::VectorXd esum_;
  /// Task error
  Eigen::VectorXd error_;

  // For the adaptive gain:
  /// Flag that sets the adaptive gain
  bool adaptiveGain_;
  /// Maximum Gain
  double lmax_;
  /// Minimum Gain
  double lmin_;
  /// Rate of descent
  double k_;

  // For internal joint configuration
  /// Joint configuration
  Eigen::VectorXd q_;

};

}

#endif
