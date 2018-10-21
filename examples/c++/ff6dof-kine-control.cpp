/*
 * Test the control of the robot
 *
 * To execute:
 *      ./ff6dofKineControl backend - where backend can be pin (by default) or rbdl 
 * 
 */

#include <oscr/model/robot-model-pin.hpp>
#include <oscr/model/robot-model-rbdl.hpp>
#include <oscr/ik/kine-task-pose.hpp>
#include <oscr/ik/osik-solvers.hpp>
#include <oscr/tools/vector-logger.hpp>


int main(int argc, char *argv[])
{
  // Use a floating base?
  bool fbase = true;
  // Robot model
  std::string modelname = "../models/ff6dof.urdf";
  // Dummy initialization
  oscr::RobotModel *robot = new oscr::RobotModelPin();
  
  if (argc<2)
  {
    std::cout << "-- Using Pinocchio as backend by default" << std::endl;
    robot = new oscr::RobotModelPin(modelname, fbase);
  }
  else
  {
    if (!strcmp(argv[1],"pin"))
    {
      std::cout << "-- Using Pinocchio as backend" << std::endl;
      robot = new oscr::RobotModelPin(modelname, fbase);
    }
    else if (!strcmp(argv[1], "rbdl"))
    {
      std::cout << "-- Using RBDL as backend" << std::endl;
      robot = new oscr::RobotModelRbdl(modelname, fbase);
    }
    else
    {
      std::cout << "Only pin and rbdl are accepted as backends ..."
                << "ending the program ..." << std::endl;
      return -1;
    }
  }
    
  std::map<std::string, unsigned int> linkID;
  linkID = robot->mapLinkNamesIDs();

  // Joint configuration with all values to zero.
  Eigen::VectorXd q(robot->ndof()); q.setZero();
  // If there is a floating base, then set a valid null quaternion
  if (robot->hasFloatingBase())
    q[3]=1.0;
  // Set the current joint configuration
  robot->updateJointConfig(q);
  // Get the current joint configuration
  q = robot->getJointConfig();

  // Definition of a task for the arm (keep) and a task for the leg
  oscr::KineTask *taskarm = new oscr::KineTaskPose(robot, linkID["larm3"], "position");
  oscr::KineTask *taskleg = new oscr::KineTaskPose(robot, linkID["lleg3"], "position");
  taskarm->setGain(1.0);
  taskleg->setGain(1.0);

  // Desired value for the arm (keep it)
  Eigen::VectorXd xarm;
  taskarm->getSensedValue(xarm);
  taskarm->setDesiredValue(xarm);
  // Desired value for the leg (move it)
  Eigen::VectorXd xleg;
  taskleg->getSensedValue(xleg);
  xleg(0) = xleg(0) + 0.1;
  xleg(1) = xleg(1) + 0.1;
  xleg(2) = xleg(2) + 0.1;
  taskleg->setDesiredValue(xleg);

  // Solver
  double dt = 0.010;
  oscr::OSIKSolverNS solver(robot, q, dt);

  // Add tasks to the solver
  solver.pushTask(taskarm);
  solver.pushTask(taskleg);

  Eigen::VectorXd qdes;
  Eigen::VectorXd error;
  taskleg->getDerivError(error);
  error = taskleg->getError();

  // Logs
  oscr::VectorLogger arm_log("/tmp/arm.txt");
  oscr::VectorLogger leg_log("/tmp/leg.txt");
  oscr::VectorLogger legerror_log("/tmp/legerror.txt");
  oscr::VectorLogger q_log("/tmp/q.txt");
  
  unsigned int i=0;
  double time = 0.0;
  while (error.norm()>0.0005)
  {
    solver.getPositionControl(q, qdes);
    robot->updateJointConfig(q);
    q = qdes;
    error = taskleg->getError();

    // Logs
    time = i*dt;
    legerror_log.save(error, time);
    q_log.save(q, time);
    taskarm->getSensedValue(xarm);
    taskleg->getSensedValue(xleg);
    arm_log.save(xarm, time);
    leg_log.save(xleg, time);

    // Print on screen
    if (++i%50==0)
      std::cout << i << " - error norm: " << error.norm() << std::endl;
  }
  
  return 0;
}
