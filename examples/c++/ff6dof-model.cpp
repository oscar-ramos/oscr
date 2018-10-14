/*
 * Test the model parsing and some basic kinematic properties of a simple
 * revolute robot with a free floating base and 6 dof (2 chains of 3 dof each)
 *
 * To execute:
 *      ./ff6dofModel backend - where backend can be pin (by default) or rbdl 
 * 
 */

#include <oscr/model/robot-model-pin.hpp>
#include <oscr/model/robot-model-rbdl.hpp>
#include <oscr/tools/model-utils.hpp>


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
    
  unsigned int ndof, ndof_actuated;
  bool has_floating_base;
  std::string floating_link;
  Eigen::VectorXd qmin, qmax, dqmax;
  std::vector<std::string> jnames;
  std::map<std::string, unsigned int> linkID;

  // Number of Degrees of Freedom
  ndof  = robot->ndof();
  // Number of Actuated Degrees of Freedom
  ndof_actuated = robot->ndofActuated();
  // Floating link
  has_floating_base = robot->hasFloatingBase();
  floating_link = robot->floatingLink();
  // Joint angular and velocity limits
  qmin  = robot->jointMinAngularLimits();
  qmax  = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();
  // Jont names
  jnames = robot->jointNames();
  // Link names and IDs
  linkID = robot->mapLinkNamesIDs();

  // Show the previous information on the screen
  std::cout << "Total ndof: " << ndof << std::endl;
  std::cout << "Total ndof actuated: " << ndof_actuated << std::endl;
  std::cout << "Has floating base: " << has_floating_base << std::endl;
  std::cout << "Floating link: " << floating_link << std::endl;
  oscr::printJointLimits(jnames, qmin, qmax, dqmax);
  oscr::printLinkIDshort(linkID);

  // Joint configuration with all values to zero.
  Eigen::VectorXd q(ndof), qcurrent(ndof); q.setZero();
  // If there is a floating base, then set a valid null quaternion
  if (has_floating_base)
    q[3]=1.0;
  // Set the current joint configuration
  robot->updateJointConfig(q);
  
  // Get the current joint configuration
  qcurrent = robot->getJointConfig();

  // Get the pose of larm3 and lleg3 and the Jacobian of larm3
  Eigen::VectorXd poselarm, poselleg;
  Eigen::MatrixXd Jlarm;
  poselarm = robot->linkPose(linkID["larm3"]);
  poselleg = robot->linkPose(linkID["lleg3"]);
  Jlarm = robot->geometricJacobian(linkID["larm3"]);
    
  std::cout << "\nCurrent joint config: " << qcurrent.transpose() << std::endl;
  std::cout << "Pose of larm3: " << poselarm.transpose() << std::endl;
  std::cout << "Pose of lleg3: " << poselleg.transpose() << std::endl;
  std::cout << "Geometric Jacobian of larm3:\n " << Jlarm << std::endl;

  return 0;
}
