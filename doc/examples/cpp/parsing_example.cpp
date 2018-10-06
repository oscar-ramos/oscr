#include <oscr/model/robot-model-pin.hpp>     // Using pinocchio
// #include <oscr/model/robot-model-pin.hpp>  // Using rbdl
#include <oscr/tools/model-utils.hpp>


int main(int argc, char *argv[])
{
  // Whether floating base is used or not
  bool fbase = true;
  std::string modelname = "mymodel.urdf";

  oscr::RobotModel *robot = new oscr::RobotModelPin(modelname, fbase);
  // oscr::RobotModel *robot = new oscr::RobotModelRbdl(modelname, fbase);

  unsigned int ndof, ndof_actuated;
  bool has_floating_base;
  std::string floating_link;
  Eigen::VectorXd qmin, qmax, dqmax;
  std::vector<std::string> jnames;
  std::map<std::string, unsigned int> maplink_name_ID;

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
  maplink_name_ID = robot->mapLinkNamesIDs();

  // Show the previous information on the screen
  std::cout << "Total ndof: " << ndof << std::endl;
  std::cout << "Total ndof actuated: " << ndof_actuated << std::endl;
  std::cout << "Has floating base: " << has_floating_base << std::endl;
  std::cout << "Floating link: " << floating_link << std::endl;
  oscr::printJointLimits(jnames, qmin, qmax, dqmax);
  oscr::printLinkID(maplink_name_ID);

  return 0;
}
