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

#include <fstream>
#include <stack>
#include <boost/shared_ptr.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <oscr/tools/math-utils.hpp>
#include <oscr/model/robot-model-rbdl.hpp>

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl_utils.h>


namespace oscr{


RobotModelRbdl::RobotModelRbdl()
  :
  RobotModel(),
  rmodel_(new RigidBodyDynamics::Model())
{
}


RobotModelRbdl::RobotModelRbdl(const std::string& model_name,
                               const bool& has_floating_base)
  :
  RobotModel(has_floating_base),
  rmodel_(new RigidBodyDynamics::Model())
{
  loadURDF(model_name, has_floating_base);
}


RobotModelRbdl::~RobotModelRbdl()
{
  delete rmodel_;
}


bool RobotModelRbdl::loadURDF(const std::string& model_name,
                              const bool& has_floating_base,
                              const bool& verbose)
{
  has_floating_base_ = has_floating_base;

  if (!RigidBodyDynamics::Addons::URDFReadFromFile(model_name.c_str(),
                                                   rmodel_,
                                                   has_floating_base_,
                                                   verbose))
  {
    std::cerr << "Error opening file: " << model_name.c_str() << std::endl;
    return false;
  }
  else
  {
    std::cout << "Model loaded: " << model_name << std::endl;
  }

  /*
   * Get joint names and joint limits from urdf (rbdl does not provide them!)
   */
  jnames_.clear();
  std::vector<double> qmin;
  std::vector<double> qmax;
  std::vector<double> dqmax;

  typedef boost::shared_ptr<urdf::Link> LinkPtr;
  typedef boost::shared_ptr<urdf::Joint> JointPtr;

  // reserve memory for the contents of the file
  std::ifstream model_file (model_name.c_str());
  std::string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  boost::shared_ptr<urdf::ModelInterface>
    urdf_model = urdf::parseURDF(model_xml_string.c_str());

  std::map<std::string, LinkPtr > link_map;
  link_map = urdf_model->links_;

  std::stack<LinkPtr > link_stack;
  std::stack<int> joint_index_stack;

  // Add the bodies in a depth-first order of the model tree
  link_stack.push (link_map[(urdf_model->getRoot()->name)]);

  if (link_stack.top()->child_joints.size() > 0) {
    joint_index_stack.push(0);
  }

  while (link_stack.size() > 0)
  {
    LinkPtr cur_link = link_stack.top();
    unsigned int joint_idx = joint_index_stack.top();

    if (joint_idx < cur_link->child_joints.size())
    {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // Increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx+1);

      link_stack.push (link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      int i = 0;
      if (cur_joint->type == urdf::Joint::REVOLUTE ||
          cur_joint->type == urdf::Joint::CONTINUOUS)
      {
        // std::cout << "joint: " << cur_joint->name << std::endl;
        jnames_.push_back(cur_joint->name);
        // For revolute joints
        if (cur_joint->type == 1)
        {
          qmin.push_back(cur_joint->limits->lower);
          qmax.push_back(cur_joint->limits->upper);
          dqmax.push_back(cur_joint->limits->velocity);
        }
        // *************************************************
        // For continuous joints, set arbitrarily big values
        // *************************************************
        else {
          qmin.push_back(-20.0);
          qmax.push_back(20.0);
          dqmax.push_back(100.0);
        }
        i++;
      }
      else if (cur_joint->type == urdf::Joint::FLOATING)
      {
        std::cout << "Found floating joint in URDF model (joint " << i++
                  << ")" << "... \n  -> Not supported. Unknown results!";
        std::cout << "Please change the \"floating\" joint to \"fixed\" in "
          "the URDF MOdel" << std::endl;
        return false;
      }
      else if (cur_joint->type == urdf::Joint::PRISMATIC)
      {
        std::cout << "Not supported joint: PRISMATIC" << std::endl;
        return false;
      }
    }
    else
    {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  qmin_.resize(qmin.size());
  qmax_.resize(qmax.size());
  dqmax_.resize(dqmax.size());
  for(unsigned int i; i< qmin.size(); ++i)
  {
    qmin_(i) = qmin[i];
    qmax_(i) = qmax[i];
    dqmax_(i) = dqmax[i];
  }

  // Get the map for links and their IDs
  std::vector<std::string> link_names;
  std::vector<unsigned int> link_ids;
  linkNames(link_names, link_ids);
  for (unsigned int i=0; i<link_names.size(); ++i)
    link_id_[link_names[i]] = link_ids[i];

  return true;
}


unsigned int RobotModelRbdl::ndof()
  const
{
  if (has_floating_base_)
    return 7 + rmodel_->dof_count;
  else
    return rmodel_->dof_count;
}


unsigned int RobotModelRbdl::ndofActuated()
  const
{
  return rmodel_->dof_count;
}


std::string RobotModelRbdl::floatingLink()
  const
{
  return RobotModel::floatingLink(0);
}


void RobotModelRbdl::linkNames(std::vector<std::string>& link_names,
                               std::vector<unsigned int>& link_ids)
  const
{
  link_names.clear();
  link_ids.clear();

  std::map< std::string, unsigned int > bodymap = rmodel_->mBodyNameMap;
  for (std::map< std::string, unsigned int >::iterator it=bodymap.begin();
       it!=bodymap.end(); ++it)
  {
    if (it->second > 1000)
      continue;
    link_names.push_back(it->first);
    link_ids.push_back(it->second);
 }
}


void RobotModelRbdl::updateJointConfig(const Eigen::VectorXd& q)
{
  q_ = q;
}


Eigen::Vector3d RobotModelRbdl::linkPosition(const unsigned int& link_number,
                                             const Eigen::Vector3d& local_pos)
  const
{
  Eigen::Vector3d lpos;
  if (has_floating_base_)
  {
    lpos = CalcBodyToBaseCoordinates(*rmodel_, q_.tail(this->ndofActuated()),
                                     link_number, local_pos, true);
    // Transform position to the world frame (using the base transformation)
    Eigen::Matrix3d Rbase;
    Rbase = quaternionToRotation(q_.segment(3,4));
    lpos = Rbase*lpos + q_.head(3);
  }
  else
    lpos = CalcBodyToBaseCoordinates(*rmodel_, q_, link_number,
                                     local_pos, true);
  return lpos;
}


Eigen::Vector3d RobotModelRbdl::linkPosition(const std::string& link_name,
                                             const Eigen::Vector3d& local_pos)
const
{
  try{
    return linkPosition(link_id_.at(link_name), local_pos);
  }
  catch (std::exception& e)
  {
    std::cerr << "WARNING: Link name " << link_name << " is invalid! ... "
              <<  "Returning zeros" << std::endl;
    return Eigen::Vector3d::Zero();
  }
}


Eigen::Vector4d RobotModelRbdl::linkOrientation(const unsigned int& link_number)
  const
{
  //Eigen::Vector4d quat;
  Eigen::Matrix3d Rlink;
  if(has_floating_base_)
  {
    Rlink = CalcBodyWorldOrientation(*rmodel_, q_.tail(this->ndofActuated()),
                                     link_number, true).transpose();
    Eigen::Matrix3d Rbase;
    Rbase = quaternionToRotation(q_.segment(3,4));
    Rlink = Rbase*Rlink;
  }
  else
    Rlink = CalcBodyWorldOrientation(*rmodel_, q_,
                                     link_number, true).transpose();

  return rotationToQuaternion(Rlink);
}


Eigen::Vector4d RobotModelRbdl::linkOrientation(const std::string& link_name)
const
{
  try{
    return linkOrientation(link_id_.at(link_name));
  }
  catch (std::exception& e)
  {
    std::cerr << "WARNING: Link name " << link_name << " is invalid! ... "
              <<  "Returning zeros" << std::endl;
    return Eigen::Vector4d::Zero();
  }
}


Eigen::VectorXd RobotModelRbdl::linkPose(const unsigned int& link_number,
                                         const Eigen::Vector3d& local_pos)
  const
{
  Eigen::VectorXd lpose;
  Eigen::Matrix3d Rlink;
  lpose.resize(7);

  if(has_floating_base_)
  {
    lpose.head(3) = CalcBodyToBaseCoordinates(*rmodel_,
                                              q_.tail(ndofActuated()),
                                              link_number, local_pos, true);
    Rlink = CalcBodyWorldOrientation(*rmodel_, q_.tail(ndofActuated()),
                                     link_number, true).transpose();
    // Transform to the world frame (using the floating base transformation)
    Eigen::Matrix3d Rbase;
    Rbase = quaternionToRotation(q_.segment(3,4));
    lpose.head(3) = Rbase*lpose.head(3) + q_.head(3);
    Rlink = Rbase*Rlink;
  }
  else
  {
    lpose.head(3) = CalcBodyToBaseCoordinates(*rmodel_, q_, link_number,
                                              local_pos, true);
    Rlink = CalcBodyWorldOrientation(*rmodel_, q_,
                                     link_number, true).transpose();
  }
  lpose.tail(4) = rotationToQuaternion(Rlink);

  return lpose;
}


Eigen::VectorXd RobotModelRbdl::linkPose(const std::string& link_name,
                                         const Eigen::Vector3d& local_pos)
const
{
  try{
    return linkPose(link_id_.at(link_name), local_pos);
  }
  catch (std::exception& e)
  {
    std::cerr << "WARNING: Link name " << link_name << " is invalid! ... "
              <<  "Returning zeros" << std::endl;
    return Eigen::VectorXd::Zero(7,1);
  }
}


Eigen::MatrixXd RobotModelRbdl::linearJacobian(const unsigned int& link_number,
                                               const Eigen::Vector3d& local_pos)
  const
{
  Eigen::MatrixXd J;

  if (has_floating_base_)
  {
    Eigen::Vector3d xlink;
    Eigen::Matrix3d Rbase, Rlink;
    Eigen::MatrixXd Jxyz, T;

    Rbase = quaternionToRotation(q_.segment(3,4));
    xlink = CalcBodyToBaseCoordinates(*rmodel_, q_.tail(this->ndofActuated()),
                                      link_number, local_pos, true);
    xlink = Rbase*xlink + q_.head(3);

    T.resize(3,4);
    T <<
      -2.0*q_(4),  2.0*q_(3), -2.0*q_(6),  2.0*q_(5),
      -2.0*q_(5),  2.0*q_(6),  2.0*q_(3), -2.0*q_(4),
      -2.0*q_(6), -2.0*q_(5),  2.0*q_(4),  2.0*q_(3);

    Jxyz.resize(6, this->ndofActuated()); Jxyz.setZero();
    // This is in the local link frame (Note that the last argument must be
    // false [because previously there was true for xlink] ... why???)
    CalcBodySpatialJacobian(*rmodel_, q_, link_number, Jxyz, false);
    // Rotation: link frame wrt world (inertial) frame
    Rlink = quaternionToRotation(linkOrientation(link_number));

    J.resize(3, this->ndof());
    J.setZero(); // Do not delete (important to avoid spurious  values...why?)
    J.leftCols(3) = Eigen::Matrix3d::Identity();
    J.middleCols(3,4) = skew(q_.head(3)-xlink)*T;
    J.rightCols(this->ndofActuated()) = Rlink*Jxyz.bottomRows(3);

    return J;
  }

  else
  {
    J.resize(6, this->ndof());
    J.setZero();      // VERY IMPORTANT FOR THE JACOBIAN IN RBDL!!!
    // This is in the local link frame
    CalcBodySpatialJacobian(*rmodel_, q_, link_number, J, true);
    // Orientation from local link frame to base frame
    Eigen::Matrix3d R;
    // Rotation: link frame wrt base frame
    R = quaternionToRotation(linkOrientation(link_number));
    // Convert Jacobian from local link frame to (fixed) base frame
    return R*J.bottomRows(3);
  }
}


Eigen::MatrixXd RobotModelRbdl::angularJacobian(const unsigned int& link_number)
  const
{
  Eigen::MatrixXd J;

  if (has_floating_base_)
  {
    Eigen::Matrix3d Rlink;
    Eigen::MatrixXd T, Jtmp; 

    T.resize(3,4);
    T <<
      -2.0*q_(4),  2.0*q_(3), -2.0*q_(6),  2.0*q_(5),  
      -2.0*q_(5),  2.0*q_(6),  2.0*q_(3), -2.0*q_(4), 
      -2.0*q_(6), -2.0*q_(5),  2.0*q_(4),  2.0*q_(3);

    Jtmp.resize(6, this->ndofActuated()); Jtmp.setZero();
    // This is in the local link frame (TODO: check if in the case of angular
    // Jacobian the 'false' is OK or should be 'true'
    CalcBodySpatialJacobian(*rmodel_, q_, link_number, Jtmp, false);
    // Rotation: link frame wrt world (inertial) frame
    Rlink = quaternionToRotation(linkOrientation(link_number));

    J.resize(3, this->ndof()); 
    J.setZero(); // Do not delete (important to avoid spurious  values...why?)
    J.middleCols(3,4) = T;
    J.rightCols(this->ndofActuated()) = Rlink*Jtmp.topRows(3);

    return J;
  }
  else
  {
    J.resize(6, this->ndof());
    J.setZero();
    // This is in the local link frame
    CalcBodySpatialJacobian(*rmodel_, q_, link_number, J, true);
    // Orientation from local link frame to base frame
    // This is OK for fixed base ... but check for the case where there is a
    // floating base
    Eigen::Matrix3d R;
    R = quaternionToRotation(linkOrientation(link_number));
    // Convert to fixed frame
    return R*J.topRows(3);
  }
}


Eigen::MatrixXd RobotModelRbdl::geometricJacobian(const unsigned int& link_number,
                                                  const Eigen::Vector3d& local_pos)
  const
{
  Eigen::MatrixXd J, Jtmp;

  if (has_floating_base_)
  {
    Eigen::Vector3d xlink;
    Eigen::Matrix3d Rbase, Rlink;
    Eigen::MatrixXd T; 

    Rbase = quaternionToRotation(q_.segment(3,4));
    xlink = CalcBodyToBaseCoordinates(*rmodel_, q_.tail(this->ndofActuated()),
                                      link_number, local_pos, true);
    xlink = Rbase*xlink + q_.head(3);

    T.resize(3,4);
    T <<
      -2.0*q_(4),  2.0*q_(3), -2.0*q_(6),  2.0*q_(5),  
      -2.0*q_(5),  2.0*q_(6),  2.0*q_(3), -2.0*q_(4), 
      -2.0*q_(6), -2.0*q_(5),  2.0*q_(4),  2.0*q_(3);

    Jtmp.resize(6, this->ndofActuated()); Jtmp.setZero();
    // This is in the local link frame (Note that the last argument must be
    // false [because previously there was true for xlink] ... why???)
    CalcBodySpatialJacobian(*rmodel_, q_, link_number, Jtmp, false);
    // Rotation: link frame wrt world (inertial) frame
    Rlink = quaternionToRotation(linkOrientation(link_number));

    J.resize(6, this->ndof()); 
    J.setZero();
    J.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
    J.block(0,3,3,4) = skew(q_.head(3)-xlink)*T;
    J.topRightCorner(3,this->ndofActuated()) = Rlink*Jtmp.bottomRows(3);

    J.block(3,3,3,4) = T;
    J.bottomRightCorner(3,this->ndofActuated()) = Rlink*Jtmp.topRows(3);

  }

  else
  {
    // CalcPointJacobian(*rmodel_, q_, link_number, local_pos, Jtmp, true);
    J.resize(6, this->ndof());
    Jtmp.resize(6, this->ndof());
    Jtmp.setZero();               // Important for RBDL
    // This is in the local link frame
    CalcBodySpatialJacobian(*rmodel_, q_, link_number, Jtmp, true);
    // Orientation from local link frame to base frame
    Eigen::Matrix3d R;
    R = quaternionToRotation(linkOrientation(link_number));
    // Convert to fixed frame and swap order of linear/angular
    J.topRows(3) = R*Jtmp.bottomRows(3);
    J.bottomRows(3) = R*Jtmp.topRows(3);
  }

  return J;
}


Eigen::VectorXd RobotModelRbdl::comPosition()
{
  double mass;
  RigidBodyDynamics::Math::Vector3d com;

  // RigidBodyDynamics::Utils::CalcCenterOfMass(
  //   *rmodel_, q_, Eigen::VectorXd::Zero(rmodel_->qdot_size), mass, com);

  if(has_floating_base_)
  {
    RigidBodyDynamics::Utils::CalcCenterOfMass(
      *rmodel_, q_.tail(this->ndofActuated()),
      Eigen::VectorXd::Zero(this->ndofActuated()), mass, com);

    Eigen::Matrix3d Rbase;
    Rbase = quaternionToRotation(q_.segment(3,4));
    com = Rbase*com + q_.head(3);
  }
  else
    RigidBodyDynamics::Utils::CalcCenterOfMass(
      *rmodel_, q_, Eigen::VectorXd::Zero(this->ndofActuated()), mass, com);

  return com;

}


Eigen::MatrixXd RobotModelRbdl::comJacobian()
  const
{

}

}
