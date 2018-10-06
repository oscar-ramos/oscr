%module kineTaskPose

// Headers from Python itself and from this library
%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "oscr/ik/kine-task.hpp"
//#include "oscr/ik/kine-task-pose.hpp"
%}

%import "robotModelPin.i"
%import robotModelRbdl.i

// typemaps.i: built-in swig interface that lets us map c++ types to other
// types (used here to map Eigen matrices to Numpy arrays).
%include <typemaps.i>
%include <std_vector.i>

// eigen.i (found in swig-eigen) contains specific definitions to convert Eigen
// matrices into Numpy arrays.
%include <eigen.i>

/* %template(vectorMatrixXd) std::vector<Eigen::MatrixXd>; */
/* %template(vectorVectorXd) std::vector<Eigen::VectorXd>; */

// Other needed includes
%include <std_string.i>

%include <stl.i>
 //%include <std_map.i>
/* instantiate the required template specializations */
/* namespace std  */
/* { */
/*   %template(IntVector)      vector<int>; */
/*   %template(UintVector)     vector<unsigned int>; */
/*   %template(DoubleVector)   vector<double>; */
/*   %template(StringVector)   vector<string>; */
/*   %template(StringUintMap)  map<string, unsigned int>; */
/* } */

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Vector4d)
// Functions don't compile correctly unless we also declare typemaps for
// Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>.
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

// Tell swig to build bindings for everything in our library
%feature("autodoc", "3");
%include "oscr/ik/kine-task.hpp"
%include "oscr/ik/kine-task-pose.hpp"
