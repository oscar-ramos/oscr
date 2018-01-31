*OSCR* - **O**perational **S**pace **C**ontrol for Redundant **R**obots


Introduction
============

OSCR is a C++ library that allows to generate whole-body motion for
multi-articulated robots, currently using inverse kinematics.

For very detailed installation instructions (if you are new to Linux) take a
look at [INSTALL.md](INSTALL.md). Otherwise, the following instructions should be
enough.


Building and Installation of the C++ core
=========================================

Before installing OSCR, it is recommended to install the
[oscr-deps](https://github.com/oscar-ramos/oscr-deps) package which contains
the required dependencies (refer to this package for installation
instructions). These dependencies can also be installed manually one by one,
but care should be taken for compatibility.

To compile the library OSCR in a separate directory use:

    mkdir build
    cd build/
    cmake .. -DCMAKE_INSTALL_PREFIX=your_prefix
    make

where *your_prefix* is the path where the library will be installed. You can
check that the compilation has been successful by executing the *ff6dofModel*
executable that can be found in *build/examples/c++* (the source code can be
found in *examples/c++*). If some joint and link information are shown without
errors, then the package has been correctly compiled.

To install this package use:

    make install


Installation of Python Bindings
===============================

To use the python bindings (still under development) you must add the python
installation path (your_prefix/lib/python2.7/site-packages) to the
environmental variable `PYTHONPATH` in .bashrc. The library loscr must also be
accessible by adding the libray installation path (your_prefix/lib) to the
environmental variable `LD_LIBRARY_PATH` in .bashrc.

To test that Python has been properly set, the files in unitTests/python and in
examples/python can be executed. They should return information about the robot
model.


Documentation
=============

The documentation is contained in the code and can be extracted using
doxygen. To generate it, use:

    make doc

This will generate documentation in *build/doc/doxygen-html*. To see it, launch
index.html with your favorite browser.


### Dependencies

The OSCR library depends on the following libraries which have to be available
on your machine.

 - Libraries:
     - rbdl (provided by oscr-deps)
     - qpOASES (provided by oscr-deps)
     - pinocchio (provided by oscr-deps)
     - Eigen
     - swig
 - System tools:
     - CMake (>=2.8)
     - pkg-config
     - usual compilation tools (GCC/G++, make, etc.)
 - For documentation:
     - doxygen
     - doxygen-latex
