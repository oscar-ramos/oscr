*OSCR* - **O**perational **S**pace **C**ontrol for Redundant **R**obots

The following instructions assume little or no experience with Linux
environments and cmake tools. The source code will be stored in
`~/dev/control/src` and the installation will be done in
`~/dev/control/install`.


Building and Installation of the C++ core
=========================================

+ Open a terminal

+ Create a folder where the source files will be copied to

        mkdir -p ~/dev/control/src

+ Clone the [*oscr-deps*](https://bitbucket.org/oscarefrain/oscr-deps)
  (dependencies) package in the created folder from the repository

        cd ~/dev/control/src
        git clone https://bitbucket.org/oscarefrain/oscr-deps

+ Go to the cloned repository, compile and install oscr-deps

        cd oscr-deps
        ./install-deps ~/dev/control/install SET_BASH
        source ~/.bashrc

+ Clone this package (oscr) in the source folder:

        cd ~/dev/control/src
        git clone https://bitbucket.org/oscarefrain/oscr

+ Go to the cloned repository

        cd oscr

+ Create a directory for compilation and compile the package

        mkdir build
        cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=~/dev/control/install
        make

+ Check that the compilation has been successful by executing 'ff6dofModel'
  (some information should be shown on the screen without any error messages)

        cd examples/c++
        ./ff6dofModel

+ Install the package (it will be installed in *~/dev/control/install*)

        cd ../..
        make install


Installation of Python Bindings
===============================

+ The SWIG package is needed. Install it as (on Debian-based distributions):

        sudo apt-get install swig

+ Add the installed python files to PYTHONPATH (in .bashrc so it is accessible
  to every terminal)

        echo "export PYTHONPATH:~/dev/control/install/lib/python2.7/site-packages:$PYTHONPATH" >> ~/.bashrc

+ Make the library loscr available in the system (using the *LD_LIBRARY_PATH*
  environmental variable)

        echo "export LD_LIBRARY_PATH:~/dev/control/install/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

+ To test that Python has been properly set execute:

        cd ~/dev/control/src/oscr
        cd unitTests/python
        python testMathUtils.py

+ Some of the other test files in *unitTests/python* and in *examples/python*
  can be executed in a similar way.


Documentation
=============

+ Install doxygen and doxygen-latex for documentation

        sudo apt-get install doxygen doxygen-latex

+ Generate the documentation

        make doc

+ View the documentation (e.g. using Firefox)

        cd doc/doxygen-html
        firefox index.html
