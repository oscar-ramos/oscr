prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_FULL_LIBDIR@
includedir=@CMAKE_INSTALL_FULL_INCLUDEDIR@

Name: Operational Space Control for Redundant Robots (OSCR)
Description: Whole-body Motion Generation
URL: TODO
Version: @PROJECT_VERSION@
Requires: eigen3, rbdl, pinocchio, qpoases
Conflicts:
Libs: -L${libdir} -loscr -Wl
Libs.private:
Cflags: -I${includedir}
