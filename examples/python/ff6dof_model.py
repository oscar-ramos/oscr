#
# Test the parsing of the model of a simple revolute robot with a free floating
# base and 6 dof (2 chains of 3 dof each)
#
# To use it interactively: ipython -i ff6dof_model.py
#
# To use rbdl: ipython -i ff6dof_model.py --back rbdl
#

from optparse import OptionParser
import numpy as np

from oscr.oscr import RobotModelPin, RobotModelRbdl
from oscr.utils import dictLink, printJointLimits, printLinkNameIDShort

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--back", dest="backend", default="pin",
                      help="Backend to be used: pin (default) or rbdl")
    (options, args) = parser.parse_args()
    backend = options.backend
    
    # Use a floating base?
    fbase = True
    # Robot model
    modelname = "../models/ff6dof.urdf"
    # Initialization
    if (backend=="pin"):
        print '-- Using Pinocchio as backend'
        robot = RobotModelPin(modelname, fbase)
    elif (backend=="rbdl"):
        print '-- Using RBDL as backend'
        robot = RobotModelRbdl(modelname, fbase)
    else:
        print '\nOnly pin and rbdl are accepted as backends ...',\
              "ending the program.\n"
        exit(0)

    # Number of Degrees of Freedom
    ndof  = robot.ndof()
    # Number of Actuated Degrees of Freedom
    ndof_actuated = robot.ndofActuated()
    # Floating link
    has_floating_base = robot.hasFloatingBase()
    floating_link = robot.floatingLink()
    # Joint angular and velocity limits
    qmin  = robot.jointMinAngularLimits()
    qmax  = robot.jointMaxAngularLimits()
    dqmax = robot.jointVelocityLimits()
    # Jont names
    jnames = robot.jointNames()
    # Link names and IDs
    linkID = dictLink(robot.mapLinkNamesIDs())
    
    print 'Total ndof: ', ndof
    print 'Total ndof actuated: ', ndof_actuated
    print 'Has floating base: ', has_floating_base
    print 'FloatingLink: ', floating_link, '\n'
    printJointLimits(jnames, qmin, qmax, dqmax)
    printLinkNameIDShort(linkID)

    # Joint configuration with all values to zero.
    q = np.zeros(ndof)
    # If there is a floating base, then set a valid null quaternion
    if (has_floating_base):
        q[3]=1.0
    # Set the current joint configuration
    robot.updateJointConfig(q)
  
    # Get the current joint configuration
    qcurrent = robot.getJointConfig()

    # Get the pose of larm3 and lleg3 and the Jacobian of larm3
    poselarm = robot.linkPose(linkID["larm3"])
    poselleg = robot.linkPose(linkID["lleg3"])
    Jlarm = robot.geometricJacobian(linkID["larm3"])
    
    print "Current joint config: ", qcurrent.T
    print "Pose of larm3: ", poselarm.T
    print "Pose of lleg3: ", poselleg.T
    print "Geometric Jacobian of larm3:\n ", Jlarm
   
