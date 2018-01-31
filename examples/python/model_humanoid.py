# ================================================================
#
# Model of the sampleHumanoid robot using RobotBase
#
#  - It parses the model and shows the basic information
#  - It shows how to compute forward kinematics
#  - It shows how to compute the geometric Jacobian
#
# ================================================================

from oscr.robot_base import RobotBase
import numpy as np

# ------------------------
# Loading the robot model
# ------------------------
# Does the robot have a floating base?
fbase = True
# What backend will be used (pin or rbdl)?
backend = 'pin'
# Model name
modelName = "../models/sampleHumanoid.urdf"

robot = RobotBase(modelName, fbase, backend)

# --------------------------------
# Information from the robot model
# --------------------------------

manual = False
if (manual):
    print '\nndof:', robot.ndof
    print 'ndofActuated:', robot.ndofActuated
    print 'hasFloatingBase:', robot.hasFloatingBase
    print 'floatingLink:', robot.floatingLink

    print 'jointMaxAngularLimits:\n', robot.qmax.T
    print 'jointMinAngularLimits:\n', robot.qmin.T
    print 'jointVelocityLimits:\n', robot.dqmax.T

    print '* jointNames (in order):\n', robot.jointNames
    print '* jointNames and IDs:\n', robot.mjoint
    print '* linkNames and IDx:\n', robot.mlink
else:
    robot.printModelInfo()

# ---------------------------------------
# Set a joint configuration for the robot
# ---------------------------------------

q = np.zeros((robot.ndof,1))
if (fbase):
    q[3] = 1.0
# Apply the joint configuration to the robot model
robot.updateJointConfig(q)
print '* getJointConfig:\n', robot.getJointConfig().T

# ------------------
# Forward kinematics
# ------------------

# Choosing a link (by name and its number)
linkName = 'Head1'
linkNumber = robot.mlink[linkName]

# Using the link name (choose names from linkNames)
pos1 = robot.linkPosition(linkName)
quat1 = robot.linkOrientation(linkName)
pose1 = robot.linkPose(linkName)

# Using the link number
pos2 = robot.linkPosition(linkNumber)
quat2 = robot.linkOrientation(linkNumber)
pose2 = robot.linkPose(linkNumber)

print '\nForward kinematics for link:', linkName
print backend, '-', 'position (using name): ', pos1.transpose()
print backend, '-', 'position (using id): ', pos2.transpose()
print backend, '-', 'orientation (using name): ', quat1.transpose()
print backend, '-', 'orientation (using id): ', quat2.transpose()
print backend, '-', 'pose (using name): ', pose1.transpose()
print backend, '-', 'pose (using id): ', pose2.transpose()

# ------------------
# Geometric Jacobian
# ------------------

linearJ = robot.linearJacobian(linkNumber)
angularJ = robot.angularJacobian(linkNumber)
geometricJ = robot.geometricJacobian(linkNumber)

printJacobians = False
if (printJacobians):
    print backend, '-', 'linear Jacobian:\n', linearJ
    print backend, '-', 'angular Jacobian:\n', angularJ
    print backend, '-', 'geometric Jacobian:\n', geometricJ
else:
    print '\nTo print Jacobians, set printJacobians to True'
