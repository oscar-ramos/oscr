# ================================================================
#
# Model of the sampleHumanoid robot WITHOUT using RobotBase
#
#  - It parses the model and shows the basic information
#  - It shows how to compute forward kinematics
#  - It shows how to compute the geometric Jacobian
#
# ================================================================

from oscr.oscr import RobotModelPin, RobotModelRbdl
import numpy as np

# ------------------------
# Loading the robot model
# ------------------------
# Does the robot have a floating base?
fbase = True
# What backend will be used (pin or rbdl)?
backend = 'rbdl'
# Model name
modelName = "../models/sampleHumanoid.urdf"

if (backend=='pin'):
    rmodel = RobotModelPin(modelName, fbase)
elif (backend=='rbdl'):
    rmodel = RobotModelRbdl(modelName, fbase)

# --------------------------------
# Information from the robot model
# --------------------------------

ndof = rmodel.ndof()
ndofActuated = rmodel.ndofActuated()
hasFloatingBase = rmodel.hasFloatingBase()
floatingLink = rmodel.floatingLink()
qmax  = rmodel.jointMaxAngularLimits().transpose()
qmin  = rmodel.jointMinAngularLimits().transpose()
dqmax = rmodel.jointVelocityLimits().transpose()
jointNames = rmodel.jointNames()
mapLinkNamesIDs = rmodel.mapLinkNamesIDs()
linkNames = mapLinkNamesIDs.keys()
linkIDs = mapLinkNamesIDs.values()

print '\nndof:', ndof
print 'ndofActuated:', ndofActuated
print 'hasFloatingBase:', hasFloatingBase
print 'floatingLink:', floatingLink

print 'jointMaxAngularLimits:\n', qmax
print 'jointMinAngularLimits:\n', qmin
print 'jointVelocityLimits:\n', dqmax

print '* jointNames (in order):\n', jointNames
print '* linkNames:\n', linkNames
print '* linkIDs:\n', linkIDs

# ---------------------------------------
# Set a joint configuration for the robot
# ---------------------------------------

q = np.zeros((ndof,1))
if (fbase):
    q[3] = 1.0
# Apply the joint configuration to the robot model
rmodel.updateJointConfig(q)
print '* getJointConfig:\n', rmodel.getJointConfig().transpose()

# ------------------
# Forward kinematics
# ------------------

# Choosing a link (by name and its number)
linkName = 'Head1'
linkNumber = mapLinkNamesIDs[linkName]

# Using the link name (choose names from linkNames)
pos1 = rmodel.linkPosition(linkName)
quat1 = rmodel.linkOrientation(linkName)
pose1 = rmodel.linkPose(linkName)

# Using the link number
pos2 = rmodel.linkPosition(linkNumber)
quat2 = rmodel.linkOrientation(linkNumber)
pose2 = rmodel.linkPose(linkNumber)

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

linearJ = rmodel.linearJacobian(linkNumber)
angularJ = rmodel.angularJacobian(linkNumber)
geometricJ = rmodel.geometricJacobian(linkNumber)

printJacobians = False
if (printJacobians):
    print backend, '-', 'linear Jacobian:\n', linearJ
    print backend, '-', 'angular Jacobian:\n', angularJ
    print backend, '-', 'geometric Jacobian:\n', geometricJ
else:
    print '\nTo print Jacobians, set printJacobians to True'
