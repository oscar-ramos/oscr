from oscr.oscr import RobotModelRbdl
import numpy as np

# ------------------------
# Loading the robot model
# ------------------------
fbase = False
model = RobotModelRbdl("../models/sampleHumanoid.urdf", fbase)

# --------------------------
# General model information
# --------------------------

ndof = model.ndof()
ndofActuated = model.ndofActuated()
hasFloatingBase = model.hasFloatingBase()
floatingLink = model.floatingLink()
qmax  = model.jointMaxAngularLimits().transpose()
qmin  = model.jointMinAngularLimits().transpose()
dqmax = model.jointVelocityLimits().transpose()
jointNames = model.jointNames()
mapLinkNamesIDs = model.mapLinkNamesIDs()
linkNames = mapLinkNamesIDs.keys()
linkIDs = mapLinkNamesIDs.values()

print '\nndof: ', ndof
print 'ndofActuated: ', ndofActuated
print 'hasFloatingBase: ', hasFloatingBase
print 'floatingLink: ', floatingLink

print 'jointMaxAngularLimits:\n', qmax
print 'jointMinAngularLimits:\n', qmin
print 'jointVelocityLimits:\n', dqmax

print 'jointNames:\n', jointNames
print 'linkNames:\n', linkNames
print 'linkIDs:\n', linkIDs

# -----------------------------
# Link position and orientation
# -----------------------------

q = np.zeros((ndof,1))
model.updateJointConfig(q)
print 'getJointConfig:\n', model.getJointConfig().transpose()

# Using link name
pHead = model.linkPosition('Head1')
# oHead = model.linkOrientation('Head')
# Using link number
pHead2 = model.linkPosition(10)
# oHead2 = model.linkOrientation(2)

print '[rbdl] Head position (using name): ', pHead.transpose()
print '[rbdl] Head position (using id): ', pHead2.transpose()
# print 'Head orientation: ', oHead.transpose()

