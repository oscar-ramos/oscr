
from oscr import RobotModelRbdl, RobotModelPin, RobotModelff6dof
from utils import *
import numpy as np


class RobotBase(object):
    """
    Generic helper class for a robot

    """

    def __init__(self, model_name, fbase, backend):
        """
        Constructor.

          model_name - path to the urdf model
          fbase - whether there is floating base (True/False)
          backend - can be 'pin' (for pinocchio)  or 'rbdl'

        """
        # Robot model
        if (backend=='rbdl'):
            self.model = RobotModelRbdl(model_name, fbase)
        elif (backend=='pin'):
            self.model = RobotModelPin(model_name, fbase)
        elif (backend=='ff6dof'):
            self.model = RobotModelff6dof(fbase)
        else:
            print 'ERROR: Invalid Backend!!! Use pin or rbdl'
        # Model elements
        self.ndof = self.model.ndof()
        self.ndofActuated = self.model.ndofActuated()
        self.hasFloatingBase = self.model.hasFloatingBase()
        self.floatingLink = self.model.floatingLink()
        self.qmax  = self.model.jointMaxAngularLimits()
        self.qmin  = self.model.jointMinAngularLimits()
        self.dqmax = self.model.jointVelocityLimits()
        self.jointNames = self.model.jointNames()
        mapLinkNamesIDs = self.model.mapLinkNamesIDs()
        # Map for the links
        self.mlink = dictLink(mapLinkNamesIDs)
        # Map for the joints
        self.mjoint = dictJoint(self.jointNames, fbase)
        # Set joints to zero
        q = np.zeros((self.ndof,1))
        self.model.updateJointConfig(q)
        # Aliases for RobotModel functions
        self.updateJointConfig = self.model.updateJointConfig
        self.linkPosition = self.model.linkPosition
        self.linkOrientation = self.model.linkOrientation
        self.linkPose = self.model.linkPose
        self.linearJacobian = self.model.linearJacobian
        self.angularJacobian = self.model.angularJacobian
        self.geometricJacobian = self.model.geometricJacobian
        self.getJointConfig = self.model.getJointConfig

    def printModelInfo(self):
        """
        Print generic information about the robot model

        """
        print '\nTotal ndof:', self.ndof, '(actuated:', \
              self.ndofActuated, ")"
        print 'Has floating base: ', self.hasFloatingBase
        print 'floatingLink: ', self.floatingLink
        printJointLimits(self.jointNames, self.qmin, self.qmax, self.dqmax)
        print 'Link names and IDs (map):\n ', self.mlink
        print 'Joint names and IDs (map):\n ', self.mjoint
        print 'Joints (in order):\n ', self.jointNames
