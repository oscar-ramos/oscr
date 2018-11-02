import numpy as np

from oscr import KineTaskPose

# Specific for ROS
from ros_markers import BallMarker, FrameMarker, color
from ros_pubs import PathPub

class RosKineTaskPose(object):
    """
    Wrapper for KineTaskPose including ROS markers for visualization

    """

    def __init__(self, robot_model, linkID, taskType, taskName,
                 show_markers=True, show_path=None):
        """
        Constructor. It sets the weight to a default value of 10 (only used
        in the WQP solver)

          robot_model - object of RobotModelPin or RobotModelRbdl
          linkID - link that will be controlled
          taskType - 'position', 'orientation' or 'pose'
          taskName - string that describes a name for the task
          show_markers - Show markers for the tasks (True/False)

        """
        self.ctask = KineTaskPose(robot_model, linkID, taskType, taskName)
        # Expose some C++ members
        self.getTaskDim = self.ctask.getTaskDim
        self.getDerivError = self.ctask.getDerivError
        self.setWeight = self.ctask.setWeight
        self.setGain = self.ctask.setGain
        # Redefine some members for ease of use
        self.getDesiredValue = self.ctask.getDesiredValue
        self.sensedValue = np.zeros((self.ctask.getTaskDim(), 1))
        self.derror = np.zeros((self.ctask.getTaskDim(), 1))
        # Default values
        self.setWeight(10.0)
        # Whether to show the path
        self.show_path = False

        if (show_markers):
            self.setDesiredValue = self._setDesiredValueWithMarkers
            self.keep = self.keepWithMarkers
            # * The first marker represents the current value
            # * The second marker represents the desired value
            if (taskType=='position'):
                # Ball markers if task is only position
                self.marker = [BallMarker(color['RED']),
                               BallMarker(color['GREEN'],0.7)]
            else:
                # Frame markers if task is position and orientation
                self.marker = [FrameMarker(),
                               FrameMarker(0.7, alpha=0.5)]
            if (show_path==None):
                self.path_pub = PathPub()
                self.show_path = True

        else:
            self.marker = [FakeMarker(), FakeMarker()]
            self.setDesiredValue = self.ctask.setDesiredValue
            self.keep = self.ctask.keep

        if (show_path):
            self.path_pub = PathPub()
            self.show_path = True


    def getError(self):
        """
        Return the updated task error (ctask.getError provides the internal
        task error which is not updated unless getDerivError is called)

        """
        self.ctask.getDerivError(self.derror)
        return self.ctask.getError()


    def getSensedValue(self):
        """
        Return the sensed values using the internal joint configuration

        """
        self.ctask.getSensedValue(self.sensedValue)
        return self.sensedValue


    def _setDesiredValueWithMarkers(self, desiredValue):
        """
        Set the desired values to the task and to the marker
        (Avoid using this method directly)

        """
        self.ctask.setDesiredValue(desiredValue)
        self.marker[1].setPose(desiredValue)


    def keepWithMarkers(self, gain):
        """
        Keep the current sensed values setting the desired value to the marker

        """
        self.ctask.keep(gain)
        desiredValue = self.ctask.getDesiredValue()
        self.marker[1].setPose(desiredValue)

    def setMask(self, mask):
        self.ctask.setMask(mask)
        # Redefine some members for ease of use
        self.sensedValue = np.zeros((self.ctask.getTaskDim(), 1))
        self.derror = np.zeros((self.ctask.getTaskDim(), 1))


class FakeMarker(object):
    def __init__(self):
        pass

    def setPose(self, pose):
        pass

    def publish(self):
        pass
