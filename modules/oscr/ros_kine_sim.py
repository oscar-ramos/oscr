import numpy as np

# Kinematic solvers from OSCR
from oscr import OSIKSolverNS, OSIKSolverWQP, OSIKSolverHQP

# Specific for ROS
from ros_robot import RosRobot
from ros_kine_tasks import RosKineTaskPose
from ros_markers import BallMarker, FrameMarker, color


class RosKineSim(object):
    """
    Class for the robot, tasks and solver in ROS

    """

    def __init__(self, node_name, package, robot_model, freq, fbase, backend,
                 type_robot='rviz', show_markers=True):
        """
        Constructor

          node_name - ROS node name
          package - ROS package where the URDF can be found
          robot_model - name of the URDF file
          freq - ROS main loop frequency (control frequency)
          fbase - whether there is floating base (True/False)
          backend - can be 'pin' (for pinocchio)  or 'rbdl'
          type_robot - For simulation or visualization ('rviz' or 'gazebo')
          show_markers - Show markers for the tasks (True/False)

        """
        # Initialize robot in ROS
        self.robot = RosRobot(node_name, package, robot_model, freq, fbase,
                              backend, type_robot)
        # Initialize tasks to empty
        self.task = dict()
        # Internal variables
        self.freq = freq
        self.show_markers = show_markers
        self.rate = self.robot.rate
        # Allocate space for the joint command
        self.qcommand = np.zeros((self.robot.ndof, 1))
        # Publish markers or do not publish them
        if (show_markers):
            self.update = self._updateModelWithTaskMarkers
        else:
            self.update = self._updateModel


    def initKineSolver(self, q=[], solverType='HQP'):
        """
        Initialize a kinematic solver.
          q - joint value to initialize the solver
          solverType - 'NS', 'WQP' or 'HQP'

        """
        if (q == []):
            q = self.robot.getJointConfig()
        if (solverType=='NS'):
            self.solver = OSIKSolverNS(self.robot.model, q, 1.0/self.freq)
        elif (solverType=='WQP'):
            self.solver = OSIKSolverWQP(self.robot.model, q, 1.0/self.freq)
            # Joint limits are added for WQP solver
            self.solver.setJointLimits(self.robot.qmin, self.robot.qmax,
                                       self.robot.dqmax)
        elif (solverType=='HQP'):
            self.solver = OSIKSolverHQP(self.robot.model, q, 1.0/self.freq)
            # Set joint limits for HQP solver (this is optional for HQP)
            self.solver.setJointLimits(self.robot.qmin, self.robot.qmax,
                                       self.robot.dqmax)
        else:
            print 'ERROR: only NS, WQP and HQP solvers are currently allowed'


    def initKineTaskPose(self, linkName, taskType, taskName=None, gain=1.0,
                         show_path = None, show_markers=[]):
        """
        Wrapper to initialize (declare) a kinematic position/orientation task.
        Note that this does not add the task to the solver.

          linkName - string containing the name of the link
          taskType - can be 'position', 'orientation', 'pose'
          taskName - name of the task (default is linkName)
          gain - task gain

        """
        linkID = self.robot.mlink[linkName]
        if (show_markers==[] or show_markers==True):
            task = RosKineTaskPose(self.robot.model, linkID, taskType,
                                   taskName, self.show_markers, show_path)
        elif (show_markers==False):
            task = RosKineTaskPose(self.robot.model, linkID, taskType,
                                   taskName, False, show_path)
        task.ctask.setGain(gain)
        # Set name for the task dictionary
        if (taskName==None):
            taskName = linkName
        self.task[taskName] = task


    def _updateModel(self, q, q_extra=False):
        """
        Update the robot joint configuration and publish the configuration.
        (Avoid using this method directly)

          q - joint configuration (numpy array)
          q_extra - extra joints, e.g. for grippers (numpy array)

        """
        self.robot.update(q, q_extra)


    def _updateModelWithTaskMarkers(self, q, q_extra=False):
        """
        Update both the robot joint configuration and the markers for each
        task. Also, publish the configuration and markers.
        (Avoid using this method directly)

          q - joint configuration (numpy array)
          q_extra - extra joints, e.g. for grippers (numpy array)

        """
        self.robot.update(q, q_extra)
        for i in self.task:
            # Set and publish the current value
            current = self.task[i].getSensedValue()
            self.task[i].marker[0].setPose(current)
            # Publish the desired value
            self.task[i].marker[1].publish()
            if (self.task[i].show_path):
                # Publish the path
                self.task[i].path_pub.publish(current)


    def doTask(self, taskName, desiredValue, maxError=0.005, log=[]):
        """
        Do a certain task until the error is smaller than maxError. It calls
        its own rate.sleep internally (do not use an external sleep)

          taskName - string containing the name of the task
          desiredValue - desired for the task (e.g. 7x1 numpy vector)
          maxError - maximum tolerance for the task error

        """
        # Set the task desired value
        self.task[taskName].setDesiredValue(desiredValue)
        # Get the robot configuration
        q = self.robot.getJointConfig()
        # Save log?
        if (log==[]):
            savelog = False
        else:
            savelog = True
        # The following 2 lines are only to recompute the error
        while (np.linalg.norm(self.task[taskName].getError()) > maxError):
            if (savelog):
                log.save(q)
            self.solver.getPositionControl(q, self.qcommand)
            self.update(self.qcommand)
            q = self.qcommand.copy()
            self.rate.sleep()


    def getPositionControl(self, q):
        """
        Proxy to get the position control from the solver

        """
        self.solver.getPositionControl(q, self.qcommand)
        return self.qcommand


    def pushTask(self, task):
        """
        Proxy to push a task to the solver

        """
        self.solver.pushTask(task.ctask)


    def removeTask(self, task_name):
        """
        Proxy to remove a task from the solver

        """
        self.solver.removeTask(task_name)
