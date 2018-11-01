# Robot model from OSCR
from robot_base import RobotBase

# Generic ROS
import rospy
from roslib import packages
# Specific helpers in ROS
from ros_pubs import JointStatePub, JointCommandPub
from ros_markers import BallMarker, FrameMarker, color


class RosRobot(RobotBase):
    """
    Class for the robot in ROS

    """

    def __init__(self, node_name, package, robot_model, freq, fbase, backend,
                 type_robot='rviz'):
        """
        Constructor

          node_name - ROS node name
          package - ROS package where the URDF can be found
          robot_model - name of the URDF file
          freq - ROS main loop frequency (control frequency)
          fbase - whether there is floating base (True/False)
          backend - can be 'pin' (for pinocchio)  or 'rbdl'
          type_robot - For simulation or visualization ('rviz' or 'gazebo')

        """
        # ROS node initialization
        rospy.init_node(node_name)
        # ROS update frequency
        self.freq = freq
        # ROS rate: it must be called in the main loop
        self.rate = rospy.Rate(freq)
        # Robot model
        self.pkg = str(packages.get_pkg_dir(package))
        full_model_name = self.pkg + robot_model
        # Initialize RobotBase
        RobotBase.__init__(self, full_model_name, fbase, backend)
        if (type_robot == 'rviz'):
            # Publisher of joint states (used for RViz)
            self.joint_pub = JointStatePub(self.ndof,
                                           self.hasFloatingBase)
            self.joint_pub.setJointNames(self.jointNames)
            self.publishJoints(self.getJointConfig())
        elif (type_robot == 'gazebo'):
            # Publisher of joint commands (used for Gazebo)
            self.joint_pub = JointCommandPub(self.ndof,
                                             self.hasFloatingBase)
            self.joint_pub.setJointNames(self.jointNames)
        # Initialize ball markers and frame markers to empty
        self.ballMarker  = []
        self.frameMarker = []
        self.ballMarkerLink  = []
        self.frameMarkerLink = []
        # For extra joints
        self.qextras = []


    def update(self, q, q_extra=False):
        """
        Update the robot joint configuration and publish the configuration.
        Note that there is no rate.sleep

          q - joint configuration (numpy array)
          q_extra - extra joints, e.g. for grippers (numpy array)

        """
        self.updateJointConfig(q)
        if (self.qextras == []):
            self.publishJoints(self.getJointConfig(), q_extra)
        else:
            self.publishJoints(self.getJointConfig(), self.qextras)

    def updateWithMarkers(self, q, q_extra=False):
        """
        Update the robot joint configuration and publish the configuration as
        well as the updated markers (if they exist).
        Note that there is no rate.sleep

          q - joint configuration (numpy array)
          q_extra - extra joints, e.g. for grippers (numpy array)

        """
        self.updateJointConfig(q)
        if (self.qextras == []):
            self.publishJoints(self.getJointConfig(), q_extra)
        else:
            self.publishJoints(self.getJointConfig(), self.qextras)
        # Publish ball markers, if any
        for i in xrange(len(self.ballMarker)):
            self.ballMarker[i].setPose(
                self.linkPosition(self.ballMarkerLink[i]))
        # Publish frame markers, if any
        for i in xrange(len(self.frameMarker)):
            self.frameMarker[i].setPose(
                self.linkPose(self.frameMarkerLink[i]))

    def initRobotBallMarkers(self, colors, links):
        """
        Initialize ball markers that are associated with a specific robot link

          colors - list of strings containing colors. The colors can be: 'RED',
                   'GREEN', 'BLUE', 'YELLOW', 'PINK', 'CYAN', 'BLACK',
                   'DARKGRAY', 'LIGHTGRAY', 'WHITE' (in ros_utils)
          links  - list of strings containing link names

        """
        for i in xrange(len(colors)):
            self.ballMarker.append( BallMarker(color[colors[i]]) )
            self.ballMarkerLink.append( self.mlink[links[i]] )

    def initRobotFrameMarkers(self, saturations, links):
        """
        Initialize frame markers that are associated with a specific robot link

          saturations - list of strings containing frame saturations (0 to 1)
          links - list of strings containing link names

        """
        for i in xrange(len(saturations)):
            self.frameMarker.append( FrameMarker(saturations[i]) )
            self.frameMarkerLink.append( self.mlink[links[i]] )

    def publishJoints(self, q, q_extra=False):
        """
        Publish joint configurations to the appropriate topic

          q - joint configuration (numpy array)
          q_extra - extra joints, e.g. for grippers (numpy array)

        """
        self.joint_pub.publish(q, q_extra)
