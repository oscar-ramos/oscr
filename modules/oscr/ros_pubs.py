
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np

class JointStatePub(object):
    """
    Class to publish joint states for display in RViz

    """
    def __init__(self, ndof, fbase, topic='joint_states', buffer=1000):
        self.ndof = ndof
        self.has_floating_base = fbase
        self.jstate_pub = rospy.Publisher(topic, JointState, queue_size=buffer)
        self.jstate = JointState()
        self.jstate.header.stamp = rospy.Time.now()
        self.no_extra_joints = True

    def setJointNames(self, jnames):
        if self.has_floating_base:
            floating_base_names = ('Px','Py','Pz','W','Ex','Ey','Ez')
            self.jstate.name = floating_base_names+jnames
            # rospy.logerror('setJointNames with floating base.')
        else:
            self.jstate.name = jnames

    def appendJointNames(self, jnames_extra):
        """
        Append extra names for extra joints (e.g. when the gripper is not in the
        urdf model that has been parsed)

          jnames_extra - tuple of strings

        """
        self.jstate.name = self.jstate.name + jnames_extra
        self.no_extra_joints = False

    def publish(self, q, q_extra=False):
        """
        Publish the joints. If extra joints are added, their names must be also
        set using appendJointNames()

          q_extra - extra joints, e.g. for grippers (numpy column array)

        """
        if (self.no_extra_joints):
            self.jstate.position = q
        else:
            self.jstate.position = np.vstack([q, q_extra])
        self.jstate.header.stamp = rospy.Time.now()
        self.jstate_pub.publish(self.jstate)


class JointCommandPub(object):
    """
    Class to publish joint commands for Gazebo

    """
    def __init__(self, ndof, fbase, topic='joint_angles', buffer = 1000):
        self.ndof = ndof
        # TODO

    def setJointNames(self, jnames):
        # TODO
        pass

    def publish(self, q, q_extra=False):
        # TODO
        pass


class PathPub(object):
    """
    Class to publish the path for display in RViz

    """
    def __init__(self, topic='op_path', buffer=1000):
        """
        Constructor. Important: the frame_id for the message is directly
        obtained from the parameter 'reference_frame', which must be set.

          topic - where the path will be published.
          buffer - buffer size

        """
        self.path_pub = rospy.Publisher(topic, Path, queue_size=buffer)
        self.path = Path()
        frame_id = rospy.get_param('~reference_frame', 'base_link')
        self.path.header.frame_id = frame_id

    def publish(self, pose):
        """
        Add the pose to the path and publish

        """
        self.path.header.stamp = rospy.Time.now()
        # Create a new pose
        self.kpose = PoseStamped()
        self.kpose.header.stamp = rospy.Time.now()
        # New position
        self.kpose.pose.position.x = pose[0,0]
        self.kpose.pose.position.y = pose[1,0]
        self.kpose.pose.position.z = pose[2,0]
        # New orientation
        if (len(pose)>4): # 4 to avoid problems with only orientation
            # TO-DO: handle orientation only in a better way
            self.kpose.pose.orientation.x = pose[4,0]
            self.kpose.pose.orientation.y = pose[5,0]
            self.kpose.pose.orientation.z = pose[6,0]
            self.kpose.pose.orientation.w = pose[3,0]
        else:
            self.kpose.pose.orientation.x = 0
            self.kpose.pose.orientation.y = 0
            self.kpose.pose.orientation.z = 0
            self.kpose.pose.orientation.w = 1
        # Set pose to the path and publish
        self.path.poses.append(self.kpose)
        self.path_pub.publish(self.path)
