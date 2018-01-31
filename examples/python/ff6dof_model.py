#
# Test the parsing of the model of a simple revolute robot with a free floating
# base and 6 dof (2 chains of 3 dof each)
#
# To use it: ipython -i ff6dof_model.py
#


from oscr.robot_base import RobotBase
import numpy as np


if __name__ == '__main__':

    # Whether floating base is used or not
    fbase = True
    # Frequency
    f = 50.0
    # Robot instance (choose between 'pin', 'rbdl' or 'ff6dof')
    robot = RobotBase('../models/ff6dof.urdf', fbase, 'pin')
    # Print robot model information
    robot.printModelInfo()

    # Joint configuration
    q = (0.75, -0.75, 1.57, -0.75, 0.75, -1.57)
    q = np.array([q]).transpose()
    if (fbase):
        # Add floating base configuration
        qb = np.array([[0., 0., 0., 1., 0., 0., 0.]]).transpose()
        q = np.vstack([qb, q])
        # Change base position
        q[0][0] = 0.2
        q[1][0] = 0.5
        q[2][0] = 0.4
        # Change base orientation
        t = np.pi/3
        q[3][0] = np.cos(t);
        q[4][0] = np.sin(t)
        #q[4][0] = np.sin(t)/np.sqrt(2.); q[5][0] = np.sin(t)/np.sqrt(2.)
    robot.updateJointConfig(q)

    # Print positions
    position1 = robot.model.linkPosition(robot.mlink['larm3'])
    position2 = robot.model.linkPosition(robot.mlink['lleg3'])
    print "Link larm3 position: ", position1.transpose()
    print "Link lleg3 position: ", position2.transpose()

    # Example of joint motion (however, no visualization)
    for k in range(200):
        q[robot.mjoint['larm_j1']] += 0.005
        q[robot.mjoint['larm_j2']] += 0.010
        q[robot.mjoint['larm_j3']] -= 0.015
        q[robot.mjoint['lleg_j1']] -= 0.005
        q[robot.mjoint['lleg_j2']] -= 0.010
        q[robot.mjoint['lleg_j3']] += 0.015
        robot.updateJointConfig(q)
