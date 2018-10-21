from optparse import OptionParser
import numpy as np

from oscr.oscr import RobotModelPin, RobotModelRbdl, KineTaskPose, OSIKSolverNS
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

    # Link names and IDs
    linkID = dictLink(robot.mapLinkNamesIDs())
    
    # Joint configuration with all values to zero.
    q = np.zeros(robot.ndof())
    # If there is a floating base, then set a valid null quaternion
    if (robot.hasFloatingBase()):
        q[3]=1.0
    # Set the current joint configuration
    robot.updateJointConfig(q)
    # Get the current joint configuration
    qcurrent = robot.getJointConfig()

    # Definition of a task for the arm (keep) and a task for the leg
    taskarm = KineTaskPose(robot, linkID["larm3"], "position")
    taskleg = KineTaskPose(robot, linkID["lleg3"], "position")
    taskarm.setGain(1.0)
    taskleg.setGain(1.0)

    # Desired value for the arm (keep it)
    xarm = np.zeros((taskarm.getTaskDim(), 1))
    taskarm.getSensedValue(xarm)
    taskarm.setDesiredValue(xarm)
    # Desired value for the leg (move it)
    xleg = np.zeros((taskleg.getTaskDim(), 1))
    taskleg.getSensedValue(xleg)
    xleg[0] = xleg[0] + 0.1
    xleg[1] = xleg[1] + 0.1
    xleg[2] = xleg[2] + 0.1
    taskleg.setDesiredValue(xleg)
    
    # Solver
    dt = 0.010
    solver = OSIKSolverNS(robot, q, dt)

    # Add tasks to the solver
    solver.pushTask(taskarm)
    solver.pushTask(taskleg)

    qdes = np.copy(q)
    error = np.zeros((taskleg.getTaskDim(), 1))
    taskleg.getDerivError(error)
    error = taskleg.getError()

    i=0; time = 0.0
    while (np.linalg.norm(error)>0.0005):
        solver.getPositionControl(q, qdes)
        robot.updateJointConfig(q)
        q = np.copy(qdes)
        error = taskleg.getError()
        # Print on screen
        if (i%50==0):
            print i, " - error norm: ", np.linalg.norm(error)
        i = i+1
