"""
Utilitary functions to help printing

"""

import sys
import numpy as np
from oscr import quaternionMult


def printJointLimits(jnames, qmin, qmax, dqmax):
    """
    Print joint names and joint limits

    """
    if (not( (len(jnames) == qmin.shape[0]) &
             (len(jnames) == qmax.shape[0]) &
             (len(jnames) == dqmax.shape[0]) )):
        print 'Joint names and joint limits size do not match!'
        return
    print 'Joint name\t qmin \t qmax \t dqmax'
    for i in xrange(len(jnames)):
        print jnames[i], '\t', qmin[i][0], '\t', qmax[i][0], '\t', dqmax[i][0]


def printLinkNameID(mapLinkNameID):
    """
    Print link names and link IDs

    """
    linkNames = mapLinkNameID.keys()
    linkIDs = mapLinkNameID.values()
    for i in xrange(len(linkNames)):
        print 'Link:', linkNames[i], ' - ID:', linkIDs[i]


def printLinkNameIDShort(mapLinkNameID):
    """
    Print link names and link IDs in short format (one line)

    """
    linkNames = mapLinkNameID.keys()
    linkIDs = mapLinkNameID.values()
    print '\nMap link_names: IDs'
    for i in xrange(len(linkNames)):
        customstr = '('+linkNames[i]+': '+str(linkIDs[i])+')'
        sys.stdout.write(customstr)
        if (i<len(linkNames)-1):
            sys.stdout.write(', ')
    print '\n'

def dictLink(mapLinkNameID):
    """
    Convert from C++ map to Python dictionary

    """
    m = dict()
    linkNames = mapLinkNameID.keys()
    linkIDs = mapLinkNameID.values()
    for i in xrange(len(linkNames)):
        m[linkNames[i]] = linkIDs[i]
    return m


def dictJoint(jointNames, hasFloatingBase):
    """
    Generate a map that associates joint name with joint ID. It increases the
    joint numbers by 7 if the robot has a floating base (to account for the
    base)

    """
    bias = 0
    if (hasFloatingBase):
        bias = 7
    m = dict()
    for i in xrange(len(jointNames)):
        m[jointNames[i]] = i+bias
    return m


def mprint(M, name="ans",eps=1e-15):
    """
    Matlab-style matrix print. Input  must be a numpy matrix

    """
    M = np.matrix(M)
    ncol = M.shape[1]
    NC = 6
    print name, " = "
    print

    Mmin = lambda M: M.min() if np.nonzero(M)[1].shape[1]>0 else M.sum()
    Mmax = lambda M: M.max() if np.nonzero(M)[1].shape[1]>0 else M.sum()
    Mm = Mmin(abs(M[np.nonzero(M)]))
    MM = Mmax(abs(M[np.nonzero(M)]))

    fmt = "% 10.3e" if Mm < 1e-5 or MM > 1e6 or MM / Mm > 1e3 else "% 1.5f"

    for i in range((ncol - 1) / NC + 1):
        cmin = i * 6
        cmax = (i + 1) * 6
        cmax = ncol if ncol < cmax else cmax
        print "Columns %s through %s" % (cmin, cmax - 1)
        print
        for r in range(M.shape[0]):
            sys.stdout.write("  ")
            for c in range(cmin, cmax):
                if abs(M[r,c])>eps: sys.stdout.write(fmt % M[r,c]  + "   ")
                else: sys.stdout.write(" 0"+" "*9)
            print
        print


def incPoseLocal(Pinit, dposition, dorient):
    """
    Increment in pose with respect to the local (current) frame.
        Pinit     - initial position and orientation (quaternion)
        dposition - increment in position
        dorient   - increment in orientation (angle[deg], axis)

    """
    Pf = Pinit.copy()
    # Position
    Pf[0][0] += dposition[0]
    Pf[1][0] += dposition[1]
    Pf[2][0] += dposition[2]
    # Orientation
    Qinit = Pinit[3:,:]
    angle2 = np.deg2rad(dorient[0])/2.0
    axis = np.array([dorient[1]]).transpose()
    axis = axis/np.linalg.norm(axis)
    dQ = np.matrix([[np.cos(angle2)],
                    [axis[0][0]*np.sin(angle2)],
                    [axis[1][0]*np.sin(angle2)],
                    [axis[2][0]*np.sin(angle2)]])
    Qf = quaternionMult(Qinit, dQ)  # With respect to local frame
    Pf[3:,:] = Qf
    return Pf


def incPoseGlobal(Pinit, dposition, dorient):
    """
    Increment in pose with respect to the global frame.
        Pinit - initial position and orientation (quaternion)
        dposition - increment in position
        dorient   - increment in orientation (angle[deg], axis)

    """
    Pf = Pinit.copy()
    # Position
    Pf[0][0] += dposition[0]
    Pf[1][0] += dposition[1]
    Pf[2][0] += dposition[2]
    # Orientation
    Qinit = Pinit[3:,:]
    angle2 = np.deg2rad(dorient[0])/2.0
    axis = np.array([dorient[1]]).transpose()
    axis = axis/np.linalg.norm(axis)
    dQ = np.matrix([[np.cos(angle2)],
                    [axis[0][0]*np.sin(angle2)],
                    [axis[1][0]*np.sin(angle2)],
                    [axis[2][0]*np.sin(angle2)]])
    Qf = quaternionMult(dQ, Qinit)  # With respect to global frame
    Pf[3:,:] = Qf
    return Pf


def incPosition(positionInitial, deltaPosition):
    """
    Increment in position.
        positionInitial - initial position (np array 3x1)
        deltaPosition - increment in position (np array 3x1)

    """
    Pf = positionInitial.copy()
    # Position
    Pf[0][0] += deltaPosition[0]
    Pf[1][0] += deltaPosition[1]
    Pf[2][0] += deltaPosition[2]
    return Pf


def quaternionFromAxisAngle(angle, axis):
    """
    Increment in pose with respect to the global frame.
        angle - double [deg]
        axis - (x,y,z) if not unitary, it will be internally normalized

    """
    angle2 = np.deg2rad(angle)/2.0
    ax = np.array([axis]).transpose()
    ax = ax/np.linalg.norm(ax)
    Q = np.matrix([[np.cos(angle2)],
                   [ax[0][0]*np.sin(angle2)],
                   [ax[1][0]*np.sin(angle2)],
                   [ax[2][0]*np.sin(angle2)]])
    return Q


def setPose(position, orientation):
    """
    Create a 7x1 array containing the pose [x,y,z,w,ex,ey,ez] containing a
    position and a quaternion
        position - (x,y,z)
        orientation - angle and axis (angle, (x,y,z))

    """
    q = quaternionFromAxisAngle(orientation[0], orientation[1])
    res = np.array([[position[0]], [position[1]], [position[2]],
                    [q[0]], [q[1]], [q[2]], [q[3]]])
    return res
