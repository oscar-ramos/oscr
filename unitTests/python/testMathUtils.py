from oscr.oscr import *
import numpy as np

v1 = np.array([[0.5],[1.0],[1.5]])
s1 = skew(v1)
print 'vector:\n', v1, '\nskew equivalent:\n', s1

Rx = rotationMatrix('x', np.pi/3.)
Ry = rotationMatrix('y', np.pi/3.)
Rz = rotationMatrix('z', np.pi/3.)
print 'Rx:\n',Rx, '\nRy:\n', Ry, '\nRz:\n', Rz
Rx2 = rotationMatrix(np.matrix([[1.],[0.],[0.]]), np.pi/3.)
print 'Rx (explicit axis):\n', Rx2

# Check RPY
vals = tuple(np.linspace(-4,4,100))
rpy = np.array([[0.],[0.],[0.]])
rin = []; rout = []
for i in vals:
    rpy[1][0] = i
    R = RPYToRotation(rpy)
    rpy_out = rotationToRPY(R)
    rin.append(rpy[1][0])
    rout.append(rpy_out[1][0])

if False:
    import matplotlib.pyplot as plt
    plt.plot(rin,rout)
    plt.show()

# Check quaternion
rotationToQuaternion(Ry)
rotationToQuaternion(Rz)
