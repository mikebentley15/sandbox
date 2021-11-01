'''
This function is all about computing the error vector between two states that
have both position and orientation.  This is to be used with optimization-based
or gradient-based solutions such as IK or resolved-rate control.

You would use computeError(T_cur, T_des) as your x_delta in such methods.
'''

import numpy as np

def matrixLog(Ae):       
    if abs(np.trace(Ae)+1) <= 0.0001:    
        phi = np.pi
        w = 1/np.sqrt(2*(1+Ae[2,2])) * np.array([Ae[0,2], Ae[1,2], 1+Ae[2,2]])
        skew = np.array([[0, -w[2], w[1]], 
                        [w[2], 0, -w[0]],
                        [-w[1], w[0], 0]])
        print("case 2 mat log")                
    else:
        phi = np.arccos(0.5*(np.trace(Ae)-1))
        skew = (1/(2*np.sin(phi)))*(Ae-np.transpose(Ae))
        print("case 3 mat log")
    return skew,phi

def computeError(currentTransform, desiredTransform):
    errorTransform = np.dot(desiredTransform, np.linalg.inv(currentTransform))
    linearError = errorTransform[:3,3:]
    skew,theta = matrixLog(errorTransform[:3,:3])
    if(theta == 0.0):
        rotationError = np.zeros((3,1))
    else:
        w_hat = skewToVector(skew)
        rotationError = w_hat * theta
    
    G = 1/theta*np.eye(3) - 1/2*skew + (1/theta - 1/2*(1/np.tan(theta/2)))   *(skew @ skew) 
    
    return np.concatenate((theta * G @ linearError, rotationError))


def skewToVector(skew):
    w = np.zeros((3,1))
    w[0,0] = skew[2,1]
    w[1,0] = skew[0,2]
    w[2,0] = skew[1,0]
    return w
