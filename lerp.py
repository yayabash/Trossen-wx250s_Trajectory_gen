import numpy as np
import sys

def lerp(tt,t1,t2,q1, q2):

    if (t2-t1<0):
        print('Error in lerp: t2 < t1')
        sys.exit(1)

    if (  np.linalg.norm(t2-t1)<0.1):
        print('Error in lerp: norm(t2-t1) too small')
        sys.exit(1)

    if (tt<t1):
        t = 0;
    elif(tt>t2):
        t = 1
    else:
        t = (tt-t1)/(t2-t1)

    # Normalize input quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute the dot product (cosine of the angle)
    cos_angle = np.dot(q1, q2)

    # If dot is negative, use -q2 to ensure the shortest path interpolation
    if cos_angle < 0.0:
        q2 = -q2
        cos_angle = -cos_angle

    #use linear interpolation
    q_lerp =  (1 - t) * q1 + t * q2
    q_lerp = q_lerp / np.linalg.norm(q_lerp)
    qd_lerp = -q1 + q2
    qdd_lerp = np.array([0,0,0,0])

    return q_lerp,qd_lerp,qdd_lerp
