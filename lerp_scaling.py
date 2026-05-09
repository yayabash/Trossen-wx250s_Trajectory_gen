import numpy as np
import sys

def lerp_scaling(tt,t1,t2,q1, q2):


    if (t2-t1<0):
        print('Error in lerp: t2 < t1')
        sys.exit(1)

    if (np.linalg.norm(t2-t1)<0.1):
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

    #parameter s such that position 0,1 velocity, acceleration are zero at the end points
    s = 6*t**5-15*t**4+10*t**3
    sd = 30*t**4 - 60*t**3 + 30*t**2
    sdd = 120*t**3 - 180*t**2 + 60*t

    q_lerp = (1-s)*q1 + s*q2
    qd_lerp = -sd*q1+ sd*q2
    qdd_lerp = -sdd*q1 + sdd*q2

    return q_lerp,qd_lerp,qdd_lerp
