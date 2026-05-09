import numpy as np
import sys
from lerp import lerp

def slerp(tt,t1,t2,q1, q2):

    if (t2-t1<0):
        print('Error in slerp: t2 < t1')
        sys.exit(1)

    if (np.linalg.norm(t2-t1)<0.1):
        print('Error in slerp: norm(t2-t1) too small')
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


    if cos_angle > 0.9995:
        q_lerp,qd_lerp,qdd_lerp = lerp(tt,t1,t2,q1, q2)
        return q_lerp,qd_lerp,qdd_lerp

    #use spherical linear interpolation
    angle = np.arccos(cos_angle)
    sin_angle = np.sin(angle);
    q_slerp = (1/sin_angle)*(np.sin((1-t)*angle)*q1 + np.sin(t*angle)*q2)
    qd_slerp = (1/sin_angle)*(-np.cos((1-t)*angle)*q1 + np.cos(t*angle)*q2)*angle
    qdd_slerp = (1/sin_angle)*(np.sin((1-t)*angle)*q1 + np.sin(t*angle)*q2)*angle*angle

    return q_slerp,qd_slerp,qdd_slerp
