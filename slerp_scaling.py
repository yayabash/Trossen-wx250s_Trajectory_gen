import numpy as np
import sys
from lerp_scaling import lerp_scaling

def slerp_scaling(tt,t1,t2,q1, q2):

    if (t2-t1<0):
        print('Error in slerp: t2 < t1')
        sys.exit(1)

    if (  np.linalg.norm(t2-t1)<0.1):
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

    # If quaternions are too close, use linear interpolation
    if cos_angle > 0.9995:
        #return q_lerp,qd_lerp,qdd_lerp
        q_lerp,qd_lerp,qdd_lerp = lerp_scaling(tt,t1,t2,q1, q2)
        return q_lerp,qd_lerp,qdd_lerp

    #parameter s such that position 0,1 velocity, acceleration are zero at the end points
    s = 6*t**5-15*t**4+10*t**3
    sd = 30*t**4 - 60*t**3 + 30*t**2
    sdd = 120*t**3 - 180*t**2 + 60*t

    #use spherical linear interpolation
    angle = np.arccos(cos_angle)
    sin_angle = np.sin(angle);

    sinv = 1/sin_angle;
    q_slerp = sinv*(q1*np.sin(angle*(1 - s)) + q2*np.sin(s*angle))
    qd_slerp = sd*sinv*(-q1*angle*np.cos(angle*(1 - s)) + q2*angle*np.cos(s*angle))
    qdd_slerp = (sd**2)*sinv*(-q1*angle**2*np.sin(angle*(1 - s)) - q2*angle**2*np.sin(s*angle)) \
                + sdd*sinv*(-q1*angle*np.cos(angle*(1 - s)) + q2*angle*np.cos(s*angle))

    return q_slerp,qd_slerp,qdd_slerp
