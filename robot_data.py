import numpy as np
import utility as ram 

class Robot:
    class Body:
        def __init__(self, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
            self.parent = parent
            self.name = name
            self.pos = np.array(pos)
            self.quat = np.array(quat)
            self.ipos = np.array(ipos)
            self.iquat = np.array(iquat)
            self.mass = mass
            self.inertia = np.array(inertia)
            self.joint_axis = np.array(joint_axis)
            self.joint_range = np.array(joint_range)

    def add_body(self, body_id, parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range):
        self.body[body_id] = Robot.Body(parent, name, pos, quat, ipos, iquat, mass, inertia, joint_axis, joint_range)

    class Params:
        def __init__(self):
            # Base body has no quat in XML, so use identity base orientation
            self.base_quat = ram.quat_normalize(np.array([1, 0, 0, 0]))
            # Site info from <site name="attachment_site" pos="0.1 0 0" quat="1 0 0 0">
            self.end_eff_pos_local = np.array([0.1, 0, 0])
            self.end_eff_quat_local = ram.quat_normalize(np.array([1, 0, 0, 0]))
            self.gravity = np.array([0,0,-9.81])

    def __init__(self):
        self.body = {}
        self.params = Robot.Params()

robot = Robot()

# Values extracted from wx250s.xml
# Body 1: Shoulder (Waist joint)
robot.add_body(1, parent='base_link', name='shoulder_link', 
               pos=[0, 0, 0.072], quat=[1, 0, 0, 0], 
               ipos=[2.23482e-05, 4.14609e-05, 0.0066287], iquat=[0.0130352, 0.706387, 0.012996, 0.707586],
               mass=0.480879, inertia=[0.000588946, 0.000555655, 0.000378999],
               joint_axis=[0, 0, 1], joint_range=[-3.14158, 3.14158])

# Body 2: Upper Arm (Shoulder joint)
robot.add_body(2, parent='shoulder_link', name='upper_arm_link', 
               pos=[0, 0, 0.03865], quat=[1, 0, 0, 0], 
               ipos=[0.0171605, 2.725e-07, 0.191323], iquat=[0.705539, 0.0470667, -0.0470667, 0.705539],
               mass=0.430811, inertia=[0.00364425, 0.003463, 0.000399348],
               joint_axis=[0, 1, 0], joint_range=[-1.88496, 1.98968])

# Body 3: Upper Forearm (Elbow joint)
robot.add_body(3, parent='upper_arm_link', name='upper_forearm_link', 
               pos=[0.04975, 0, 0.25], quat=[1, 0, 0, 0], 
               ipos=[0.107963, 0.000115876, 0], iquat=[0.000980829, 0.707106, -0.000980829, 0.707106],
               mass=0.234589, inertia=[0.000888, 0.000887807, 3.97035e-05],
               joint_axis=[0, 1, 0], joint_range=[-2.14675, 1.6057])

# Body 4: Lower Forearm (Forearm roll)
robot.add_body(4, parent='upper_forearm_link', name='lower_forearm_link', 
               pos=[0.175, 0, 0], quat=[1, 0, 0, 0], 
               ipos=[0.0374395, 0.00522252, 0], iquat=[-0.0732511, 0.703302, 0.0732511, 0.703302],
               mass=0.220991, inertia=[0.0001834, 0.000172527, 5.88633e-05],
               joint_axis=[1, 0, 0], joint_range=[-3.14158, 3.14158])

# Body 5: Wrist (Wrist angle joint)
robot.add_body(5, parent='lower_forearm_link', name='wrist_link', 
               pos=[0.075, 0, 0], quat=[1, 0, 0, 0], 
               ipos=[0.04236, -1.0663e-05, 0.010577], iquat=[0.608721, 0.363497, -0.359175, 0.606895],
               mass=0.084957, inertia=[3.29057e-05, 3.082e-05, 2.68343e-05],
               joint_axis=[0, 1, 0], joint_range=[-1.74533, 2.14675])

# Body 6: Gripper (Wrist rotate joint)
robot.add_body(6, parent='wrist_link', name='gripper_link', 
               pos=[0.065, 0, 0], quat=[1, 0, 0, 0], 
               ipos=[0.0325296, 4.2061e-07, 0.0090959], iquat=[0.546081, 0.419626, 0.62801, 0.362371],
               mass=0.110084, inertia=[0.00307592, 0.00307326, 0.0030332],
               joint_axis=[1, 0, 0], joint_range=[-3.14158, 3.14158])

# Final normalization
for body_id, body in robot.body.items():
    body.quat = ram.quat_normalize(body.quat)
    body.iquat = ram.quat_normalize(body.iquat)

#print(robot.body[1].parent)
#print(robot.body[1].name)
#print(robot.body[1].pos)