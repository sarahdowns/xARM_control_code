import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from xarm.wrapper import XArmAPI
import time

import move_safe_Corke as base

# Use your confirmed IP
arm_ip = '192.168.1.225'
arm = XArmAPI(arm_ip)

CUSTOM_HOME = [0, 0, 0, 0, 0, 0, 0] 

def get_xarm7():
    """
    Manually creates the xArm7 model using Modified DH parameters.
    Dimensions are in meters.
    """
    # Modified DH Parameters: [alpha, a, theta, d]
    # alpha = link twist, a = link length, d = link offset
    links = [
        rtb.RevoluteMDH(alpha=0,      a=0,      d=0.267),
        rtb.RevoluteMDH(alpha=-np.pi/2, a=0,      d=0),
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0,      d=0.293),
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0.0525, d=0),
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0.0775, d=0.3425),
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0,      d=0),
        rtb.RevoluteMDH(alpha=-np.pi/2, a=0.076,  d=0.097)
    ]
    
    robot = rtb.DHRobot(links, name="xArm7")
    
    # Add the official joint limits (in radians)
    # Based on the xarm7_joint_limits.xacro
    robot.qlim = np.array([
        [-2.0*np.pi, 2.0*np.pi], # J1 (Limited by cable usually, but math allows 360)
        [-2.059,     2.059],     # J2
        [-2.0*np.pi, 2.0*np.pi], # J3
        [-0.1919,    3.927],     # J4
        [-2.0*np.pi, 2.0*np.pi], # J5
        [-1.6929,    3.14159],   # J6
        [-2.0*np.pi, 2.0*np.pi]  # J7
    ]).T
    return robot

def go_custom_home(arm):
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    
    print(f"Moving to custom home: {CUSTOM_HOME}")
    arm.set_servo_angle(angle=CUSTOM_HOME, speed=30, wait=True)

# 1. Reset and Enable
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)  # Position mode
arm.set_state(state=0)  # Ready state
time.sleep(1) # Give it a second to engage the brakes

# 2. Get current position to use as a baseline
code, angles = arm.get_servo_angle()

if code == 0:
    print(f"Current angles: {angles}")
    
    # Let's move Joint 1 (the base) by 10 degrees
    # We take the current angle of joint 1 and add 10
    new_angle_j1 = angles[0]        # + is CCW 
    new_angle_j2 = angles[1] 	    # + is CCW
    new_angle_j3 = angles[2] 
    new_angle_j4 = angles[3] 
    new_angle_j5 = angles[4] 
    new_angle_j6 = angles[5]
    new_angle_j7 = angles[6]
    
    print(f"Moving joint 1 to: {new_angle_j1}")
    print(f"Moving joint 2 to: {new_angle_j2}")
    print(f"Moving joint 3 to: {new_angle_j3}")
    print(f"Moving joint 4 to: {new_angle_j4}")
    print(f"Moving joint 5 to: {new_angle_j5}")
    print(f"Moving joint 6 to: {new_angle_j6}")
    print(f"Moving joint 7 to: {new_angle_j7}")

    
    # set_servo_angle parameters: 
    # servo_id=1 (Joint 1), angle=new_angle_j1, speed=20, wait=True
    arm.set_servo_angle(servo_id=1, angle=new_angle_j1, speed=20, is_radian=False, wait=True)
    arm.set_servo_angle(servo_id=2, angle=new_angle_j2, speed=20, is_radian=False, wait=True)
    arm.set_servo_angle(servo_id=3, angle=new_angle_j3, speed=20, is_radian=False, wait=True)
    arm.set_servo_angle(servo_id=4, angle=new_angle_j4, speed=20, is_radian=False, wait=True)
    arm.set_servo_angle(servo_id=5, angle=new_angle_j5, speed=20, is_radian=False, wait=True)
    arm.set_servo_angle(servo_id=6, angle=new_angle_j6, speed=20, is_radian=False, wait=True)
    arm.set_servo_angle(servo_id=7, angle=new_angle_j7, speed=20, is_radian=False, wait=True)

    # go_custom_home(arm)
    base.print_current_state(arm)

    print("Movement complete!")
else:
    print(f"Failed to get angles, code: {code}")

#from xarm.wrapper import XArmAPI
#print([method for method in dir(XArmAPI) if 'box' in method or 'fence' in method or 'confine' in method])

#robot_model = get_xarm7()
#print("Manual xArm7 Model Created Successfully!")
#print(robot_model)

arm.disconnect()