import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from xarm.wrapper import XArmAPI
import matplotlib
import sys

# Setup xArm
arm_ip = '192.168.1.225'
arm = XArmAPI(arm_ip)

# Standard units here are mm for xArm SDK
SAFE_BOUNDS = [170, 700, 10, 400, 300, 600]

def setup_safety():
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    
    # In SDK 1.17.6, the hardware boundary is enabled like this:
    # Note: Ensure SAFE_BOUNDS is [x_min, x_max, y_min, y_max, z_min, z_max]
    arm.set_fence_mode(True) 
    
    print(f"Hardware Fence Active via set_fence_mode")
    print(f"Software Safety Logic Active (Boundaries: {SAFE_BOUNDS} mm)")

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

def move_to_cartesian_safe(x_mm, y_mm, z_mm, roll=0, pitch=0, yaw=0):
    # Get current state for IK seed
    _, current_deg = arm.get_servo_angle(is_radian=False)
    q_current = np.radians(current_deg)

    # Convert mm to Meters for the Toolbox math
    tx, ty, tz = x_mm/1000.0, y_mm/1000.0, z_mm/1000.0

    # 1. Prepare Target Pose (Meters)
    T_target = SE3(tx, ty, tz) * SE3.RPY(np.radians([roll, pitch, yaw]))
    
    # 2. Solve IK
    sol = robot_model.ikine_LM(T_target, q0=q_current)
    
    if sol.success:   
        w = robot_model.manipulability(sol.q)
        if w < 0.01:  # Threshold for "Danger Zone"
            print(f"!!! CRITICAL: Move cancelled. Manipulability {w:.4f} is too low.")
            return # Exit the function before asking to move
        target_deg = np.degrees(sol.q)
        
        print("\n" + "="*30)
        print("MOVE PREVIEW (SAFE)")
        print(f"Target XYZ: {x_mm}, {y_mm}, {z_mm} mm")
        print(f"Manipulability: {w:.4f}")
        print("="*30)

        # Verify FK of the IK solution (Fixed Indentation)
        check_pos = robot_model.fkine(sol.q).t * 1000 
        print(f"Toolbox Predicted Final XYZ: {check_pos}")
        
        confirm = input("Proceed with move? (y/n): ")
        if confirm.lower() == 'y':
            arm.set_servo_angle(angle=target_deg, speed=30, wait=True)
            print("Move complete.")
        else:
            print("Move cancelled.")

# --- Execute ---
setup_safety()

# Use the manual DH model we built (reliable) instead of the URDF file
robot_model = get_xarm7()

print(f"Robot Model '{robot_model.name}' ready for IK calculations.")

# This is inside your 200-800mm fence.
move_to_cartesian_safe(400, 0, 300)

arm.disconnect()