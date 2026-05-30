# File name: move_safe_Corke.py
# This code acts as a "base" for all xARM code.

import time
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from xarm.wrapper import XArmAPI
from contextlib import contextmanager

# Global Configuration
ARM_IP = '192.168.1.225'

# Safe boundaries in Millimeters [X_min, X_max, Y_min, Y_max, Z_min, Z_max]
SAFE_BOUNDS = [100, 730, -700, 600, 100, 999]

# Custom Safe Home in degrees [J1, J2, J3, J4, J5, J6, J7]
SAFE_HOME_JOINTS = [90.0, 0.0, 0.0, 0.0, 0.0, -90.0, 0.0]	# With gripper
# SAFE_HOME_JOINTS = [0.0, 120.0, 0.0, 180.0, 0.0, -90.0, 0.0]	# Test gripper, disconnected
# SAFE_HOME_JOINTS = [0.0, 90.0, 0.0, 180.0, 0.0, 0.0, 0.0]	# straight in +X
# SAFE_HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]	# Without gripper
# SAFE_HOME_JOINTS = [0.0, 0.0, 0.0, 180.0, 0.0, 0.0, 0.0]	# Point Straight up


@contextmanager
def connect_xarm(ip=ARM_IP):
    """Handles connection context and guarantees safe shutdown."""
    print(f"Connecting to hardware xArm at {ip}...")
    arm = XArmAPI(ip)
    try:
        arm.clean_error()
        arm.motion_enable(True)
        arm.set_mode(0)
        arm.set_state(0)
        
        arm.set_fence_mode(True)
        print("Hardware spatial fence active via controller.")
        yield arm
    except Exception as e:
        print(f"!!! CRITICAL EXCEPTION DURING ROBOT OPERATION: {e}")
        raise e
    finally:
        arm.disconnect()
        print("Disconnected safely from physical xArm hardware.")

def send_safe_home(arm, speed=20):
    """Safely moves the arm back to the pre-defined custom resting joint configuration."""
    print("\n" + "="*40)
    print("         RETURNING TO SAFE HOME         ")
    print(f" Target Joints : {SAFE_HOME_JOINTS} degrees")
    print("="*40)
    
    confirm = input("Proceed with homing? (y/n): ")
    if confirm.lower() == 'y':
        print(f"Moving to custom home at speed {speed}...")
        # Moving by joint angles is safer for homing than task space
        arm.set_servo_angle(angle=SAFE_HOME_JOINTS, speed=speed, is_radian=False, wait=True)
        print("Home position reached.")
        return True
    else:
        print("Homing cancelled.")
        return False
    
def send_linear_move(arm, robot_model, x_mm, y_mm, z_mm, roll=0, pitch=0, yaw=0, speed=30, mask=None):
    """Executes a perfectly straight Cartesian line to the target after safety checks."""
    
    # 1. Boundary & Safety Checks (Same as your standard move)
    if not is_within_bounds(x_mm, y_mm, z_mm):
        print(f"!!! REJECTED: Target ({x_mm}, {y_mm}, {z_mm}) mm violates SAFE_BOUNDS.")
        return False

    _, current_deg = arm.get_servo_angle(is_radian=False)
    q_current = np.radians(current_deg)

    tx, ty, tz = x_mm/1000.0, y_mm/1000.0, z_mm/1000.0
    T_target = SE3(tx, ty, tz) * SE3.RPY(np.radians([roll, pitch, yaw]))
    
    sol = robot_model.ikine_LM(T_target, q0=q_current, mask=mask)
    
    if not sol.success:
        print("!!! REJECTED: IK Solver failed to find a mathematical path.")
        return False
        
    if not are_joints_within_limits(sol.q, robot_model.qlim):
        print("!!! REJECTED: Target hits a physical hardware joint limit.")
        return False
        
    w = robot_model.manipulability(sol.q)
    if w < 0.01:
        print(f"!!! CRITICAL CANCEL: Singularity zone (Manipulability: {w:.4f}).")
        return False
        
    print("\n" + "="*40)
    print("      LINEAR MOTION PREVIEW (STRAIGHT LINE)      ")
    print(f" Target Location : X:{x_mm}, Y:{y_mm}, Z:{z_mm} mm")
    print(f" Manipulability  : {w:.4f} (Safe Area)")
    print("="*40)
    
    confirm = input("Proceed with linear hardware execution? (y/n): ")
    if confirm.lower() == 'y':
        print(f"Moving linearly to target at speed {speed} mm/s...")
        # CRITICAL DIFFERENCE: set_position forces a straight Cartesian line
        arm.set_position(x=x_mm, y=y_mm, z=z_mm, roll=roll, pitch=pitch, yaw=yaw, speed=speed, wait=True)
        print("Linear move complete.")
        return True
    else:
        print("Move cancelled.")
        return False
    
def init_robotiq_gripper(arm):
    """
    Initializes and activates a Robotiq 2F gripper connected via the xArm tool port.
    Robotiq grippers require a specific reset and activation sequence on every boot.
    """
    print("\nInitializing Robotiq Gripper...")
    
    # 1. Reset the gripper's internal controller
    code, _ = arm.robotiq_reset()
    if code != 0:
        print(f"!!! ERROR: Failed to reset Robotiq gripper. Code: {code}")
        return False
        
    # 2. Activate the gripper (It will slowly open and close to calibrate itself)
    print("Activating gripper (Stand back, it will calibrate its stroke)...")
    code, _ = arm.robotiq_set_activate(wait=True)
    if code != 0:
        print(f"!!! ERROR: Failed to activate Robotiq gripper. Code: {code}")
        return False
        
    print("Robotiq Gripper Active and Ready.")
    return True

def set_safe_tcp_payload(arm, weight_kg, center_of_mass_mm):
    """
    Safely sets the TCP payload and center of gravity for accurate physics and F/T readings.
    """
    # Safety Layer: Prevent catastrophic typos (xArm7 max payload is ~3.5kg)
    MAX_PAYLOAD_KG = 3.5
    
    if weight_kg > MAX_PAYLOAD_KG or weight_kg < 0:
        print(f"!!! REJECTED: Payload {weight_kg}kg violates safe limits (0 - {MAX_PAYLOAD_KG}kg).")
        return False
        
    print("\n" + "-"*35)
    print("      UPDATING TCP PAYLOAD     ")
    print(f" Mass : {weight_kg} kg")
    print(f" CoM  : {center_of_mass_mm} mm")
    print("-" * 35)

    # 1. Send the payload and Center of Mass to the controller
    code = arm.set_tcp_load(weight_kg, center_of_mass_mm)
    
    if code == 0:
        # 2. Give the internal physics engine half a second to recalculate gravity
        time.sleep(0.5)
        # 3. Ensure the arm state is active and ready
        arm.set_state(0)
        print("Payload updated successfully.")
        return True
    else:
        print(f"!!! ERROR: Failed to set TCP payload. Code: {code}")
        return False
    
def read_active_tcp_config(arm):
    """
    Reads and prints the currently active TCP payload and offset directly from the controller.
    Returns:
        success (bool): True if successfully read.
        weight (float): Payload mass in kg.
        com (list): Center of mass [X, Y, Z] in mm.
        offset (list): TCP Offset [X, Y, Z, Roll, Pitch, Yaw].
    """
    # In the xArm SDK, these are live properties, not functions!
    load_data = arm.tcp_load
    offset_data = arm.tcp_offset
    
    if load_data is not None and offset_data is not None:
        # load_data is structured as: [weight, [x, y, z]]
        weight = load_data[0]
        com = load_data[1]  # This already contains the [x, y, z] list
        
        # offset_data is [x, y, z, roll, pitch, yaw]
        offset = offset_data 
        
        print("\n" + "-"*35)
        print("      ACTIVE TCP CONFIGURATION     ")
        print(f" Payload Mass  : {weight} kg")
        print(f" Center of Mass: {[round(c, 2) for c in com]} mm")
        print(f" TCP Offset    : {[round(o, 2) for o in offset]}")
        print("-" * 35)
        
        return True, weight, com, offset
    else:
        print("!!! ERROR: Failed to read TCP config from properties.")
        return False, None, None, None

def set_robotiq_position(arm, position, speed=255, force=25, wait=True):
    """
    Actuates the Robotiq gripper.
    :param position: 0 (fully open) to 255 (fully closed).
    :param speed: 0 (slowest) to 255 (fastest).
    :param force: 0 (weakest) to 255 (strongest).
    """
    print(f"Moving gripper to position {position} (Speed: {speed}, Force: {force})...")
    code, _ = arm.robotiq_set_position(position, speed=speed, force=force, wait=wait)
    
    if code == 0:
        print("Gripper move complete.")
        return True
    else:
        print(f"!!! ERROR: Gripper move failed. Code: {code}")
        return False

def print_current_state(arm):
    """Fetches and prints the current Joint Angles and Cartesian TCP position. Prints
    status code (_) and data []. Tuple unpacking"""
    _, angles = arm.get_servo_angle(is_radian=False)
    _, pos = arm.get_position(is_radian=False)
    
    print("\n" + "-"*35)
    print("        CURRENT ROBOT STATE        ")
    print(f" Joint Angles : {[round(a, 2) for a in angles]}")
    # pos contains [X, Y, Z, Roll, Pitch, Yaw]
    print(f" TCP Position : {[round(p, 2) for p in pos]}")
    print("-"*35 + "\n")

def get_xarm7():
    """Returns the custom 7-DOF Modified DH mathematical model with exact limits."""
    links = [
        rtb.RevoluteMDH(alpha=0,        a=0,      d=0.267,  m=2.177, r=[0.00015, 0.02724, -0.01357]),                    
        rtb.RevoluteMDH(alpha=-np.pi/2, a=0,      d=0,      m=1.716, r=[0.00022, -0.12470, 0.01890]),                       
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0,      d=0.293,  m=1.485, r=[0.04600, -0.02230, -0.00847]), 
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0.0525, d=0,      m=1.574, r=[0.06975, -0.11250, 0.01320]),
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0.0775, d=0.3425, m=1.209, r=[-0.00035, 0.01760, -0.02840]),
        rtb.RevoluteMDH(alpha=np.pi/2,  a=0,      d=0,      m=1.214, r=[0.06365, 0.03084, 0.02170]),     
        rtb.RevoluteMDH(alpha=-np.pi/2, a=0.076,  d=0.097,  m=0.170, r=[0.00000, -0.00677, -0.01098])
    ]
    
    robot = rtb.DHRobot(links, name="xArm7")
    
    # Official joint limits (in radians)
    robot.qlim = np.array([
        [-2.0*np.pi, 2.0*np.pi], 
        [-2.059,     2.059],     
        [-2.0*np.pi, 2.0*np.pi], 
        [-0.1919,    3.927],     
        [-2.0*np.pi, 2.0*np.pi], 
        [-1.6929,    3.14159],   
        [-2.0*np.pi, 2.0*np.pi]  
    ]).T
    
    return robot

def is_within_bounds(x, y, z):
    """Validates if a millimeter coordinate is inside the software safe zone."""
    x_min, x_max, y_min, y_max, z_min, z_max = SAFE_BOUNDS
    return (x_min <= x <= x_max) and (y_min <= y <= y_max) and (z_min <= z <= z_max)

def are_joints_within_limits(q, qlim):
    """Explicitly verifies that all calculated IK angles stay within mechanical bounds."""
    # qlim shape is (2, 7) -> row 0 is min, row 1 is max
    for idx, joint_angle in enumerate(q):
        j_min = qlim[0, idx]
        j_max = qlim[1, idx]
        if not (j_min <= joint_angle <= j_max):
            print(f"!!! JOINT LIMIT VIOLATION: Joint {idx+1} calculated at {joint_angle:.4f} rad. "
                  f"Allowed Range: [{j_min:.4f}, {j_max:.4f}]")
            return False
    return True

def send_task_space_move(arm, robot_model, x_mm, y_mm, z_mm, roll=0, pitch=0, yaw=0, speed=20, mask=None):
    """Core defensive movement function with strict Cartesian and Joint-limit filtering."""
    # Safety Layer 1: Cartesian Bounding Box Check
    if not is_within_bounds(x_mm, y_mm, z_mm):
        print(f"!!! MOVE REJECTED: Target ({x_mm}, {y_mm}, {z_mm}) mm violates software boundaries {SAFE_BOUNDS}.")
        return False

    # Seed numerical IK solver using active encoder feedback
    _, current_deg = arm.get_servo_angle(is_radian=False)
    q_current = np.radians(current_deg)

    tx, ty, tz = x_mm/1000.0, y_mm/1000.0, z_mm/1000.0
    T_target = SE3(tx, ty, tz) * SE3.RPY(np.radians([roll, pitch, yaw]))
    
    # Solve Inverse Kinematics WITH the mask
    # If mask is None, the solver defaults to strictly enforcing all 6 DOF.
    sol = robot_model.ikine_LM(T_target, q0=q_current, mask=mask)
    
    if not sol.success:
        print("!!! MOVE REJECTED: IK Solver failed to find a mathematical configuration path.")
        return False
        
    # Safety Layer 2: Explicit Mechanical Joint Limit Verification
    if not are_joints_within_limits(sol.q, robot_model.qlim):
        print("!!! MOVE REJECTED: Calculated configuration path hits a physical hardware joint limit.")
        return False
        
    # Safety Layer 3: Mathematical Singularity Check
    w = robot_model.manipulability(sol.q)
    if w < 0.01:
        print(f"!!! CRITICAL CANCEL: Target falls inside a low-dexterity singularity zone (Manipulability: {w:.4f}).")
        return False
        
    # All safety validation checks passed
    target_deg = np.degrees(sol.q)
    check_pos = robot_model.fkine(sol.q).t * 1000 
    
    print("\n" + "="*40)
    print("      SAFE TASK SPACE MOTION PREVIEW      ")
    print(f" Target Location : X:{x_mm}, Y:{y_mm}, Z:{z_mm} mm")
    print(f" Solver Check XYZ: X:{check_pos[0]:.1f}, Y:{check_pos[1]:.1f}, Z:{check_pos[2]:.1f} mm")
    print(f" Mask Applied    : {mask if mask else '[1, 1, 1, 1, 1, 1] (Strict)'}")
    print(f" Manipulability  : {w:.4f} (Safe Area)")
    print("="*40)
    
    confirm = input("Proceed with hardware execution? (y/n): ")
    if confirm.lower() == 'y':
        print(f"Moving to target at speed {speed} mm/s...")
        arm.set_servo_angle(angle=target_deg, speed=speed, wait=True)
        print("Move complete.")
        return True
    else:
        print("Move explicitly cancelled by user.")
        return False