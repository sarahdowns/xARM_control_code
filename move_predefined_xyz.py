# Author: Sarah Downs
# This script uses move_safe_Corke.py as a base to move the xARM to the designated and uncommented
# predefined locations for testing
# THIS CODE WILL NOT AVOID OBSTACLES OR RUNNING INTO ITSELF!!!!!

import time
import move_safe_Corke as base
from xarm.wrapper import XArmAPI

ARM_IP = '192.168.1.225'

# CONFIGURATION DATABASE (Modify coordinates for specific workflows)
# A. TASK SPACE TARGETS: [X, Y, Z, Roll, Pitch, Yaw] (mm, degrees)
TASK_LOCATIONS = {
    "Task_1": [309.67, 576.54, 134.30, 0.0, 0.0, 0.0], # Cleaned placeholder to real TCP target
    "Task_2": [400.0, 200.0, 200.0, 0.0, 0.0, 0.0],
    "Task_3": [350.0, -100.0, 150.0, 180.0, 0.0, 0.0],
    "Pickup_Sled_TCP": [420, 240, 111.12, 67.88, -88.93, 109.37]
}

# B. JOINT SPACE TARGETS: [J1, J2, J3, J4, J5, J6, J7] (Degrees)
JOINT_LOCATIONS = {
    "Safe_Home": [-90.0, -0.0, 180.0, 0.0, 0.0, -90.0, 180.0],
    "Safe_Transit_WayPoint": [-32.8, 0.0, 0.0, 0.0, 0.0, -90.0, 0.0],
    "Setup_LateralY": [-32.8, -104.18, 106.55, 46.54, 21.26, 14.02, 88.84],
    "Setup_Forward": [-180.0, -0.0, 180.0, 0.0, 0.0, -90.0, 180.0],
    "Pickup_Sled_Joints": [-37.1, -105.2, 125.2, 27.8, 47.1, -11.92, 80.0]
}

# ==============================================================================
# ACTIVE RUN TARGETS (Uncomment exactly ONE location for your movement test)
# ==============================================================================
# Testing a Task Space Destination:
# ACTIVE_DESTINATION = TASK_LOCATIONS["Task_1"]
# ACTIVE_DESTINATION = TASK_LOCATIONS["Task_2"]

# Testing a Joint Space Destination:
ACTIVE_DESTINATION = JOINT_LOCATIONS["Safe_Home"]


# MOTION EXECUTION BLOCK
def execute_dynamic_destination(arm, robot_model, target):
    """
    Dynamically identifies if the target is Joint Space (7 elements) or 
    Task Space (6 elements), validates safety boundaries for Cartesian paths,
    and runs the appropriate movement command.
    """
    print("\n" + "="*50)
    print(" INITIATING POSITION DEPLOYMENT ROUTINE")
    print("="*50)
    
    # Check length to determine coordinate space
    target_len = len(target)
    
    # Case 1: Joint Space Movement (7-DoF Array)
    if target_len == 7:
        print(f"Detected JOINT SPACE Target Array:")
        print(f" -> { [round(j, 2) for j in target] } degrees")
        print(" -> Executing Joint-Space Trajectory...")
        
        arm.set_servo_angle(angle=target, speed=20, is_radian=False, wait=True)
        print("-> Target joints configuration reached successfully.")
        return True

    # Case 2: Task Space Movement (Cartesian Pose Array)
    elif target_len == 6:
        tgt_x, tgt_y, tgt_z, roll, pitch, yaw = target
        print(f"Detected TASK SPACE Cartesian Target:")
        print(f" -> X: {tgt_x:.2f}, Y: {tgt_y:.2f}, Z: {tgt_z:.2f}")
        
        # MATHEMATICAL SAFETY PRE-FLIGHT LAYER via move_safe_Corke
        if not base.is_within_bounds(tgt_x, tgt_y, tgt_z):
            print(f"!!! MOTION REJECTED: Target ({tgt_x}, {tgt_y}, {tgt_z}) mm violates SAFE_BOUNDS.")
            return False

        print(" -> Safety boundaries verified. Commanding physical hardware path...")
        
        # Executes a clean linear Cartesian path to the target zone
        arm.set_position(x=tgt_x, y=tgt_y, z=tgt_z, roll=roll, pitch=pitch, yaw=yaw, 
                         speed=30, wait=True)
        print("-> Target Cartesian pose reached successfully.")
        return True
        
    else:
        print(f"!!! ERROR: Invalid target array length ({target_len}). Must be 6 or 7 elements.")
        return False


# EXECUTIVE ENTRY LOOP
if __name__ == "__main__":
    with base.connect_xarm(ip=ARM_IP) as arm:
        # Load custom mathematical DH parameter verification configuration 
        robot_model = base.get_xarm7()

        # Apply standard hardware stack masses and static offsets
        print("\n--- Initializing Tool Center Point Parameters ---")
        arm.set_tcp_load(1.37, [0.0, 0.0, 70.0])
        arm.set_tcp_offset([0.0, 0.0, 197.0, 0.0, 0.0, 0.0])
        arm.set_state(0)
        time.sleep(0.5)

        # Log current physical coordinates prior to execution
        _, initial_joints = arm.get_servo_angle(is_radian=False)
        _, initial_cartesian = arm.get_position(is_radian=False)
        print(f"Starting Joint Position: {[round(j,1) for j in initial_joints]}")
        print(f"Starting TCP Position:   X={initial_cartesian[0]:.1f}, Y={initial_cartesian[1]:.1f}, Z={initial_cartesian[2]:.1f}")

        # Deploy directly to whichever single configuration target is uncommented above
        execute_dynamic_destination(arm, robot_model, target=ACTIVE_DESTINATION)
        
        print("\nTrajectory operation complete.")