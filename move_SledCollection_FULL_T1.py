# Author: Sarah Downs
# File name:

# Multi-Stage Automated Assembly Routine
# Phase 0: Grip sled and move into position
# Phase 1: Probe Search (+Y)
# Phase 2: Retract (-Y)
# Phase 3: Lateral Shift (+X)
# Phase 4: Force-Guarded Insertion (+Y)

import sys

import move_safe_Corke as base
import time
import math

ARM_IP = '192.168.1.225'

STATIONARY_PROBE_BASE_XYZ = [321, 625, 128.8]
Socket_X = 409
DIST_PROBE_SOCKET_X = Socket_X - 321
    # Socket = 408.8, 565.9, 128.3  7/9/26
RETREAT_DIST_Y = 60.0
MAX_SAFE_Y = 600.0  # Max permitted Base Y travel based on Corke safety bounds
FORCE_LIMIT_N = 12.0
SPEED = 15

# Which F/T sensor axis experiences the insertion force? 
# 0 = Tool X, 1 = Tool Y, 2 = Tool Z
FT_AXIS_INDEX = 2  

# --- PHASE 0 CONFIGURATION ---
# Waypoints can be 7-element (Joint angles in Degrees) OR 6-element (Cartesian XYZ RPY in mm/Degrees)
PHASE_0_WAYPOINTS = [
    #[-31.92, -97.44, 98.65, 35.28, 25.24, 4.16, 75.2],      # Starting place (Joint)
    [385, 385, 150, 88.81, -89.2, 88.82],                    # Starting place (Cartesian adjustment)
    [359.38, 342.71, 126.63, 94.45, -86.03, 84.16],         # Pick up the sled (cartesian), move back in X 
    [-23.33, -104.27, 112.47, 35.04, 25.29, 10.28, 87.55],   # Joint angle adjustment
    [321, 342.71, 126.63, 94.45, -86.03, 84.16], 
    [321, 400, 126.63, 94.45, -86.03, 84.16]           # Setup for lateral move in Y
]
PHASE_0_WAIT_SEC = 1.0

def execute_multistage_insertion(arm):
    print("\n" + "="*50)
    print(" INITIATING 5-PHASE MULTI-STAGE INSERTION")
    print("="*50)

    # ---------------------------------------------------------
    # PHASE 0: PRE-SEARCH STAGING
    # ---------------------------------------------------------
    print("\n--- PHASE 0: Pre-Search Staging ---")
    arm.set_state(0)
    
    total_waypoints = len(PHASE_0_WAYPOINTS)
    for i, target in enumerate(PHASE_0_WAYPOINTS):
        target_len = len(target)
        
        # Case 1: Joint Space Movement (7 elements)
        if target_len == 7:
            print(f"Moving to Waypoint {i+1}/{total_waypoints} (Joint Space): {target}")
            arm.set_servo_angle(angle=target, speed=SPEED, is_radian=False, wait=True)
            
        # Case 2: Cartesian Task Space Movement (6 elements)
        elif target_len == 6:
            tgt_x, tgt_y, tgt_z, roll, pitch, yaw = target
            print(f"Moving to Waypoint {i+1}/{total_waypoints} (Cartesian Space): X:{tgt_x:.1f}, Y:{tgt_y:.1f}, Z:{tgt_z:.1f}")
            
            # Pre-flight mathematical safety check
            if not base.is_within_bounds(tgt_x, tgt_y, tgt_z):
                print(f"!!! HALTING: Waypoint {i+1} violates SAFE_BOUNDS. Aborting sequence.")
                return
            
            arm.set_position(x=tgt_x, y=tgt_y, z=tgt_z, roll=roll, pitch=pitch, yaw=yaw, speed=30, wait=True)
            
        else:
            print(f"!!! HALTING: Waypoint {i+1} has invalid length ({target_len}). Must be 6 or 7 elements.")
            return

        print(f"  -> Waiting {PHASE_0_WAIT_SEC} seconds...")
        time.sleep(PHASE_0_WAIT_SEC)

        # GRIPPER ACTUATION AFTER WAYPOINT 2
        if i == 2:  # i == 1 is the second waypoint (0-indexed)
            print("  -> Actuating Robotiq Gripper...")
            time.sleep(3)
            # Adjust position (0-255), speed, and force as needed to prep for the sled
            base.set_robotiq_position(arm, position=160, speed=150, force=40)
            time.sleep(1.5)

    # ---------------------------------------------------------
    # GET STARTING POSE FOR CARTESIAN PHASES
    # ---------------------------------------------------------
    code, pos = arm.get_position(is_radian=False)
    if code != 0:
        print("!!! HALTING: Failed to read starting position after Phase 0.")
        return
    
    start_x, start_y, start_z, roll, pitch, yaw = pos

    # ---------------------------------------------------------
    # PHASE 1: GUARDED PROBE SEARCH (+Y)
    # ---------------------------------------------------------
    print("\n--- PHASE 1: Guarded Probe Search (+Y) ---")
    
    if not base.is_within_bounds(start_x, MAX_SAFE_Y, start_z):
        print("!!! HALTING: Max Y search target exceeds SAFE_BOUNDS.")
        return

    arm.set_state(0)
    arm.set_position(x=start_x, y=MAX_SAFE_Y, z=start_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=SPEED, wait=False)
    
    probe_triggered = False
    stop_y = start_y

    while True:
        _, current_pos = arm.get_position(is_radian=False)
        
        if abs(current_pos[1] - MAX_SAFE_Y) < 1.0:
            print("\nPhase 1 Failed: Reached safety boundary without probe trigger.")
            return

        code, pin_state = arm.get_cgpio_digital(ionum=8)
        if code == 0:
            if pin_state == 0:
                arm.set_state(4)  # INSTANT E-STOP
                time.sleep(0.1)
                
                # Calculate Length using absolute Base Frame coordinates
                _, stop_pos = arm.get_position(is_radian=False)
                stop_y = stop_pos[1]
                probe_base_y = STATIONARY_PROBE_BASE_XYZ[1]
                
                object_length = abs(stop_y - probe_base_y)
                
                print("\n\n" + "="*40)
                print(" STATIONARY PROBE TRIGGERED!")
                print(f" Recorded TCP Y     : {stop_y:.2f} mm")
                print(f" Known Probe Base Y : {probe_base_y:.2f} mm")
                print(f" Calculated Length  : {object_length:.2f} mm")
                print("="*40)
                
                probe_triggered = True
                break
            else:
                print(f"Phase 1 Seeking... Y: {current_pos[1]:.1f} mm", end="\r")
        time.sleep(0.01)

    if not probe_triggered:
        return

    # ---------------------------------------------------------
    # PHASE 2: BLIND RETREAT (-Y)
    # ---------------------------------------------------------
    print("\n--- PHASE 2: Blind Retreat (-Y) ---")
    arm.set_state(0)
    
    retreat_y = stop_y - RETREAT_DIST_Y
    print(f"Retracting {RETREAT_DIST_Y}mm to Y={retreat_y:.2f}...")
    
    arm.set_position(x=start_x, y=retreat_y, z=start_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=SPEED, wait=True)
                     
    # ---------------------------------------------------------
    # PHASE 3: LATERAL SHIFT TO SOCKET (+X)
    # ---------------------------------------------------------
    print("\n--- PHASE 3: Lateral Socket Alignment (+X) ---")
    
    shift_x = start_x + DIST_PROBE_SOCKET_X
    print(f"Shifting {DIST_PROBE_SOCKET_X}mm to X={shift_x:.2f}...")
    
    arm.set_position(x=shift_x, y=retreat_y, z=start_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=SPEED, wait=True)

    # ---------------------------------------------------------
    # PHASE 4: FORCE-GUARDED INSERTION (+Y)
    # ---------------------------------------------------------
    print("\n--- PHASE 4: Force-Guarded Insertion (+Y) ---")
    
    print("Taring F/T Sensor...")
    arm.ft_sensor_set_zero()
    time.sleep(0.5)
    arm.set_state(0)
    
    print("Initiating slow insertion move...")
    arm.set_position(x=shift_x, y=MAX_SAFE_Y, z=start_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=SPEED, wait=False)

    while True:
        _, current_pos = arm.get_position(is_radian=False)
        
        if abs(current_pos[1] - MAX_SAFE_Y) < 1.0:
            print("\nSUCCESS: Traveled full permitted distance without excess force.")
            break

        code, ft_data = arm.get_ft_sensor_data()
        if code == 0:
            raw_force = ft_data[FT_AXIS_INDEX]
            abs_force = abs(raw_force)
            
            dir_label = f">>>_{raw_force:.2f}N" if raw_force >= 0 else f"<<<_{abs_force:.2f}N"
            
            if abs_force > FORCE_LIMIT_N:
                arm.set_state(4)  # INSTANT E-STOP
                print("\n\n" + "!"*40)
                print(" >>> INSERTION FORCE THRESHOLD REACHED! <<<")
                print(f" Axis {FT_AXIS_INDEX} Force: {dir_label}")
                print(f" Final Y-Depth: {current_pos[1]:.2f} mm")
                print("!"*40)
                
                # Clear error state so the controller doesn't lock up, 
                # but immediately exit the function to stop the script.
                arm.set_state(0)
                print("\nForce limit detected. Script execution halted.")
                return 
            else:
                print(f"Inserting... Live Force: {dir_label}    ", end="\r")
                
        time.sleep(0.05)


if __name__ == "__main__":
    with base.connect_xarm(ip=ARM_IP) as arm:
        arm.clean_error()
        arm.motion_enable(True)

        print("\n--- Querying Active Controller TCP Parameters ---")
        success, active_weight, active_com, active_offset = base.read_active_tcp_config(arm)
        
        if success and active_weight > 0.05:
            base.set_safe_tcp_payload(arm, weight_kg=active_weight, center_of_mass_mm=active_com)
            arm.set_tcp_offset(active_offset)
            arm.set_state(0)
            time.sleep(0.5)
        else:
            print("!!! HALTING: TCP Payload invalid or too low. Check UFACTORY Studio.")
            sys.exit()

        # Initialize Gripper
        base.init_robotiq_gripper(arm)
        
        # Enable F/T Sensor
        arm.ft_sensor_enable(1)
        time.sleep(1.0)
        
        execute_multistage_insertion(arm)
        
        print("\nSequence Complete.")