import move_safe_Corke as base
import time
import math

def execute_guarded_y_move(arm, target_y_mm, speed_mms=20, force_limit_N=15.0):
    """
    Drives the arm linearly along the Y-axis. Constantly polls the F/T sensor.
    If the force limit is exceeded, the arm emergency-stops and prompts the user.
    """
    # 1. Fetch current pose to lock X, Z, and Orientation
    code, pos = arm.get_position(is_radian=False)
    if code != 0:
        print("Failed to read position.")
        return
    
    cur_x, cur_y, cur_z, roll, pitch, yaw = pos
    
    print("\n" + "="*40)
    print(f" INITIATING GUARDED Y-AXIS MOVE ")
    print(f" Target Y : {target_y_mm} mm")
    print(f" Force Limit: {force_limit_N} Newtons")
    print("="*40)

    # 2. Tare the F/T sensor immediately before moving to get a clean baseline
    print("Taring F/T sensor...")
    #arm.ft_sensor_set_zero()
    time.sleep(0.5)

    # >>> NEW FIX: Clear the shockwave stop-state <<<
    arm.set_state(0) 
    time.sleep(0.1)

    # 3. Start the non-blocking movement
    arm.set_position(x=cur_x, y=target_y_mm, z=cur_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=speed_mms, wait=False)
    
    # 4. The High-Speed Monitoring Loop (IMMEDIATELY after moving)
    while True:
        # A. Check if we reached the target successfully
        _, current_pos = arm.get_position(is_radian=False)
        if abs(current_pos[1] - target_y_mm) < 1.0: # Within 1mm of target
            print("\nSUCCESS: Target reached without interference.")
            break
            
        # B. Read the F/T Sensor
        code, ft_data = arm.get_ft_sensor_data()
        if code == 0:
            # We look at the Y-axis force (index 1). We use absolute value 
            y_force = abs(ft_data[1])
            
            # C. Check against our safety threshold
            if y_force > force_limit_N:
                # INSTANTLY STOP THE ROBOT
                arm.set_state(4) 
                
                print("\n" + "!"*40)
                print(f" CONTACT DETECTED! ")
                print(f" Y-Axis Force Spiked to: {y_force:.2f} N")
                print("!"*40)
                
                ans = input("\nProcess paused. Do you want to resume the movement? (y/n): ")
                
                if ans.lower() == 'y':
                    print("Resuming movement...")
                    arm.set_state(0)
                    arm.ft_sensor_set_zero()
                    time.sleep(0.5)
                    arm.set_position(x=cur_x, y=target_y_mm, z=cur_z, roll=roll, pitch=pitch, yaw=yaw, 
                                     speed=speed_mms, wait=False)
                else:
                    print("Movement aborted by user. Remaining in current position.")
                    break
        
        # Poll at 50ms intervals (20 times a second)
        time.sleep(0.05)


# =========================================================
# MAIN EXECUTION
# =========================================================
if __name__ == "__main__":
    
    with base.connect_xarm() as arm:
        # 1. Wake up & Clear Errors
        arm.clean_error()
        arm.motion_enable(True)

        # ---------------------------------------------------------
        # 2. HARDWARE STACK CONFIGURATION & VERIFICATION
        # ---------------------------------------------------------
        print("\n--- Applying & Verifying Hardware Configuration ---")
        measured_weight = 1.37          
        measured_com = [0.0, 0.0, 70.0] 
        
        # First, send our known safe payload to the controller
        base.set_safe_tcp_payload(arm, weight_kg=measured_weight, center_of_mass_mm=measured_com)
        arm.set_tcp_offset([0, 0, 197, 0, 0, 0])
        arm.set_state(0)
        time.sleep(0.5)
        
        # Second, read it back from the controller to guarantee it was applied
        success, current_weight, current_com, current_offset = base.read_active_tcp_config(arm)
        
        if not success or current_weight < 0.5:
            print("!!! HALTING: TCP Configuration failed to apply correctly.")
            exit()

        # 3. Initialize Gripper & Enable F/T Sensor
        base.init_robotiq_gripper(arm)
        arm.ft_sensor_enable(1)
        time.sleep(1.0)
        
        print("\n--- Hardware Ready ---") 

        # 4. Grab the Rigid Object
        print("Closing gripper on rigid object...")
        base.set_robotiq_position(arm, position=250, speed=150, force=50)
        time.sleep(7.0) 
        
        # 5. Execute the Guarded Move
        #target_y = 400.0
        target_y = 602.0
        # target_y = current_pos[1] + 100.0    # To move relative to where it currently is
        execute_guarded_y_move(arm, target_y_mm=target_y, speed_mms=20, force_limit_N=15.0)
        
        # 6. End of Task
        print("\nTask Complete. Releasing object...")
        #base.set_robotiq_position(arm, position=0)