# Author: Sarah Downs
# When run this script will direct the arm to move 200mm in +y, but stop moving if the probe is triggered
# and DI0 = 0. The script with then output the current XYZ coordinates of the robot's TCP.

import time
from xarm.wrapper import XArmAPI

ARM_IP = '192.168.1.225'    # Robot IP

def execute_recorded_probe_search():
    print(f"Connecting to {ARM_IP}...")
    arm = XArmAPI(ARM_IP)
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)

    # ---------------------------------------------------------
    # 1. HARDWARE STACK CONFIGURATION (F/T Sensor + Gripper)
    # ---------------------------------------------------------
    # This guarantees the XYZ coordinates recorded match the Tool Center Point
    print("\n--- Configuring TCP for F/T Sensor & Robotiq Gripper ---")
    weight_kg = 1.37          
    com_mm = [0.0, 0.0, 70.0] 
    tcp_offset = [0.0, 0.0, 197.0, 0.0, 0.0, 0.0] # 197mm Z-offset to the tool tip
    
    arm.set_tcp_load(weight_kg, com_mm)
    arm.set_tcp_offset(tcp_offset)
    arm.set_state(0)
    time.sleep(0.5) # Give the controller's physics engine a moment to update

    # ---------------------------------------------------------
    # 2. ESTABLISH BASELINE
    # ---------------------------------------------------------
    code, pos = arm.get_position(is_radian=False)
    if code != 0:
        print("Failed to get starting position.")
        arm.disconnect()
        return

    cur_x, cur_y, cur_z, roll, pitch, yaw = pos
    target_y = cur_y + 200.0  # Max travel limit

    print("\n" + "="*45)
    print(f" INITIATING TCP-RECORDED PROBE SEARCH")
    print(f" Max Y Travel : {target_y:.1f} mm")
    print(f" Search Speed : 10 mm/s (Probe Safe Limit)")
    print("="*45)

    # ---------------------------------------------------------
    # 3. LAUNCH SEARCH MOTION
    # ---------------------------------------------------------
    arm.set_position(x=cur_x, y=target_y, z=cur_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=10, wait=False)

    print("\n>>> ARM IS MOVING. AWAITING PROBE TRIGGER... <<<\n")

    # ---------------------------------------------------------
    # 4. HIGH-SPEED POLLING & RECORDING LOOP
    # ---------------------------------------------------------
    while True:
        # Check if we hit the 200mm boundary without finding anything
        _, current_pos = arm.get_position(is_radian=False)
        if abs(current_pos[1] - target_y) < 1.0:
            print("\n\nSearch exhausted. Reached 200mm limit without trigger.")
            break

        # Read DI0 directly via index 8
        code, pin_state = arm.get_cgpio_digital(ionum=8)
        
        if code == 0:
            if pin_state == 1:
                # Print live tracking cleanly
                print(f"Moving... DI0: {pin_state} (Idle), Current Y: {current_pos[1]:.1f}", end="\r") 
                
            elif pin_state == 0:
                # INSTANT E-STOP
                arm.set_state(4) 
                
                # Allow physical momentum to settle completely before taking the measurement
                time.sleep(0.1) 
                
                # --- RECORD THE EXACT TCP POSITION ---
                _, stop_pos = arm.get_position(is_radian=False)
                stop_x, stop_y, stop_z = stop_pos[0], stop_pos[1], stop_pos[2]
                
                print("\n\n" + "="*50)
                print("Probe Triggered. Movement Halted")
                print(f" Recorded TCP X : {stop_x:.2f} mm")
                print(f" Recorded TCP Y : {stop_y:.2f} mm")
                print(f" Recorded TCP Z : {stop_z:.2f} mm")
                print("="*50)
                
                # Clear the error state so the robot can accept future commands
                arm.set_state(0) 
                
                # --- OPTIONAL: Save to file here ---
                # with open("probe_data.txt", "a") as file:
                #     file.write(f"Contact at: X={stop_x:.2f}, Y={stop_y:.2f}, Z={stop_z:.2f}\n")
                
                break
                
        # 100Hz polling rate
        time.sleep(0.01)

    print("\nDisconnecting...")
    arm.disconnect()

if __name__ == "__main__":
    execute_recorded_probe_search()