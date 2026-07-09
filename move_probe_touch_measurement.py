import time
import move_safe_Corke as base
from xarm.wrapper import XArmAPI

# Replace with your actual robot IP
ARM_IP = '192.168.1.225' 

def execute_recorded_probe_search():
    print(f"Connecting to {ARM_IP}...")
    arm = XArmAPI(ARM_IP)
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)

    # ---------------------------------------------------------
    # 1. HARDWARE STACK CONFIGURATION
    # ---------------------------------------------------------
    print("\n--- Configuring TCP for F/T Sensor & Robotiq Gripper ---")
    weight_kg = 1.37          
    com_mm = [0.0, 0.0, 70.0] 
    tcp_offset = [0.0, 0.0, 197.0, 0.0, 0.0, 0.0] # 197mm Z-offset to the tool tip
    
    arm.set_tcp_load(weight_kg, com_mm)
    arm.set_tcp_offset(tcp_offset)
    arm.set_state(0)
    time.sleep(0.5)

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
    print(f" Search Speed : 3 mm/s (Probe Safe Limit)")
    print("="*45)

    # ---------------------------------------------------------
    # 3. LAUNCH SEARCH MOTION (Non-blocking)
    # ---------------------------------------------------------
    arm.set_position(x=cur_x, y=target_y, z=cur_z, roll=roll, pitch=pitch, yaw=yaw, 
                     speed=3, wait=False)

    print("\n>>> ARM IS MOVING. AWAITING PROBE TRIGGER... <<<\n")

    # ---------------------------------------------------------
    # 4. HIGH-SPEED POLLING & RECORDING LOOP
    # ---------------------------------------------------------
    while True:
        _, current_pos = arm.get_position(is_radian=False)
        
        if abs(current_pos[1] - target_y) < 1.0:
            print("\n\nSearch exhausted.")
            break

        code, pin_state = arm.get_cgpio_digital(ionum=8)
        
        if code == 0 and pin_state == 0:
            # 1. INSTANT E-STOP
            arm.set_state(4) 
            time.sleep(0.1) 
            
            # 2. RECORD POSITION
            _, stop_pos = arm.get_position(is_radian=False)
            
            # 3. CALCULATE AND PRINT (All in one block)
            PROBE_OFFSET_Y = 6.18 
            object_length = abs(stop_pos[1] - PROBE_OFFSET_Y)
            
            print("\n\n" + "!"*50)
            print(" >>> PROBE TRIGGERED! MOVEMENT HALTED! <<<")
            print(f" Recorded TCP Y  : {stop_pos[1]:.2f} mm")
            print(f" Calculated Len. : {object_length:.2f} mm")
            print("!"*50)
            
            # 4. AUTOMATED RETREAT
            retreat_y = stop_pos[1] - 50.0
            print(f"\n-> Retreating 50mm to Y: {retreat_y:.2f} mm...")
            
            arm.set_state(0) # Re-enable motion
            arm.set_position(x=stop_pos[0], y=retreat_y, z=stop_pos[2], 
                             roll=stop_pos[3], pitch=stop_pos[4], yaw=stop_pos[5], 
                             speed=20, wait=True)
            
            print("-> Retreat complete.")
            break # Exit the while loop
            
        elif pin_state == 1:
            print(f"Moving... DI0: {pin_state} (Idle), Current Y: {current_pos[1]:.1f}", end="\r")
        
        time.sleep(0.01)

    print("\nDisconnecting...")
    arm.disconnect()

if __name__ == "__main__":
    execute_recorded_probe_search()