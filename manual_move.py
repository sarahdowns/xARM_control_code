import move_safe_Corke as base
import time

def run_joint_freedrive():
    with base.connect_xarm() as arm:
        print("\n" + "!"*45)
        print(" KEEP ONE HAND ON THE ROBOT BEFORE STARTING")
        print("!"*45)
        input("Press ENTER to engage gravity compensation...")
        
        # 1. Aggressively clear errors and wait for the controller
        print("Clearing hardware states...")
        arm.clean_error()
        time.sleep(1.0)
        
        # 2. Re-enable motors and brakes
        arm.motion_enable(True)
        time.sleep(0.5)
        
        # 3. Enter Manual Teaching (Mode 2)
        arm.set_mode(2)
        arm.set_state(0)
        time.sleep(0.5)
        
        # 4. Safety Check: Verify the mode actually changed
        # FIXED: Use the property 'arm.mode' instead of the non-existent 'arm.get_mode()'
        current_mode = arm.mode
        if current_mode != 2:
            print(f"\n!!! FAILED: Robot refused Freedrive. Currently stuck in Mode {current_mode}.")
            print("Action required: Power cycle the xArm control box manually.")
            return

        print("\n" + "="*40)
        print(" >>> COMPLIANCE ACTIVE (FREEDRIVE) <<<")
        print("="*40)
        print("You can now physically move the joints.")
        
        # 5. Wait for the user to finish moving the arm
        input("\nMove to a safe position, hold the arm steady, and press ENTER to lock...")
        
        # 6. Lock motors back into standard Position Control (Mode 0)
        print("Locking brakes...")
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.5)
        
        print("Motors locked. You can safely let go.")
        
        # 7. Print the new coordinates so you can save them
        base.print_current_state(arm)

if __name__ == "__main__":
    run_joint_freedrive()