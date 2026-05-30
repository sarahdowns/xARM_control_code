import move_safe_Corke as base
import time

with base.connect_xarm() as arm:
    
    # 1. Initialize and calibrate the gripper
    success = base.init_robotiq_gripper(arm)
    
    if success:
        print("\n--- Testing Gripper Limits ---")
        
        # 2. Fully Open the gripper
        base.set_robotiq_position(arm, position=0)
        time.sleep(1) # Brief pause
        
        # 3. Fully Close the gripper (Gently! Force set to 50 out of 255)
        base.set_robotiq_position(arm, position=255, speed=80, force=20)
        time.sleep(1)
        
        # 4. Open it back up halfway
        base.set_robotiq_position(arm, position=128)