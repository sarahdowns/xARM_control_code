import move_safe_Corke as base

# 1. Initialize mathematical model
robot_model = base.get_xarm7()

with base.connect_xarm() as arm:
    
    # 2. Fetch the current real-world state of the robot
    code, current_pos = arm.get_position(is_radian=False)
    
    if code != 0:
        print(f"!!! ERROR: Could not read current position. Code {code}")
    else:
        # Unpack the current coordinates and orientation
        cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw = current_pos
        
        print("\n" + "-"*35)
        print("          STARTING STATE          ")
        print(f" XYZ : {cur_x:.1f}, {cur_y:.1f}, {cur_z:.1f}")
        print(f" RPY : {cur_roll:.2f}, {cur_pitch:.2f}, {cur_yaw:.2f}")
        print("-" * 35)

        # 3. Define your NEW Target XYZ
        target_x = 400
        target_y = 400.0
        target_z = 150
        
        print(f"\n--- Initiating Linear XYZ Translation ---")
        print(f"Moving to XYZ: {target_x}, {target_y}, {target_z}")
        print("Holding starting orientation strictly locked.")
        
        # 4. Execute the linear move
        base.send_linear_move(
            arm, 
            robot_model, 
            x_mm=target_x, 
            y_mm=target_y, 
            z_mm=target_z, 
            roll=cur_roll,     # Reusing the exact starting Roll
            pitch=cur_pitch,   # Reusing the exact starting Pitch
            yaw=cur_yaw,       # Reusing the exact starting Yaw
            speed=20,          # 20 mm/s for a safe, smooth slide
            mask=[1, 1, 1, 0, 0, 0]  # Strict enforcement of all 6 axes
        )