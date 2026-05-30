# File name: move_task_space.py
# This code moves the xARM iteratively using move_safe_Corke.py

import move_safe_Corke as base

robot_model = base.get_xarm7()

with base.connect_xarm() as arm:
   # print(f"xArm Firmware Version: {arm.version}")
   # print(f" Axes Count  : {arm.axis}")
   # print(f" Device Type : {arm.device_type}")
    base.print_current_state(arm)

    # 1. Move to safe home position
    #base.send_safe_home(arm)	# Set in move_safe_Corke.py
    
    # 2. Execute your task space move
    #base.send_task_space_move(arm, robot_model, 450, 0, 130, mask=[1,1,1,0,0,0])
    
    # 3. Return home when done
    base.send_safe_home(arm)
    base.print_current_state(arm)
