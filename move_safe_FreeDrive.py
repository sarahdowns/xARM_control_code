# File name: move_safe_FreeDrive.py

import move_safe_Corke as base
import time

def monitor_ft_sensor():
    with base.connect_xarm() as arm:
        print("\nInitializing Force/Torque Sensor...")
        
        # 1. Enable the sensor
        arm.clean_error()
        time.sleep(1.0)
        arm.motion_enable(True)
        time.sleep(0.5)

        arm.ft_sensor_enable(1)
        time.sleep(0.5) # Give the sensor a moment to wake up
        
        # 2. Tare the sensor (Ensure you are NOT touching the arm when this runs)
        print("Taring sensor (Do not touch!)...")
        arm.ft_sensor_set_zero()
        time.sleep(1.0)
        
        print("\n" + "="*45)
        print(" LIVE SENSOR STREAM (Press Ctrl+C to stop) ")
        print("="*45)
        
        try:
            while True:
                code, data = arm.get_ft_sensor_data()
                
                if code == 0:
                    # Format the output to stay on a single updating line (optional, but clean)
                    fx, fy, fz = data[0], data[1], data[2]
                    tx, ty, tz = data[3], data[4], data[5]
                    
                    print(f"Force [N]: X:{fx:6.2f}  Y:{fy:6.2f}  Z:{fz:6.2f} | "
                          f"Torque [Nm]: Roll:{tx:5.2f} Pitch:{ty:5.2f} Yaw:{tz:5.2f}")
                else:
                    print(f"Sensor read error. Code: {code}")
                    
                time.sleep(0.5) # Read twice a second
                
        except KeyboardInterrupt:
            print("\nDiagnostic stream stopped by user.")

if __name__ == "__main__":
    monitor_ft_sensor()