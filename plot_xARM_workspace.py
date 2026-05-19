import numpy as np
import roboticstoolbox as rtb
from move_safe_Corke import get_xarm7
import matplotlib.pyplot as plt

robot = get_xarm7()

# We will "sample" the workspace by moving joints 
# This is faster than grid-searching every XYZ point
n_samples = 10000
x_coords = []
y_coords = []
z_coords = []

print("Simulating 10,000 random joint configurations...")

for _ in range(n_samples):
    # Generate random valid joint angles within robot limits
    q = robot.random_q()
    
    # Calculate Forward Kinematics
    T = robot.fkine(q)
    pos = T.t
    
    x_coords.append(pos[0])
    y_coords.append(pos[1])
    z_coords.append(pos[2])

# Plotting the 3D Cloud
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
img = ax.scatter(x_coords, y_coords, z_coords, c=z_coords, cmap='viridis', s=2, alpha=0.5)

ax.set_title("xArm7 Total Reach Envelope (Full 3D)")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_zlabel("Z (meters)")
fig.colorbar(img, label='Height (Z)')
plt.show()