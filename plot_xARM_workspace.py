import numpy as np
import roboticstoolbox as rtb
from move_safe_Corke import get_xarm7
import matplotlib.pyplot as plt

robot = get_xarm7()

n_samples = 10000
x_coords = []
y_coords = []
z_coords = []
manipulability = []  # Store the dexterity score here

print("Simulating 10,000 configurations for dexterity analysis...")

for _ in range(n_samples):
    # Generate random valid joint angles
    q = robot.random_q()
    
    # Calculate Forward Kinematics for position
    T = robot.fkine(q)
    pos = T.t
    
    # Calculate the Jacobian matrix at this joint configuration
    J = robot.jacob0(q)
    
    # Calculate Yoshikawa's Manipulability Index
    # (Using pinv or multiplying J by its transpose handles non-square matrices safely)
    mu = np.sqrt(np.linalg.det(J @ J.T))
    
    x_coords.append(pos[0])
    y_coords.append(pos[1])
    z_coords.append(pos[2])
    manipulability.append(mu)

# Plotting the 3D Cloud mapped by Dexterity
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Color code by 'manipulability' instead of 'z_coords'
img = ax.scatter(x_coords, y_coords, z_coords, c=manipulability, cmap='plasma_r', s=2, alpha=0.6)

ax.set_title("xArm7 Workspace: Reachable vs. Dexterous")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_zlabel("Z (meters)")

# Colorbar to easily see high vs low dexterity zones
cbar = fig.colorbar(img, label='Manipulability Index (Dexterity)')
plt.show()