# import math
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # Convert degrees to radians
# angle_1_deg = 160
# angle_2_deg = -160
# angle_1_rad = math.radians(angle_1_deg)
# angle_2_rad = math.radians(angle_2_deg)

# # Convert angles to quaternions assuming rotation around z-axis
# quat_1 = R.from_euler('z', angle_1_rad, degrees=False).as_quat()
# quat_2 = R.from_euler('z', angle_2_rad, degrees=False).as_quat()

# # Calculate the difference quaternion (delta_quat)
# delta_quat = R.from_quat(quat_2) * R.from_quat(quat_1).inv()

# # Convert delta_quat to angle
# angle_axis = delta_quat.as_rotvec()
# delta_angle_rad = np.linalg.norm(angle_axis)

# # Convert delta_angle back to degrees
# delta_angle_deg = math.degrees(delta_angle_rad)

# print("Angle 1 in radians:", angle_1_deg)
# print("Angle 2 in radians:", angle_2_deg)
# print("Quaternion 1:", quat_1)
# print("Quaternion 2:", quat_2)
# print("Delta quaternion:", delta_quat.as_quat())
# print("Delta angle in degrees:", delta_angle_deg)


import numpy as np

# Convert degrees to radians
angle_1_deg = 160
angle_2_deg = -160
angle_1_rad = np.radians(angle_1_deg)
angle_2_rad = np.radians(angle_2_deg)

# Convert angles to rotation matrices
def angle_to_rot_mat(angle_rad):
    cos_val = np.cos(angle_rad)
    sin_val = np.sin(angle_rad)
    rot_mat = np.array([[cos_val, -sin_val, 0],
                        [sin_val, cos_val, 0],
                        [0, 0, 1]])
    return rot_mat

rot_mat_1 = angle_to_rot_mat(angle_1_rad)
rot_mat_2 = angle_to_rot_mat(angle_2_rad)

# Calculate the difference between rotation matrices
delta_rot_mat = np.dot(rot_mat_2, rot_mat_1.T)

# Convert the delta rotation matrix back to delta angle
def rot_mat_to_angle(rot_mat):
    delta_angle_rad = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])
    return delta_angle_rad

delta_angle_rad = rot_mat_to_angle(delta_rot_mat)
delta_angle_deg = np.degrees(delta_angle_rad)

print("Angle 1 (degrees):", angle_1_deg)
print("Angle 2 (degrees):", angle_2_deg)
print("Delta Angle (degrees):", delta_angle_deg)

