"""
Author Dominic McNicholas
UC Mecatronics
Date 09/09/25

Co orinates transformation functions for the assignment and approach points.
"""

import numpy as np
import scipy.spatial.transform as sp
import pandas as pd
import re
import indices
from robodk import robolink, robomath
from robodk.robomath import Pose_2_TxyzRxyz, TxyzRxyz_2_Pose


robot_name = 'robot_3' 
points_filepath = 'robodk_stations/points.xlsx'
points_df = pd.read_excel(points_filepath, sheet_name=robot_name)

#exsiting columns are Key, Frame, Fixture, X, Y, Z, Location (that may contain o'clock)

#So. Perant is just the "Fixture" coulmn. The global positon of each perant is the "X", "Y", "Z" position of the row with Fixture = "Perant", Frame = "world" and Origin = "Yes"
#Add rotation columns for each axis
points_df['Rx'] = 0
points_df['Ry'] = 0
points_df['Rz'] = 0
#All Keys have zero rotation except the global position of each perant frame

#Rows in the Frame = 'world' and Origin = 'No' define the rotation of rows with the same Fixture as the Frame value
#if there is no rotation then it is at the zero (1200) in its shared plane with its perant
#if there is a rotaion (o'clock) then it is rotated that many hours from the zero point in the plane shared with its perant

#find these rotaions
points_df['Relitive Pos'] = 0.0
for i, row in points_df.iterrows():
    location = row['Location']
    if isinstance(location, str):
        match = re.search(r'\((\d+)\s*o\'clock\)', location)
        if match:
            oclock_value = int(match.group(1))
            points_df.at[i, 'Relitive Pos'] += oclock_value
#convert "o'clock" to radians
points_df['Relitive Pos'] = np.where(
    points_df['Relitive Pos'] != 0,
    np.deg2rad((points_df['Relitive Pos'] - 1200) * 360 / 1200),
    0
)

#find the plane wich each rotaion is defined in and then apply that to the world origin for that fixture
# do this for all non origin world frames
points_df['qx'] = 0.0
points_df['qy'] = 0.0
points_df['qz'] = 0.0
points_df['qw'] = 1.0  # identity rotation
points_df['theta_x'] = 0.0
points_df['theta_y'] = 0.0
points_df['theta_z'] = 0.0

eps = 1e-10

for i, row in points_df.iterrows():
    if row['Frame'] == 'world' and str(row.get('Origin', '')).lower() == 'no':
        fixture = row['Fixture']

        # print(f"Processing rotation for fixture '{fixture}' at row {i}")
        # --- Find parent origin row ---
        parent_origin = points_df[
            (points_df['Fixture'] == fixture) &
            (points_df['Frame'] == 'world') &
            (points_df['Origin'].astype(str).str.lower() == 'yes')
        ]
        if parent_origin.empty:
            continue
        prow = parent_origin.iloc[0]

        # --- Positions ---
        p_origin = np.array([prow['X'], prow['Y'], prow['Z']], dtype=float)
        p_child  = np.array([row['X'], row['Y'], row['Z']], dtype=float)
        v = p_child - p_origin
        if np.linalg.norm(v) < eps:
            continue
        v /= np.linalg.norm(v)
        # print(f"Direction vector v: {v}")
        # --- Reference direction (zero o’clock sibling if present) ---
        ref_dir = np.array([1.0, 0.0, 0.0])  # default fallback
        siblings = points_df[
            (points_df['Fixture'] == fixture) &
            (points_df['Frame'] == 'world') &
            (points_df['Origin'].astype(str).str.lower() == 'no')
        ]
        zero_sibs = siblings[np.isclose(siblings['Relitive Pos'], 0.0)]
        if not zero_sibs.empty:
            zrow = zero_sibs.iloc[0]
            p_zero = np.array([zrow['X'], zrow['Y'], zrow['Z']], dtype=float)
            ref = p_zero - p_origin
            if np.linalg.norm(ref) > eps:
                ref_dir = ref / np.linalg.norm(ref)

            # print(f"Reference direction ref_dir: {ref_dir}")

        # --- Rotation axis ---
        axis = np.cross(ref_dir, v)
        if np.linalg.norm(axis) < 1e-9:
            # fallback if parallel
            if abs(ref_dir[2]) < 0.9:
                axis = np.cross(ref_dir, np.array([0.0, 0.0, 1.0]))
            else:
                axis = np.cross(ref_dir, np.array([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        # print(f"Chosen rotation axis: {axis}")

        # # --- Relative rotation quaternion ---
        # rel_angle = float(row['Relitive Pos'])
        # q_rel = sp.Rotation.from_rotvec(axis * rel_angle)

        # # --- Parent quaternion ---
        # q_parent_arr = np.array([prow['qx'], prow['qy'], prow['qz'], prow['qw']], dtype=float)
        # if np.linalg.norm(q_parent_arr) < eps:
        #     q_parent = sp.Rotation.identity()
        # else:
        #     q_parent_arr /= np.linalg.norm(q_parent_arr)
        #     q_parent = sp.Rotation.from_quat(q_parent_arr)
        # # print(f"Parent quaternion: {q_parent.as_quat()}")
        # # --- Compose child quaternion ---
        # q_child = q_parent * q_rel
        # qx, qy, qz, qw = q_child.as_quat()

        # # --- Save quaternion ---
        # points_df.at[i, 'qx'] = qx
        # points_df.at[i, 'qy'] = qy
        # points_df.at[i, 'qz'] = qz
        # points_df.at[i, 'qw'] = qw

        # # --- Save Euler (XYZ, radians) if needed ---
        # theta_x, theta_y, theta_z = q_child.as_euler('xyz', degrees=False)
        # points_df.at[i, 'theta_x'] = theta_x
        # points_df.at[i, 'theta_y'] = theta_y
        # points_df.at[i, 'theta_z'] = theta_z

        # print(f"Row {i}: Quaternion = {qx, qy, qz, qw}, Euler angles = {theta_x, theta_y, theta_z}")



def get_global_pose(frame_key, pos_x, pos_y, pos_z, psi, phi, theta):
    """
    Compute the global pose of a frame/tool by recursively walking up the frame hierarchy.

    Parameters:
        frame_key : int
            The key of the current frame in points_df
        pos_x, pos_y, pos_z : float
            Local position relative to the frame
        rot_theta : float
            Local rotation relative to the frame (planar)
    Returns:
        4x4 np.ndarray : global homogeneous transformation matrix
    """
    # Local transform of the tool in this frame
    local_ht = TxyzRxyz_2_Pose([pos_x, pos_y, pos_z, psi, phi, theta])
    
    # Get frame position and rotation
    # print(frame_key)
    # print(points_df.iloc[frame_key-1])
    frame_row = points_df.loc[points_df['Key'] == frame_key].iloc[0]
    frame_ht = TxyzRxyz_2_Pose([
        frame_row['X'],
        frame_row['Y'],
        frame_row['Z'],
        frame_row.get('Roll', 0),
        frame_row.get('Pitch', 0),
        frame_row['Planar Rotation']]
    )
    
    # Compute the transform of this frame relative to its parent
    combined_ht = frame_ht @ local_ht
    
    parent_key = frame_row['Parent']
    
    if parent_key is None or pd.isna(parent_key):
        return combined_ht
    else:
        # Recursively compute the parent global transform
        return get_global_pose(parent_key, 0, 0, 0, 0) @ combined_ht




if __name__ == "__main__":

    target = TxyzRxyz_2_Pose([-591,-215.5,214/1,0,90*np.pi/180,0])
    TCP = TxyzRxyz_2_Pose([0,0,102.82, np.pi,0*np.pi/180,0])
    inv_TCP = robomath.invH(TCP)
    pose = target * inv_TCP
    # print(pose)

    row = points_df.iloc[2]
    frame_key = row['Key']
    pos_x = row['X']
    pos_y = row['Y']
    pos_z = row['Z']
    psi = row.get('Roll', 0)
    phi = row.get('Pitch', 0)
    theta = row.get('Planar Rotation', 0)

    print(f"pose for key {frame_key}:")
    print(f"{TxyzRxyz_2_Pose([pos_x, pos_y, pos_z, psi, phi, theta])}")
    
    



