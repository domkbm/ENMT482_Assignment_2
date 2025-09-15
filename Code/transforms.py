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
import indices as idx



            

def create_points_df(robot_name='robot_3', filepath='robodk_stations/points.xlsx'):
    points_df = pd.read_excel(filepath, sheet_name=robot_name)
    points_df['Euler_Rx'] = np.nan
    points_df['Euler_Ry'] = np.nan
    points_df['Euler_Rz'] = np.nan

    for i, row in points_df.iterrows():
        if row['Frame'] == 'world' and str(row.get('Origin', '')).lower() == 'yes':
            fixture = row['Fixture']
            children = points_df[
                (points_df['Fixture'] == fixture)
                & (points_df['Frame'] == 'world')
                & (points_df['Origin'].str.lower() == 'no')
            ]

            if children.empty:
                # print(f"Row {i+1}: No points defining rotation, frame is not rotated compared to origin")
                continue

            # Collect local vectors to define rotation
            local_vectors = []
            global_vectors = []
            for j, child_row in children.iterrows():
                location = child_row['Location']
                local_frame = points_df[
                    (points_df['Location'] == location)
                    & (points_df['Frame'] != 'world')
                ]
                if local_frame.empty:
                    # print(f"Row {i+1}, Child Row {j+1}: No local frame found, skipping")
                    continue

                local_frame = local_frame.iloc[0]
                vec_local = np.array([local_frame['X'], local_frame['Y'], local_frame['Z']], dtype=float)
                # print(f"Row {i+1}, Child Row {j+1}: Local vector {vec_local}")
                local_vectors.append(vec_local)
                vec_global = np.array([child_row['X'], child_row['Y'], child_row['Z']], dtype=float) - np.array([row['X'], row['Y'], row['Z']], dtype=float)
                # print(f"Row {i+1}, Child Row {j+1}: Global vector {vec_global}")
                global_vectors.append(vec_global)
            
            #now we have all the vectors, lets find the rotation for the perant frame
            global_vectors = np.array(global_vectors)
            local_vectors = np.array(local_vectors)
            #cup dispencer and mazzer have complex rotations that cannot be idetifed with just the vectors given
            if fixture == 'cup_dispenser':
                #cup dispencer is first rotated about Y 90degrees then rotated around its new x by an amout defined by its child point (point 2)
                v = global_vectors[0]
                v_proj = np.array([v[0], v[1], 0])
                ax1 = np.array([1, 0, 0])  # x-axis as reference
                ax2 = np.array([0, 1, 0])  # y-axis
                #calc rottaion around new x, is the old z so we can just use that 
                theta1 = np.arctan2(-np.dot(v_proj, ax1), np.dot(v_proj, ax2)) # this is the angle between the 2 and the global frame 
                theta2 = np.arctan2(local_vectors[0][2],local_vectors[0][1] ) # this is the angle between the 2 and the local frame 
                theta = theta1 - theta2 # so this is the rotation of the local frame wrt global (around x)
                Ry = np.array([[0, 0, -1],
                              [0, 1, 0],
                              [1, 0, 0]])
                Rx = np.array([[1,0,0],
                               [0, np.cos(theta), -np.sin(theta)],
                               [0,np.sin(theta), np.cos(theta)]])
                R = Ry @ Rx
                rotation = sp.Rotation.from_matrix(R)
                euler_angles = rotation.as_euler('XYZ', degrees=False)
                # print(f"Row {i+1}: Computed Euler angles for cup_dispenser: {180*euler_angles/np.pi} deg")
                points_df.loc[i, ['Euler_Rx', 'Euler_Ry', 'Euler_Rz']] = euler_angles


            elif fixture == 'mazzer':
                local_vectors = np.vstack([local_vectors, np.array([0,0,0])])
                global_vectors = np.vstack([global_vectors, np.array([row['X'], row['Y'], row['Z']])])
                rotation = sp.Rotation.align_vectors(global_vectors, local_vectors)
                euler_angles = rotation[0].as_euler('XYZ', degrees=False)
                points_df.loc[i, ['Euler_Rx', 'Euler_Ry', 'Euler_Rz']] = euler_angles
                # print(f"Row {i+1}: Computed Euler angles {180*euler_angles / np.pi} deg")                


            else: # all other fframes are the simple case of rotation around z, this is a quick way of doing it.
                #we know they are rotations in z so set the z values to zero to remove abiguity 
                global_vectors[:,2] = 0 
                local_vectors[:,2] = 0 
                rotation = sp.Rotation.align_vectors(global_vectors, local_vectors)
                euler_angles = rotation[0].as_euler('XYZ', degrees=False)
                points_df.loc[i, ['Euler_Rx', 'Euler_Ry', 'Euler_Rz']] = euler_angles

        else: #currently assume that there is no rotation going on in local frames
            euler_angles = [0,0,0]
            points_df.loc[i, ['Euler_Rx', 'Euler_Ry', 'Euler_Rz']] = euler_angles


   
    return points_df

  


def get_global_frame(frame_df, frame_key):
    """
    Compute the global pose of a frame by recursively walking up the frame hierarchy.

    Parameters:
        frame_key : int
            The key of the current frame in points_df
    Returns:
        4x4 np.ndarray : global homogeneous transformation matrix
    """
    # Get frame position and rotation
    frame_row = frame_df.loc[frame_df['Key'] == frame_key].iloc[0]
    # print(frame_row)
    frame_ht = TxyzRxyz_2_Pose([
        frame_row['X'],
        frame_row['Y'],
        frame_row['Z'],
        frame_row['Euler_Rx'],
        frame_row['Euler_Ry'],
        frame_row['Euler_Rz']]
    )
    return frame_ht


def pose(frame_df,frame_key, tool=None, pos_x = 0, pos_y = 0, pos_z = 0, theta_x = 0, theta_y = 0, theta_z = 0, off_x = 0, off_y = 0, off_z = 0):
    """
    Compute the global pose of a frame/tool by recursively walking up the frame hierarchy.

    Parameters:
        frame_key : int
            The key of the desired frame in points_df 
        tool : int
            The key of the desired tool in points_f
        pos_x, pos_y, pos_z : float
            Local position relative to the frame (mm)
        theta_x, theta_y, theta_z : float (deg)
            Local rotation relative to the frame
        off_x,off_y,off_z
            Experiemental tool translations, probs olny for mazzer (dont use these atm)    
    Returns:
        4x4 np.ndarray : global homogeneous transformation matrix
    """
    theta_x = np.deg2rad(theta_x)
    theta_y = np.deg2rad(theta_y)
    theta_z = np.deg2rad(theta_z)

    if tool == None:
        tool_ht = TxyzRxyz_2_Pose([0,0,0,0,0,0])
    else:
        tcp_key = tool
        tool_row = frame_df.loc[frame_df['Key'] == tcp_key].iloc[0]
        tool_ht = TxyzRxyz_2_Pose([
        tool_row['X'] - off_x,
        tool_row['Y'] - off_y,
        tool_row['Z'] - off_z,
        tool_row.get('Euler_Rx', 0),
        tool_row.get('Euler_Ry', 0),
        tool_row.get('Euler_Rz', 0)])
        tool_ht = robomath.invH(tool_ht)
        # print(f"TOOL Row {tool_row}")
        # print(f"tool pose {tool_row['X'],tool_row['Y'],tool_row['Z'],}")
        # print(f"TOOL HT {tool_ht}")
    
    # Local transform of the robot in this frame
    local_ht = TxyzRxyz_2_Pose([pos_x, pos_y, pos_z, theta_x, theta_y, theta_z])
    
    # Get frame position and rotation
    # print(f'frame {frame_key}')
    # print(frame_df.iloc[frame_key-1])
    frame_row = frame_df.loc[frame_df['Key'] == frame_key].iloc[0]
    frame_ht = TxyzRxyz_2_Pose([
        frame_row['X'],
        frame_row['Y'],
        frame_row['Z'],
        frame_row.get('Euler_Rx', 0),
        frame_row.get('Euler_Ry', 0),
        frame_row.get('Euler_Rz', 0)]
    )
 
    # Compute the transform of this frame relative to its parent
    combined_ht = frame_ht * local_ht
       
    if frame_row['Origin'] == 'yes': # This is an orgin in world co-ords, end of the line.
        return combined_ht * tool_ht
    else:
        # nothing is defined under two local transforms so not really nessesary to do recursive (readibility) to compute the parent global transform
        perant_rows = frame_df[
                (frame_df['Fixture'] == frame_row['Fixture'])
                & (frame_df['Frame'] == 'world')
                & (frame_df['Origin'].str.lower() == 'yes')
            ]
    
        perant_key = perant_rows.iloc[0]['Key']
        return get_global_frame(frame_df, perant_key) * combined_ht * tool_ht




if __name__ == "__main__":
    frames = create_points_df()
    mazzer = pose(frames, idx.Mazzer)
    print(mazzer)
    # on_button = pose(frames, idx.Mazzer_On_Button)
    # print(on_button)
    # # check1 = pose(frames, idx.Cup, tool=idx.Mazzer_Tool)
    # # print(check1)

    


