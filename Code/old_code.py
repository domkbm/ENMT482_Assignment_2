
arc_single = tf.generate_circular_path(spin_start_pose, tf.pose(points_df, id.Rancillio_Gasket), 45)
def basket_spin_fwd():
    UR5.MoveL(spin_start_pose)
    UR5.MoveC(arc_single[1], arc_single[2])

def basket_spin_bkwd():
    # UR5.MoveL(spin_end_pose) # if coming from somwhere else
    UR5.MoveC(arc_single[1], spin_start_pose)
    UR5.MoveL(tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool, pos_z=-15, theta_y=-90, off_theta_x=45))


def basken_spin_fwd_circ():
    UR5.MoveL(spin_start_pose)
    print(len(arc_many[1:]) / 2)
    for i in range(int(len(arc_many[1:]) / 2)):
        UR5.MoveC(arc_many[2*i+1], arc_many[2*i+2])


while weight <= weight_target:
    # Forward movement
    i = j = 0
    for pose in circular_path:
        i += 1
        print(f"fwrd, step {i} weight {weight}")
        UR5.MoveL(pose, blocking=True)     
        weight += 7/60  # increment weight after each move
        if weight > weight_target:
            break  # exit the for loop if weight exceeded
    if weight > weight_target:
        break  # exit the while loop if weight exceeded
    # Backward movement
    i = j = 0
    for pose in reversed(circular_path):
        j += 1
        print(f"back, step {j} weight {weight}")
        UR5.MoveL(pose, blocking=True)
        weight = weight  # increment weight after each move
        if weight > weight_target:
            break
    if weight > weight_target:
        break

if i == 0:
    UR5.MoveL(rm.TxyzRxyz_2_Pose([0,0,20,0,0,0]) * circular_path[len(circular_path) - j])
elif j==0:
    UR5.MoveL(rm.TxyzRxyz_2_Pose([0,0,20,0,0,0]) * circular_path[i])
else: 
    pass     


local_norms = np.linalg.norm(local_vectors, axis=1,keepdims=True)
                local_norms[local_norms == 0] = 1 
                local_norms = local_vectors / local_norms
                global_norms = np.linalg.norm(global_vectors, axis=1,keepdims=True)
                global_norms[global_norms == 0] = 1 
                global_norms = global_vectors / global_norms
                print(f"local normed {local_norms}")
                print(f"global normed {global_norms}")


pose_from_euler = TxyzRxyz_2_Pose([global_vectors[-1][0],global_vectors[-1][1],global_vectors[-1][2],euler_angles[0], euler_angles[1],euler_angles[2] ])
                print(pose_from_euler)

                print(f"Row {i+1}: Computed Euler angles "
                        f"{180*euler_angles[0] /np.pi:.1f}°, {180*euler_angles[1]/np.pi:.1f}°, {180*euler_angles[2]/np.pi:.1f}°")
                
                def rotation_z(theta):
                    return np.array([
                        [np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]
                    ])

                # Function to calculate the rotation matrix for X-axis
                def rotation_x(phi):
                    return np.array([
                        [1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]
                    ])
         
                
                theta = np.arctan2(np.dot(global_vectors[0],[0,1,0]), np.dot(global_vectors[0],[1, 0, 0]))
                print((theta+np.pi)*180/np.pi)
                Rz = rotation_z(theta + np.pi)
                theta = np.arctan2(np.dot(global_vectors[1],[0,0,1]), np.sqrt(np.dot(global_vectors[1],[1, 0, 0])**2 +np.dot(global_vectors[1],[0, 1, 0])**2))
                print((theta)*180/np.pi)
                Rx = rotation_x(theta)
                R = Rz @ Rx
                #make HT matrix 
                ht_matrix = np.eye(4)
                ht_matrix[:3, :3] = R
                ht_matrix[:3, 3] = global_vectors[-1]
                print(ht_matrix)
                rotation = sp.Rotation.from_matrix(R)
                euler_angles = rotation.as_euler('XYZ', degrees=False)
                print(f"Row {i+1}: Computed Euler angles for mazzer: {180*euler_angles/np.pi} deg")
                pose_from_euler = TxyzRxyz_2_Pose([global_vectors[-1][0],global_vectors[-1][1],global_vectors[-1][2],euler_angles[0], euler_angles[1],euler_angles[2] ])
                print(pose_from_euler)
# #not nessesary lol convert to using robodk 
def create_ht_matrix(x, y, z, psi, phi, theta):
    rot_z = sp.Rotation.from_euler('xyz', [psi, phi, theta]).as_matrix()
    ht_matrix = np.eye(4)
    ht_matrix[:3, :3] = rot_z
    ht_matrix[:3, 3] = [x, y, z]
    return ht_matrix

def inverse_ht_matrix(x, y, z, psi, phi, theta):
    rot_z = sp.Rotation.from_euler('xyz', [psi, phi, theta]).as_matrix()
    inv_ht_matrix = np.eye(4)
    inv_ht_matrix[:3, :3] = rot_z.T
    inv_ht_matrix[:3, 3] = -rot_z.T @ np.array([x, y, z])
    return inv_ht_matrix


target = create_ht_matrix(-591,-215.5,214/1,0,90*np.pi/180,0)
tool_correction = inverse_ht_matrix(0,0,102.82, 0,50*np.pi/180,0)
pose = target @ tool_correction
print_np_array(pose)


def print_np_array(matrix, decimals=6):
    """
    Print a NumPy array in a copy-pasteable np.array([...]) format.
    
    Parameters:
        matrix (np.ndarray): The matrix to print.
        decimals (int): Number of decimal places for formatting.
    """
    formatted = np.array2string(matrix, separator=', ', precision=decimals, suppress_small=True)
    print(f"np.array({formatted})")



    points_df['Parent'] = None #NOTE if a key has no perant, then it is in the world frame

#cup despencer (key 1) has local frame key 16 with children keys 17-20
points_df.loc[points_df['Frame'].isin([16]), 'Parent'] = 1
points_df.loc[points_df['Frame'].isin([17, 18, 19, 20]), 'Parent'] = 1
#Mazzer (key 3) has local frame key 21 with children keys 22-26
points_df.loc[points_df['Frame'].isin([21]), 'Parent'] = 3
points_df.loc[points_df['Frame'].isin([22, 23, 24, 25, 26]), 'Parent'] = 3
#Mazzer scale (key 6) has local frame key 27 with children keys 28-43
points_df.loc[points_df['Frame'].isin([27]), 'Parent'] = 6
points_df.loc[points_df['Frame'].isin([28, 29, 30, 31]), 'Parent'] = 6
#The tamper (key 8) has local frame key 32 with children keys 33-34
points_df.loc[points_df['Frame'].isin([32]), 'Parent'] = 8
points_df.loc[points_df['Frame'].isin([33, 34]), 'Parent'] = 8
#Rancilio (key 10) has local frame key 35 with children keys 36-38
points_df.loc[points_df['Frame'].isin([35]), 'Parent'] = 10
points_df.loc[points_df['Frame'].isin([36, 37, 38]), 'Parent'] = 10
#Rancilio scale (key 12) has local frame key 39 with children keys 40-43
points_df.loc[points_df['Frame'].isin([39]), 'Parent'] = 12
points_df.loc[points_df['Frame'].isin([40, 41, 42, 43]), 'Parent'] = 12
#WDT (key 14) has local frame key 44 with children keys 45-46
points_df.loc[points_df['Frame'].isin([44]), 'Parent'] = 14
points_df.loc[points_df['Frame'].isin([45, 46]), 'Parent'] = 14
#Tool cleaner (key 56) has frame key 58 with children keys 59-61
points_df.loc[points_df['Frame'].isin([58]), 'Parent'] = 56
points_df.loc[points_df['Frame'].isin([59, 60, 61]), 'Parent'] = 56

#ok these points are defined from the toolhead so will be left with their perant being their origin, not the world frame pose of their origin
#Cup tool has local frame key 47 with children keys 48-49
points_df.loc[points_df['Frame'].isin([47]), 'Parent'] = None
points_df.loc[points_df['Frame'].isin([48, 49]), 'Parent'] = 47
#Mazzer tool has local frame key 50 with children keys 51-52
points_df.loc[points_df['Frame'].isin([50]), 'Parent'] = None
points_df.loc[points_df['Frame'].isin([51, 52]), 'Parent'] = 50
#Rancilio tool has local frame key 53 with children keys 54-55
points_df.loc[points_df['Frame'].isin([53]), 'Parent'] = None
points_df.loc[points_df['Frame'].isin([54, 55]), 'Parent'] = 53

points_df['Relitive Rotation'] = points_df.get('rotation', 0)




def create_points_df(robot_name = 'robot_3', filepath = 'robodk_stations/points.xlsx'):


    points_df = pd.read_excel(filepath, sheet_name=robot_name)

    #exsiting columns are Key, Frame, Fixture, X, Y, Z, Location (that may contain o'clock)

    #So. Perant is just the "Fixture" coulmn. The global positon of each perant is the "X", "Y", "Z" position of the row with Fixture = "Perant", Frame = "World" and Origin = "Yes"
    #Add rotation columns for each axiis
    #All Keys have zero rotation except the global position of each perant frame

    #Rows in the Frame = 'World' and Origin = 'No' define the rotation of rows with the same Fixture as the Frame value
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
    points_df['theta_x'] = 0.0 
    points_df['theta_y'] = 0.0
    points_df['theta_z'] = 0.0
    for i, row in points_df.iterrows():
        if row['Frame'] == 'world' and str(row.get('Origin', '')).lower() == 'no':
            fixture = row['Fixture']
            print(f"Processing rotation for fixture '{fixture}' at row {i +1}")
            # --- Find parent origin row ---
            parent_origin = points_df[
                (points_df['Fixture'] == fixture) &
                (points_df['Frame'] == 'world') &
                (points_df['Origin'].str.lower() == 'yes')
            ]

            if parent_origin.empty:
                print(f"Warning: No origin found for fixture '{fixture}'")
                continue

            parent_row = parent_origin.iloc[0] # There should be only one
            print(f"Row {i+1}: Found parent origin at row {parent_row.name + 1}")
            # --- Position vectors ---
            p_origin = np.array([parent_row['X'], parent_row['Y'], parent_row['Z']])
            p_child  = np.array([row['X'], row['Y'], row['Z']])
            print(f"Row {i+1}: Parent origin position = {p_origin}, Child position = {p_child}")
            v = p_child - p_origin  # direction vector in world coords

            # Simplest assumption: the axis of rotation is the dominant axis perpendicular
            # to v. For now, choose the world axis with smallest alignment with v.
            world_axes = {
                'x': np.array([1, 0, 0]),
                'y': np.array([0, 1, 0]),
                'z': np.array([0, 0, 1])
            }

            dot_vals = {k: abs(np.dot(v, ax)) for k, ax in world_axes.items()}
            rot_axis = min(dot_vals, key=dot_vals.get)  # least aligned axis
            print(f"Row {i+1}: Chosen rotation axis = {rot_axis}")

            # --- Normalise v ---
            if np.linalg.norm(v) == 0:
                continue
            v = v / np.linalg.norm(v)   
            print(f"Row {i+1}: Direction vector v = {v}")

            # to find angle project v onto the plane perpendicular to rot_axis
            if rot_axis == 'x':
                v_proj = np.array([0, v[1], v[2]])
                ax1 = np.array([0, 1, 0])  # y-axis as reference
                ax2 = np.array([0, 0, 1])  # z-axis as reference
            elif rot_axis == 'y':
                v_proj = np.array([v[0], 0, v[2]])
                ax1 = np.array([1, 0, 0])  # x-axis as reference
                ax2 = np.array([0, 0, 1])  # z-axis as reference
            else:  # rot_axis == 'z'
                v_proj = np.array([v[0], v[1], 0])
                ax1 = np.array([1, 0, 0])  # x-axis as reference
                ax2 = np.array([0, 1, 0])  # y-axis
                
            if np.linalg.norm(v_proj) == 0:
                continue

            #procet onto both axes and use atan2 to get the angle
            theta = np.arctan2(-np.dot(v_proj, ax1), np.dot(v_proj, ax2))
            print(f"Row {i+1}: Projected vector v_proj = {v_proj}, angle theta = {np.rad2deg(theta)} degrees")
            print(f"Row {i+1}: Relative rotaion (deg) = {np.rad2deg(row['Relitive Pos'])}")
            # --- Apply relative rotation ---
            if rot_axis == 'x':
                points_df.at[i, 'theta_x'] = theta + row['Relitive Pos'] # so if a point is at 3 o'clock we have just caluclated the angle to the 3 o'clock positon, wich is -90 degrees 
            elif rot_axis == 'y':
                points_df.at[i, 'theta_y'] = theta + row['Relitive Pos']
            else:  # rot_axis == 'z'
                points_df.at[i, 'theta_z'] = theta + row['Relitive Pos']
            print(f"Row {i+1}: Updated rotation angles - Rx: {np.rad2deg(points_df.at[i, 'theta_x'])}, Ry: {np.rad2deg(points_df.at[i, 'theta_y'])}, Rz: {np.rad2deg(points_df.at[i, 'theta_z'])}")
    return points_df












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


def create_points_df(robot_name = 'robot_3', filepath = 'robodk_stations/points.xlsx'):


    points_df = pd.read_excel(filepath, sheet_name=robot_name)

    #exsiting columns are Key, Frame, Fixture, X, Y, Z, Location (that may contain o'clock)

    #So. Perant is just the "Fixture" coulmn. The global positon of each perant is the "X", "Y", "Z" position of the row with Fixture = "Perant", Frame = "World" and Origin = "Yes"
    #Add rotation columns for each axiis
    #All Keys have zero rotation except the global position of each perant frame

    #Rows in the Frame = 'World' and Origin = 'No' define the rotation of rows with the same Fixture as the Frame value
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
    points_df['theta_x'] = 0.0 
    points_df['theta_y'] = 0.0
    points_df['theta_z'] = 0.0
    for i, row in points_df.iterrows():
        if row['Frame'] == 'world' and str(row.get('Origin', '')).lower() == 'no':
            fixture = row['Fixture']
            location = row['Location']
            print(f"Processing rotation for fixture '{fixture}' at row {i +1}")
            # --- Find parent origin row ---
            parent_origin = points_df[
                (points_df['Fixture'] == fixture) &
                (points_df['Frame'] == 'world') &
                (points_df['Origin'].str.lower() == 'yes')
            ]

            if parent_origin.empty:
                print(f"Warning: No origin found for fixture '{fixture}'")
                continue

            parent_row = parent_origin.iloc[0] # There should be only one
            print(f"Row {i+1}: Found parent origin at row {parent_row.name + 1}")
            # --- Position vectors ---
            p_origin = np.array([parent_row['X'], parent_row['Y'], parent_row['Z']])
            p_child  = np.array([row['X'], row['Y'], row['Z']])
            print(f"Row {i+1}: Parent origin position = {p_origin}, Child position = {p_child}")
            v = p_child - p_origin  # direction vector in world coords
            if np.linalg.norm(v) == 0:
                continue
            v = v / np.linalg.norm(v)   

            #now deterimine what plane this vector lies in in the local frame.
            child_local = points_df[
                (points_df['Frame'] == 'local') &
                (points_df['Fixture'] == fixture) &
                (points_df['Location'] == location)
            ]

            child_local = child_local.iloc[0] # There should be only one
            #rotation axis is the coordinate in local frame that is zero
            if child_local['X'] == 0:
                print(f"Row {i+1}: Child local pos X=0, rotation in YZ plane")
                local_plane = 'YZ'
            elif child_local['Y'] == 0:
                print(f"Row {i+1}: Child local pos Y=0, rotation in XZ plane")
                local_plane = 'XZ'
            elif child_local['Z'] == 0:
                print(f"Row {i+1}: Child local pos Z=0, rotation in XY plane")
                local_plane = 'XY'
            # --- Determine rotation axis ---               


            # Simplest assumption: the axis of rotation is the dominant axis perpendicular
            # to v. For now, choose the world axis with smallest alignment with v.
            world_planes = {
                'YZ': np.array([1, 0, 0]),
                'XZ': np.array([0, 1, 0]),
                'XY': np.array([0, 0, 1])
            }

            dot_vals = {k: abs(np.dot(v, ax)) for k, ax in world_planes.items()}
            global_plane = min(dot_vals, key=dot_vals.get)  # least aligned axis
            print(f"Row {i+1}: Global plane = {global_plane}")

            # to find angle project v onto the plane perpendicular to rot_axis
            if global_plane == 'YZ':
                v_proj = np.array([0, v[1], v[2]])
                ax1 = np.array([0, 1, 0])  # y-axis as reference
                ax2 = np.array([0, 0, 1])  # z-axis as reference
            elif global_plane == 'XZ':
                v_proj = np.array([v[0], 0, v[2]])
                ax1 = np.array([1, 0, 0])  # x-axis as reference
                ax2 = np.array([0, 0, 1])  # z-axis as reference
            else:  # rot_axis == 'z'
                v_proj = np.array([v[0], v[1], 0])
                ax1 = np.array([1, 0, 0])  # x-axis as reference
                ax2 = np.array([0, 1, 0])  # y-axis
                
            if np.linalg.norm(v_proj) == 0:
                continue

            print(f"Row {i+1}: Direction vector v = {v}, Projected vector v_proj = {v_proj}")

        
            #procet onto both axes and use atan2 to get the angle
            theta = np.arctan2(-np.dot(v_proj, ax1), np.dot(v_proj, ax2))
            print(f"Row {i+1}: Angle = {np.rad2deg(theta)} deg")
            print(f"Row {i+1}: Relative rotaion = {np.rad2deg(row['Relitive Pos'])} deg")
            # --- Apply relative rotation  to perant frame---
            if global_plane == 'YZ':
                points_df.at[parent_row.name, 'theta_x'] = theta + row['Relitive Pos'] # so if a point is at 3 o'clock we have just caluclated the angle to the 3 o'clock positon, wich is -90 degrees 
            elif global_plane == 'XZ':
                points_df.at[parent_row.name, 'theta_y'] = theta + row['Relitive Pos']
            else:  # rot_axis == 'z'
                points_df.at[parent_row.name, 'theta_z'] = theta + row['Relitive Pos']
            print(f"Row {parent_row.name+1}: Updated rotation angles - Rx: {np.rad2deg(points_df.at[parent_row.name, 'theta_x'])}, Ry: {np.rad2deg(points_df.at[parent_row.name, 'theta_y'])}, Rz: {np.rad2deg(points_df.at[parent_row.name, 'theta_z'])}")
    return points_df



            



  





def get_global_pose(frame_df,frame_key, pos_x = 0, pos_y = 0, pos_z = 0, psi = 0, phi = 0, theta = 0):
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
    # print(frame_df.iloc[frame_key-1])
    frame_row = frame_df.loc[frame_df['Key'] == frame_key].iloc[0]
    frame_ht = TxyzRxyz_2_Pose([
        frame_row['X'],
        frame_row['Y'],
        frame_row['Z'],
        frame_row.get('theta_x', 0),
        frame_row.get('theta_w', 0),
        frame_row.get('theta_z', 0)]
    )
    
    # Compute the transform of this frame relative to its parent
    combined_ht = frame_ht * local_ht
    
    parent_key = None
    
    if parent_key is None or pd.isna(parent_key):
        return combined_ht
    else:
        # Recursively compute the parent global transform
        return get_global_pose(parent_key, 0, 0, 0, 0) @ combined_ht




if __name__ == "__main__":
    frames = create_points_df()

    target = TxyzRxyz_2_Pose([-589,-221.1,212.9,0,0,90*np.pi/180])
    TCP = TxyzRxyz_2_Pose([0,0,102.82, np.pi,0*np.pi/180,0])
    inv_TCP = robomath.invH(TCP)
    pose = target * inv_TCP
    print(target)


    Cup_Frame = get_global_pose(frames,idx.Cup_Frame)
    print(Cup_Frame)

    



def create_points_df(robot_name = 'robot_3', filepath = 'robodk_stations/points.xlsx'):


    points_df = pd.read_excel(filepath, sheet_name=robot_name)

    #exsiting columns are Key, Frame, Fixture, X, Y, Z, Location (that may contain o'clock)

    #So. Perant is just the "Fixture" coulmn. The global positon of each perant is the "X", "Y", "Z" position of the row with Fixture = "Perant", Frame = "World" and Origin = "Yes"
    #Add rotation columns for each axiis
    #All Keys have zero rotation except the global position of each perant frame


    #find the plane wich each rotaion is defined in and then apply that to the world origin for that fixture
    # do this for all non origin world frames
    #set the quaternion for the origin world frames to 1,0,0,0
    points_df['Qw'] = np.nan
    points_df['Qx'] = np.nan
    points_df['Qy'] = np.nan
    points_df['Qz'] = np.nan
    
    for i, row in points_df.iterrows():
        if row['Frame'] == 'world' and str(row.get('Origin', '')).lower() == 'yes':
            #this is a perant frame, lets caluclate its rotation
            fixture = row['Fixture']
            location = row['Location']
            # --- Find children that define the rotation (Global Frame) ---
            children = points_df[
                (points_df['Fixture'] == fixture) &
                (points_df['Frame'] == 'world') &
                (points_df['Origin'].str.lower() == 'no')
            ]

            if children.empty:
                print(f"Row {i+1}: No points defineing rotation, frame is not rotated compared to origin")
                continue
            
            print(f"Row {i+1}: Found {len(children)} child(ren) defining rotation")

            #for each child, find their coresponding point in the local frame to determine what plane they lie in
            for j, child_row in children.iterrows():
                print(f"  Processing child row {j+1}")
                #find the child's local frame definition
                location = child_row['Location']
                local_frame = points_df[
                    (points_df['Location'] == location) &
                    (points_df['Frame'] != 'world')
                ]
                if local_frame.empty:
                    print(f"        No local frame found for child row {j+1}, skipping")
                    continue
                local_frame = local_frame.iloc[0]
                print(f"        Found local frame for child row {j+1}: row {local_frame.name+1}")

                #first check if point is inline with an axis
                axis_aligned = False
                planar = False
                if local_frame['X'] == 0 and local_frame['Y'] == 0:
                    print(f"        Point is inline with local Z axis")
                    axis_aligned = True
                    planar = True
                if local_frame['Y'] == 0 and local_frame['Z'] == 0:
                    print(f"        Point is inline with local X axis")
                    axis_aligned = True
                    planar = True
                if local_frame['X'] == 0 and local_frame['Z'] == 0:
                    print(f"        Point is inline with local Y axis")
                    axis_aligned = True
                    planar = True    
                
                
                if not axis_aligned: # the point is not inline with an axis so we need determine the plane it lies in
                    if local_frame['X'] == 0:
                        print(f"        Point lies in local YZ plane")
                        planar = True
                        plane = 'YZ'
                    elif local_frame['Y'] == 0:
                        print(f"        Point lies in local XZ plane")
                        planar = True
                        plane = 'XZ'
                    elif local_frame['Z'] == 0:
                        print(f"        Point lies in local XY plane")
                        planar = True
                        plane = 'XY'
                    else:
                        print(f"        Point does not lie in a loacl plane, what the shit")
                
                    #calculate the angle of the point in the XY plane

                    if axis_aligned and planar: 
                        #this is the case for points inline with an axis, calculate the quaternion defining the rotation
                        pass


                    if not axis_aligned and planar:
                        #this is the case for points in a plane, calculate the quaternion defining the rotation
                        pass




    return points_df