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