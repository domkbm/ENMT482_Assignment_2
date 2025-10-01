# """
# Author Oliver Handforth
# UC Mechatronics
# Date 08/09/25

# NOTE THESE PATHS ARE NOT NESSESARY IF YOU SET YOUR DEFUALT VSC PYTHON INTERPERTER TO C:\Apps\RoboDK\Python-Embedded\python.exe
# (then you just have to press play with the robodk already open with the corret bench (ie robot 3))

# path to open RoboDk env
# C:\Apps\RoboDK\Python-Embedded

# path to run toolpath (for me lol)
# C:\Users\dmc284\OneDrive - University of Canterbury\2025\ENMT482\Assignment 2\Code\group307toolpath.py
# path to run example (for me lol)
# "C:\Users\dmc284\OneDrive - University of Canterbury\2025\ENMT482\Assignment 2\Code\robodk_stations\robodk_basics.py"


# For each of the Demonstration tasks below, we will receive 1 mark for each task correctly demonstrated.
# Collisions with equipment will be penalised at -1 per collision, up to a maximum of -3.
# Robot or tool moving outside the bounding box will receive a penalty of -1 for the demonstration.
# Note: we cannot receive a negative mark for either demonstration.

#  C:\RoboDK\Python-Embedded\python.exe -m pip install scipy openpyxl
# & "C:\Apps\RoboDK\Python-Embedded\python.exe" -m pip install scipy openpyxl
# """

import numpy as np
import scipy as sp
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

from robodk.robodialogs import *
from modbus_scale_client import modbus_scale_client


import tools
import indices as id
import transforms as tf

CORRECT_WEIGHT = 32 # Grams
GROUNDS_WEIGHT_TARGET = 19.9


#Visualize fcns
def list_program_items(RDK):
    # Returns Item objects, then call .Name()
    prog_items = RDK.ItemList(ITEM_TYPE_PROGRAM, False)
    for p in prog_items:
        print(p.Name(), "(type", p.Type(), ")")
    return prog_items

def run_visual_program(RDK, name, blocking=True):
    prog = RDK.Item(name, ITEM_TYPE_PROGRAM)
    if not prog.Valid():
        raise ValueError(f"Program '{name}' not found")
    prog.RunCode()
    if blocking:
        prog.WaitFinished()


RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
tls = tools.Tools(RDK)
mazzer_scale =  modbus_scale_client.ModbusScaleClient(host = id.IP_MAZZER_3)
if mazzer_scale.server_exists() == False:
    if RUNMODE_SIMULATE:
        print("Mazzer scale not detected, output will be simulated.")
    else:
        RDK.ShowMessage("Mazzer scale not detected, output will be simulated.")
rancilio_scale =  modbus_scale_client.ModbusScaleClient(host = id.IP_RANCILIO_3)
if mazzer_scale.server_exists() == False:
    if RUNMODE_SIMULATE:
        print("Rancilio scale not detected, output will be simulated.")
    else:
        RDK.ShowMessage("Rancilio scale not detected, output will be simulated.")

mazzer_tool = RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL) 
rancilio_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL) 

#get all the frames
points_df = tf.create_points_df()

# reset the sim
robot_program = RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

#poses
basket_drop_pose = tf.pose(points_df, id.Mazzer_Scale_Ball, tool=id.Rancillio_Indent, theta_y=90, theta_z=180, pos_z=-1)
above_wdt_pose = tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, off_x=-50, pos_y = 1)
wdt_pose = tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, pos_y = 1)


####HELPER FCNS TO SPIN THE BASKET IN THE MACHINE#####
spin_start_pose = tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool, pos_z=0, theta_y=-90, off_theta_x=45)
arc = tf.generate_circular_path(spin_start_pose, tf.pose(points_df, id.Rancillio_Gasket), 45)
spin_end_pose = arc[-1]
def basket_spin_fwd():
    UR5.MoveL(tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool, pos_z=-15, theta_y=-90, off_theta_x=45))
    UR5.MoveL(spin_start_pose)
    UR5.MoveC(arc[1], arc[2])

def basket_spin_bkwd():
    # UR5.MoveL(spin_end_pose) # if coming from somwhere else
    UR5.MoveC(arc[1], spin_start_pose)
    UR5.MoveL(tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool, pos_z=-15, theta_y=-90, off_theta_x=45))

def A(): #TODO a) Pick up the Rancilio tool and place it on the Mazzer Scale pan.
    tls.rancilio_tool_attach_l_ati()
    UR5.MoveJ([146.160000, -80.140000, 137.450000, -63.010000, 115.440000, -220])
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Ball, tool=id.Rancillio_Indent, theta_y=90, theta_z=180, pos_z = 6))
    UR5.MoveL(basket_drop_pose, blocking=True)
    tls.student_tool_detach()
    run_visual_program(RDK, 'Show_Mazzer_Scale_Rancilio_Tool', blocking=True) #put the tool on the scales (visual) 
    rancilio_tool.setVisible(False, False) #take the tool of the toolhead (visual)
    UR5.MoveJ([146.390000, -80.000000, 137.570000, -62.720000, 116.640000, -220])#intermeidate point to avoid the mazzer

def B(): #TODO b) Use the Mazzer tool to unlock the Mazzer Scale.
    tls.mazzer_tool_attach_l_ati()
    UR5.MoveJ([117.580000, -93.770000, 117.650000, -113.010000, -89.130000, -220]) #intermeidate point to avoid the mazzer
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=50,theta_x=-120,off_theta_z=150), blocking=True)
    # print("move to above the lock")
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=4,theta_x=-120,off_theta_z=150), blocking=True)
    # print("down to unlock the lock")
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -22, pos_z=-8,theta_x=-120,off_theta_z=150), blocking=True)
    # print("down and across to unlock the lock")
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -22, pos_z=50,theta_x=-120, off_theta_z=150), blocking=True)
    # print("leave lock")

def C(): #TODO c) Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.
    # UR5.MoveJ([117.580000, -93.770000, 117.650000, -113.010000, -89.130000, -204.190000]) #intermeidate point (not needed atm) to avoid the mazzer
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_On_Button, tool=id.Mazzer_Tip_Tool, pos_y=10,theta_y=-110, off_z= 0), blocking=True) #go to on button
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_On_Button, tool=id.Mazzer_Tip_Tool, pos_y=10,theta_y=-110, off_z= -10), blocking=True) #push on button
    the_time = time.time()
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_On_Button, tool=id.Mazzer_Tip_Tool, pos_y=10,theta_y=-110, off_z= 0), blocking=True)
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_Off_Button, tool=id.Mazzer_Tip_Tool, pos_y=15,theta_y=-115, off_x = 10, off_z=0), blocking=True) #go to off button
    while (time.time() - the_time) < 1: # wait 15 secs
        # print(f"Waited {time.time() - the_time:.1f} s")
        pass # do nothing
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_Off_Button, tool=id.Mazzer_Tip_Tool, pos_y=15,theta_y=-115, off_x = 10, off_z=-10), blocking=True) #push 
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_Off_Button, tool=id.Mazzer_Tip_Tool, pos_y=15,theta_y=-115, off_x = 10, off_z=50), blocking=True) #go to off button

def D(): #TODO d) Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of coffee grounds has been deposited in the Rancilio tool.
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_Lever, tool=id.Mazzer_Bar_Tool, theta_x=-190, off_x= 9, off_y= 30, off_z = -12))
    circle_start_pose = tf.pose(points_df, id.Mazzer_Lever, tool=id.Mazzer_Bar_Tool, theta_x=-200, off_x= 8, off_y= 30, off_z = -7)
    UR5.MoveJ(circle_start_pose, blocking=True)

    circular_path = tf.generate_circular_path(circle_start_pose, tf.pose(points_df, id.Mazzer), -65, n_steps=2)
    mazzer_scale.tare()
    weight = mazzer_scale.read() #this is a bit weird but maby add a difrencing thing. 
    i = 0
    while weight <= GROUNDS_WEIGHT_TARGET:
        i += 1
        # Forward movement
        UR5.MoveC(circular_path[1], circular_path[-1], blocking=False)
        ii = 0
        print(f"robot still moving {UR5.Busy()}")

        
        while UR5.Busy():
            ii += 1
            weight = mazzer_scale.read()
            print(f'weight {weight}, loop {i}.0{ii}')
            if weight >= GROUNDS_WEIGHT_TARGET:
                UR5.Stop()
                print('ohoh')
                break
        if weight >= GROUNDS_WEIGHT_TARGET:
            break


        # weight += 7 # TODO remove
        time.sleep(0.5)
        UR5.MoveC(circular_path[1], circle_start_pose, blocking=False)
        ii = 0
        while UR5.Busy():
            ii += 1
            weight =  mazzer_scale.read()
            print(f'weight {weight}, loop {i}.{ii}0')
            if weight >= GROUNDS_WEIGHT_TARGET:
                UR5.Stop()
                break
        if weight >= GROUNDS_WEIGHT_TARGET:
            break
    #move up to let go of the lever
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([0,0,60,0,0,0]) * UR5.Pose(), blocking=True)

def E():# TODO e) Use the Mazzer tool to lock the Mazzer Scale.
    # UR5.MoveJ([117.580000, -93.770000, 117.650000, -113.010000, -89.130000, -204.190000]) #intermeidate point to avoid the mazzer
    # UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=50,theta_x=-120, off_theta_z=180), blocking=True)
    # print("move to above the lock")
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -3, pos_z=8,theta_x=-120, off_theta_z=180), blocking=True)
    # print("down to unlock the lock")
    UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= 10, pos_z=-3,theta_x=-120, off_theta_z=180), blocking=True)
    # print("down and across to unlock the lock")

def F(): #TODO f) Remove the Rancilio tool from the Mazzer.
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ([146.390000, -80.000000, 137.570000, -62.720000, 116.640000, 140.000000])#intermeidate point to avoid the mazzer
    UR5.MoveL(basket_drop_pose, blocking=True)
    tls.student_tool_attach()
    run_visual_program(RDK, 'Hide_Mazzer_Scale_Rancilio_Tool', blocking=True) #dissapear the tool from the scales (visual)
    rancilio_tool.setVisible(True,False) #put it on the toolhead (visual)
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([25,0,0,0,0,0]) * basket_drop_pose, blocking=True)

def G(): #TODO g) Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
    tls.wdt_open()
    UR5.MoveJ([122.700000, -94.510000, 152.060000, -59.680000, 52.670000, 141.300000])
    UR5.MoveL(above_wdt_pose, blocking=True)
    UR5.MoveL(wdt_pose, blocking=True)

def H(): #TODO h) Release the Rancilio tool and close the WDT fixture.
    tls.student_tool_detach()
    run_visual_program(RDK, 'Show_WDT_Rancilio_Tool', blocking=True) #the tool in the wdt (visual)
    rancilio_tool.setVisible(False,False) #dissaper it from the toolhead (visual)
    tls.wdt_shut()
    UR5.MoveL(tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, off_z=50), blocking=True)

def I(): #TODO i) Use the Mazzer tool to turn the WDT rotor five full revolutions.
    REVOLUTIONS = 2
    tls.mazzer_tool_attach_l_ati()
    UR5.MoveL(tf.pose(points_df, id.WDT_Spinner, tool=id.Mazzer_Tip_Tool, theta_x=-180, off_z=20), blocking=True)
    circle_start_pose = tf.pose(points_df, id.WDT_Spinner, tool=id.Mazzer_Tip_Tool, theta_x=-180, off_z=6)
    UR5.MoveL(circle_start_pose, blocking=True)
    circular_path = tf.generate_circular_path(circle_start_pose, tf.pose(points_df, id.WDT), REVOLUTIONS*360, n_steps=REVOLUTIONS*4, spin_tool=False)
    for i in range(REVOLUTIONS): # do some spinnning
        UR5.MoveC(circular_path[4*i+1], circular_path[4*i+2], blocking=True)
        # time.sleep(0.5)
        UR5.MoveC(circular_path[4*i+3], circular_path[4*i+4], blocking=True)
    tls.mazzer_tool_detach_l_ati()

def J():#TODO j) Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.
    tls.wdt_open()
    UR5.MoveJ([122.700000, -94.510000, 152.060000, -59.680000, 52.670000, -217.56])
    UR5.MoveL(tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, off_z=50), blocking=True)
    UR5.MoveL(wdt_pose, blocking=True)
    tls.student_tool_attach()
    run_visual_program(RDK, 'Hide_WDT_Rancilio_Tool', blocking=True) #tool ouff the wdt (visual)
    rancilio_tool.setVisible(True,False) #put it back on the toolhead (visual)
    UR5.MoveL(above_wdt_pose)


def K():#TODO k. Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine tamps the coffee grounds.
    UR5.MoveJ([110.110000, -93.140000, 147.240000, -51.240000, 14.470000, 140.000000])
    UR5.MoveJ(tf.pose(points_df, id.PUQ_Clamp, tool=id.Rancillio_Basket_Tool, theta_x = -90,theta_y=-90, theta_z=-90, pos_x=80, off_z=-2), blocking=True)
    UR5.MoveL(tf.pose(points_df, id.PUQ_Clamp, tool=id.Rancillio_Basket_Tool, theta_x = -90,theta_y=-90, theta_z=-90, off_x=3, off_z=-2), blocking=True)

def L():#TODO l. Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group head.
    UR5.MoveL(tf.pose(points_df, id.PUQ_Clamp, tool=id.Rancillio_Basket_Tool, theta_x = -90,theta_y=-90, theta_z=-90, pos_x=75, off_z=-2), blocking=True)
    UR5.MoveJ([-20.420000, -87.420000, 136.600000, -50.120000, -32.130000, 140.560000])
    UR5.MoveJ([32.440000, -84.510000, 131.920000, -49.900000, 72.850000, 141.770000])
    UR5.MoveJ(robomath.TxyzRxyz_2_Pose([0,0,-10,0,0,0]) * spin_start_pose)
    basket_spin_fwd()
    tls.student_tool_detach()
    run_visual_program(RDK, 'Show_Rancilio_Rancilio_Tool_Rotated', blocking=True)
    rancilio_tool.setVisible(False, False)
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([0,200,0,0,0,0]) * UR5.Pose(), blocking=True)


def M():# #TODO m) Use the Mazzer tool to operate the cup dispenser.
    tls.mazzer_tool_attach_l_ati()
    UR5.MoveJ([112.020000, -64.930000, 130.100000, -223.900000, -70.020000, 230.210000]) #another intermeidiate point so we dont hit the tool holder
    UR5.MoveJ([71.680000, -64.390000, 129.270000, -262.950000, -88.330000, 214.230000]) #another intermeidiate point so we dont hit the tool holder
    UR5.MoveJ([9.594445, -72.709382, 129.539942, -234.755204, -96.626572, 259.018886])
    UR5.MoveJ(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool, pos_x=50), blocking=True)
    print("move to above the latch")
    time.sleep(1)
    UR5.MoveL(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool), blocking=True)
    print("into latch")
    time.sleep(1)
    UR5.MoveL(tf.pose(points_df, id.Cup_Open, tool=id.Mazzer_Tip_Tool), blocking=True)
    print("open") 
    run_visual_program(RDK, 'Show_Cup_Dispenser_Open')
    time.sleep(1)
    UR5.MoveL(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool), blocking=True)
    print("close")
    run_visual_program(RDK, 'Show_Cup_Dispenser_Shut')
    time.sleep(1)
    UR5.MoveL(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool, pos_x=50), blocking=True)
    print("move to above the latch")
    time.sleep(1)
    run_visual_program(RDK, 'Show_Cup_Dispenser_Cup')
    UR5.MoveJ([47.010000, -72.190000, 130.980000, -238.790000, -96.780000, 269.990000]) #another intermeidiate point so we dont hit the cup stack
    tls.mazzer_tool_detach_r_ati() 

def N():
    run_visual_program(RDK, 'Show_Cup_Dispenser_Cup')
    tls.cup_tool_attach_l_ati()
    UR5.MoveJ([-21.230000, -94.640000, 78.390000, -69.030000, 95.230000, -220.420000])
    UR5.MoveJ([-21.230000, -98.640000, 138.390000, -37.030000, 95.230000, -220.430000])
    tls.cup_tool_open_ur5()
    UR5.MoveJ([6.780000, -98.640000, 138.390000, -37.030000, 95.230000, -220.430000])

    UR5.MoveL(tf.pose(points_df, id.Cup_Coffee, tool=id.Cup_Holder_Top_Face_Centre_Open, pos_x=-25), blocking=True)
    tls.cup_tool_shut_ur5()
    run_visual_program(RDK, 'Hide_Cup_Dispenser_Cup')
    run_visual_program(RDK, 'Show_Cup_Tool_Cup')
    UR5.MoveL(tf.pose(points_df, id.Cup_Coffee, tool=id.Cup_Holder_Top_Face_Centre_Open), blocking=True)
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([100,0,0,0,0,0]) * UR5.Pose(), blocking=True)
    
    # Go to the cup approach postison
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale_Centre, tool=id.Cup_Closed_Tool, pos_z=74, theta_y= 90, theta_x=90, theta_z=90), blocking=True)
    tls.cup_tool_open_ur5()
    run_visual_program(RDK, 'Show_Rancilio_Scale_Cup', blocking=True)
    run_visual_program(RDK, 'Hide_Cup_Tool_Cup', blocking=True) 
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale_Centre, tool=id.Cup_Closed_Tool, pos_z=76, pos_x=-100, theta_y= 90, theta_x=90, theta_z=90), blocking=True)
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_l_ati()




def O(): # Use the Mazzer tool to unlock the Rancilio Scale.
    # PART O - Ready to Test  
    # intermeidate point to avoid the tool holder
    tls.mazzer_tool_attach_l_ati()
    UR5.MoveJ([88.444300, -105.969638, 109.059761, -96.960870, -110.438692, -144.563103]) 

    UR5.MoveJ([55.405011, -71.147417, 121.402556, -145.965266, -139.759830, -47.462793])    
    # Postion before unlocking scales
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5, pos_y=21.53, pos_z=0, theta_x = 125), blocking=True)
    # Flipping Ranccilio Scale Switch
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5-20, pos_y=21.53, pos_z=-25, theta_x = 125), blocking=True)

def P(): # Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports 32±0.1g of water has been dispensed in the cup.

    UR5.MoveJ([58.312263, -80.285462, 115.871894, -129.070049, -138.163083, -46.417414])

    # PART P - Ready to Test
    UR5.MoveJ([16.840846, -94.755605, 114.632103, -26.459245, 35.176778, -42.243328])

    # Press Switch in the Rancillio Pose
    UR5.MoveJ(tf.pose(points_df, 37, tool=id.Mazzer_Tip_Tool, pos_x = 15, pos_z = 5, theta_x=0, theta_y=-90, theta_z=180), blocking=True)
    print("Preparing to press")

    # Press Button
    UR5.MoveJ(tf.pose(points_df, 37, tool=id.Mazzer_Tip_Tool,  pos_x = -5, pos_z = 5, theta_x=0, theta_y=-90, theta_z=180), blocking=True)
    print("Pressing Button Up")

    weight = 0
    rancilio_scale.tare()
    print("Tared Scale")
    while (weight >= CORRECT_WEIGHT):
        weight = rancilio_scale.read()
        print(f"Current Weight: {weight}g")

    # Press Button
    UR5.MoveJ(tf.pose(points_df, 37, tool=id.Mazzer_Tip_Tool,  pos_x = -5, pos_z = -5, theta_x=0, theta_y=-90, theta_z=180), blocking=True)
    print("Pressing Button Up")

    # # Pull Tool Back
    UR5.MoveJ(tf.pose(points_df, 37, tool=id.Mazzer_Tip_Tool, pos_x = 15, pos_z = 5, theta_x=0, theta_y=-90, theta_z=180), blocking=True)
    print("Preparing to press")
        
def Q(): #Use the Mazzer tool to lock the Rancilio Scale.
    # PART Q - Ready to Test

    UR5.MoveJ([58.312263, -80.285462, 115.871894, -129.070049, -138.163083, -46.417414])
    UR5.MoveJ([55.405011, -71.147417, 121.402556, -145.965266, -139.759830, -47.462793])    

    # Postion before unlocking scales
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5, pos_y=21.53, pos_z=0, theta_x = 125), blocking=True)
    # Flipping Ranccilio Scale Switch
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5+20, pos_y=+25, pos_z=-25, theta_x = 135), blocking=True)

    UR5.MoveJ([88.444300, -105.969638, 109.059761, -96.960870, -110.438692, -144.563103]) 

    tls.mazzer_tool_detach_l_ati()

def S(): # Remove the Rancilio tool from the group head.
    
    # tls.rancilio_tool_attach_l_ati()
    UR5.MoveJ([24.744220, -85.109034, 125.337227, -42.040277, -290.254345, -219.372494])
    print("approach postion")

    UR5.MoveJ(spin_end_pose, blocking=True)

    tls.student_tool_attach()
    run_visual_program(RDK, 'Hide_Rancilio_Rancilio_Tool_Rotated', blocking=True) #hide the tool on the machine
    rancilio_tool.setVisible(True,False) #show it on the toolhead (visual)

    basket_spin_bkwd()

    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90-45, theta_x= 90, theta_z=90, pos_z= -20), blocking=True)
    UR5.MoveJ([42.109253, -95.512633, 140.361378, -46.587357, -257.904029, -220.364433])

def T(): # Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s.

    rancilio_tool.setVisible(True,False) #show it on the toolhead (visual)
    UR5.MoveJ(tf.pose(points_df, 60, tool=id.Rancillio_Basket_Tool_Base, pos_z = 100, pos_y=75, theta_z=90, theta_x=90), blocking=True)
    UR5.MoveJ(tf.pose(points_df, 60, tool=id.Rancillio_Basket_Tool_Base, pos_z = 100, pos_y=0, theta_z=90, theta_x=90), blocking=True)

    # HOW DO I FIX THIS - WORKS IN SIM BUT NOT IRL
    UR5.MoveL([255.520000, -89.820000, 244.110000, -152.330000, -104.170000, -40.620000], blocking=True)

    UR5.MoveJ(tf.pose(points_df, 60, tool=id.Rancillio_Basket_Tool_Base, pos_z = -10, pos_y=0, theta_z=-90, theta_x=90), blocking=True)
    time.sleep(5) #actuate for 5s


    UR5.MoveJ(tf.pose(points_df, 60, tool=id.Rancillio_Basket_Tool_Base, pos_z = 20, pos_y=0, theta_z=-90, theta_x=90), blocking=True)

def U(): # Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and actuate for 5s.
    UR5.MoveJ(tf.pose(points_df, 61, tool=id.Rancillio_Basket_Tool_Base, pos_z = 20, pos_y=0, theta_z=-90, theta_x=90), blocking=True)
    UR5.MoveJ(tf.pose(points_df, 61, tool=id.Rancillio_Basket_Tool_Base, pos_z = -10, pos_y=0, theta_z=-90, theta_x=90), blocking=True)
    time.sleep(5) #actuate for 5s

def V(): # Return the Rancilio tool to the tool stand.
    UR5.MoveJ(tf.pose(points_df, 61, tool=id.Rancillio_Basket_Tool_Base, pos_z = 100, pos_y=100, theta_z=-90, theta_x=90), blocking=True)
    tls.rancilio_tool_detach_l_ati()

def R(): #Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.
    # PART R - STARTED
    tls.cup_tool_attach_l_ati()
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale_Centre, tool=id.Cup_Closed_Tool, pos_z=74, pos_x=-100, theta_y= 90, theta_x=90, theta_z=90), blocking=True)
    
    tls.cup_tool_open_ur5()
    

    # Go to the cup approach postison
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale_Centre, tool=id.Cup_Closed_Tool, pos_z=74, theta_y= 90, theta_x=90, theta_z=90), blocking=True)


    tls.cup_tool_shut_ur5()
    run_visual_program(RDK, 'Hide_Rancilio_Scale_Cup', blocking=True) #hide the cup on the scales

    run_visual_program(RDK, 'Show_Cup_Tool_Cup', blocking=True) #hide the cup on the scales
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale_Centre, tool=id.Cup_Closed_Tool, pos_z=76, pos_x=-100, theta_y= 90, theta_x=90, theta_z=90), blocking=True)


    UR5.MoveJ([-48.776840, -113.520569, 142.795718, -28.628641, -0.576878, -220.638884])

    # TODO: PUT IN ACTUAL CUSTOMER ZONE 

    UR5.MoveJ([-91.037149, -104.362910, 148.652730, -41.925520, -0.572249, -222.344276])

    tls.cup_tool_open_ur5()

    run_visual_program(RDK, 'Hide_Cup_Tool_Cup', blocking=True) #hide the cup on the scales

    UR5.MoveJ([-121.815757, -119.327302, 147.901424, -28.561688, -31.335721, -219.990653])
    tls.cup_tool_shut_ur5()
    UR5.MoveJ([-121.815757, -126.554662, 119.721341, 6.845755, -31.335721, -219.990653])

    tls.cup_tool_detach_r_ati()


M()
N()
# A()
# B()
# C()
# D()
# E()
# F()
# G()
# H()
# I()
# J()
K()
L()
# O()
# P()
# Q()
# S()
# T()
# U()
# V()
# R() 


