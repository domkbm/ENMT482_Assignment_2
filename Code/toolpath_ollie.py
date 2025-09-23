# """
# Author Dominic McNicholas
# UC Mecatronics
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

# from robodk.robodialogs import *
# from modbus_scale_client import modbus_scale_client


import tools
import indices as id
import transforms as tf

GROUNDS_WEIGHT_TARGET = 19.9


#DOM 
# a. Pick up the Rancilio tool and place it on the Mazzer Scale pan.
# b. Use the Mazzer tool to unlock the Mazzer Scale.
# c. Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.
# d. Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of coffee grounds has been deposited in the Rancilio tool.
# e. Use the Mazzer tool to lock the Mazzer Scale.
# f. Remove the Rancilio tool from the Mazzer.
# g. Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
# h. Release the Rancilio tool and close the WDT fixture.
# i. Use the Mazzer tool to turn the WDT rotor five full revolutions.
# j. Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.

#Ollie
# k. Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine tamps the coffee grounds.
# l. Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group head.
# m. Use the Mazzer tool to operate the cup dispenser.
# n. Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan.

#Lenny
# o. Use the Mazzer tool to unlock the Rancilio Scale.
# p. Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports 32±0.1g of water has been dispensed in the cup.
# q. Use the Mazzer tool to lock the Rancilio Scale.
# r. Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.
# s. Remove the Rancilio tool from the group head.
# t. Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s.
# u. Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and actuate for 5s.
# v. Return the Rancilio tool to the tool stand.


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
# mazzer_scale =  modbus_scale_client.ModbusScaleClient(host = id.IP_MAZZER_3)
# if mazzer_scale.server_exists() == False:
#     if RUNMODE_SIMULATE:
#         print("Mazzer scale not detected, output will be simulated.")
#     else:
#         RDK.ShowMessage("Mazzer scale not detected, output will be simulated.")
# rancilio_scale =  modbus_scale_client.ModbusScaleClient(host = id.IP_RANCILIO_3)
# if mazzer_scale.server_exists() == False:
#     if RUNMODE_SIMULATE:
#         print("Rancilio scale not detected, output will be simulated.")
#     else:
#         RDK.ShowMessage("Rancilio scale not detected, output will be simulated.")

mazzer_tool = RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL) 
rancilio_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL) 


#get all the frames
points_df = tf.create_points_df()

ranc_in_pose = tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90-15, theta_x= 90, theta_z=90, pos_z= 10)

ranc_mid_pose = tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90-30, theta_x= 90, theta_z=90)

ranc_out_pose = tf.pose(points_df, id.Rancillio_Gasket, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90-45, theta_x= 90, theta_z=90)





# reset the sim
robot_program = RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()



basket_drop_pose = tf.pose(points_df, id.Mazzer_Scale_Ball, tool=id.Rancillio_Indent, theta_y=90, theta_z=180, pos_z=-1)

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
    # mazzer_scale.tare()
    # weight = mazzer_scale.read() #this is a bit weird but maby add a difrencing thing. 
    # i = 0
    # while weight <= GROUNDS_WEIGHT_TARGET:
    #     i += 1
    #     # Forward movement
    #     UR5.MoveC(circular_path[1], circular_path[-1], blocking=False)
    #     ii = 0
    #     print(f"robot still moving {UR5.Busy()}")

        
    #     while UR5.Busy():
    #         ii += 1
    #         weight = mazzer_scale.read()
    #         print(f'weight {weight}, loop {i}.0{ii}')
    #         if weight >= GROUNDS_WEIGHT_TARGET:
    #             UR5.Stop()
    #             print('ohoh')
    #             break
    #     if weight >= GROUNDS_WEIGHT_TARGET:
    #         break


    #     # weight += 7 # TODO remove
    #     time.sleep(0.5)
    #     UR5.MoveC(circular_path[1], circle_start_pose, blocking=False)
    #     ii = 0
    #     while UR5.Busy():
    #         ii += 1
    #         weight =  mazzer_scale.read()
    #         print(f'weight {weight}, loop {i}.{ii}0')
    #         if weight >= GROUNDS_WEIGHT_TARGET:
    #             UR5.Stop()
    #             break
    #     if weight >= GROUNDS_WEIGHT_TARGET:
    #         break
    #move up to let go of the lever
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([0,0,60,0,0,0]) * UR5.Pose(), blocking=True)

def D_alt(): #blocking version
    UR5.MoveJ(tf.pose(points_df, id.Mazzer_Lever, tool=id.Mazzer_Bar_Tool, theta_x=-190, off_x= 9, off_y= 30, off_z = -12))
    circle_start_pose = tf.pose(points_df, id.Mazzer_Lever, tool=id.Mazzer_Bar_Tool, theta_x=-200, off_x= 8, off_y= 30, off_z = -7)
    UR5.MoveJ(circle_start_pose, blocking=True)

    circular_path = tf.generate_circular_path(circle_start_pose, tf.pose(points_df, id.Mazzer), -65, n_steps=20)
    reversed_circular_path = circular_path.copy()
    reversed_circular_path = list(reversed(reversed_circular_path))
    reversed_circular_path.append(circle_start_pose)
    reversed_circular_path = reversed_circular_path[1:]
    # mazzer_scale.tare()
    # weight = mazzer_scale.read()
    # weight_target = weight + GROUNDS_WEIGHT_TARGET # alt method
    # weight = 0
    # while weight < GROUNDS_WEIGHT_TARGET:

    #     #move fwd
    #     for pose in circular_path:
    #         UR5.MoveL(pose)
    #         weight = mazzer_scale.read()
    #         if weight >= GROUNDS_WEIGHT_TARGET:
    #             break

    #     # weight += 20
    #     if weight >= GROUNDS_WEIGHT_TARGET:
    #         break
            
    #     #move back
    #     for pose in reversed_circular_path:
    #         UR5.MoveL(pose)
    #         weight = mazzer_scale.read()
    #         if weight >= GROUNDS_WEIGHT_TARGET:
    #             break
    #     if weight >= GROUNDS_WEIGHT_TARGET:
    #         break
        
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([0,0,60,0,0,0]) * UR5.Pose(), blocking=True)
    UR5.MoveJ([117.300000, -102.340000, 94.140000, -80.130000, -95.230000, -231.640000])
        
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
    UR5.MoveL(robomath.TxyzRxyz_2_Pose([0,0,25,0,0,0]) * basket_drop_pose, blocking=True)

above_wdt_pose = tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, off_x=-50, pos_y = 1)
wdt_pose = tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, pos_y = 1)

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
    UR5.MoveJ([122.700000, -94.510000, 152.060000, -59.680000, 52.670000, 141.300000])
    UR5.MoveL(tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool_Base, theta_y=-90, off_z=50), blocking=True)
    UR5.MoveL(wdt_pose, blocking=True)
    tls.student_tool_attach()
    run_visual_program(RDK, 'Hide_WDT_Rancilio_Tool', blocking=True) #tool ouff the wdt (visual)
    rancilio_tool.setVisible(True,False) #put it back on the toolhead (visual)
    UR5.MoveL(above_wdt_pose)


def K():#TODO k. Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine tamps the coffee grounds.
    UR5.MoveJ([122.700000, -94.510000, 152.060000, -59.680000, 52.670000, 141.300000])
    UR5.MoveJ([88.250000, -94.510000, 152.060000, -59.680000, -32.420000, 141.300000]) #align angle with puq
    UR5.MoveJ([88.250000, -94.510000, 144.280000, -51.830000, -49.030000, 141.300000])
    UR5.MoveL(tf.pose(points_df, id.PUQ_Clamp, tool=id.Rancillio_Basket_Tool, theta_x = -90,theta_y=-90, theta_z=-90, off_x=3, off_z=-2), blocking=True)
    time.sleep(2) #wait for the puq to tamp

def L():#TODO l. Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group head.
    UR5.MoveJ([88.250000, -94.510000, 144.280000, -51.830000, -49.030000, 141.300000])
    UR5.MoveJ([-37.820000, -94.510000, 144.280000, -51.830000, -49.030000, 141.300000])
    UR5.MoveJ([-37.820000, -94.510000, 138.680000, -35.020000, -18.210000, 130.270000])
    UR5.MoveJ([-4.200000, -77.040000, 138.680000, 214.320000, -1.400000, -138.680000])
    # -4.200000, -95.120000, 138.680000, 214.320000, -1.400000, -121.870000
    # UR5.MoveJ([42.109253,-95.512633, 140.361378, -46.587357, -257.904029, -220.364433], blocking=True)

    
# m. Use the Mazzer tool to operate the cup dispenser.

# n. Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan.




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





# A()
# B()
# C()
# D_alt() # or D()
# E()
# F()
# G()
# H()
# I()
J()
K()
L()

# tls.rancilio_tool_attach_l_ati()
# UR5.MoveJ([32.440000, -84.510000, 131.920000, -49.900000, 52.850000, 141.770000])
# basket_spin_fwd()
# basket_spin_bkwd()