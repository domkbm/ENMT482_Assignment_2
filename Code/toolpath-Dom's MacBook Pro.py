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

import tools
import indices as id
import transforms as tf


# TODO: start with tasks (b), (c) and (m)

#DOM 
# a. Pick up the Rancilio tool and place it on the Mazzer Scale pan.
# b. Use the Mazzer tool to unlock the Mazzer Scale.
# c. Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.
# d. Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of
# coffee grounds has been deposited in the Rancilio tool.
# e. Use the Mazzer tool to lock the Mazzer Scale.
# f. Remove the Rancilio tool from the Mazzer.

#Ollie
# g. Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
# h. Release the Rancilio tool and close the WDT fixture.
# i. Use the Mazzer tool to turn the WDT rotor five full revolutions.
# j. Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.
# k. Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine
# tamps the coffee grounds.
# l. Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group
# head.
# m. Use the Mazzer tool to operate the cup dispenser.
# n. Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan.

#Lenny
# o. Use the Mazzer tool to unlock the Rancilio Scale.
# p. Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports
# 32±0.1g of water has been dispensed in the cup.
# q. Use the Mazzer tool to lock the Rancilio Scale.
# r. Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.
# s. Remove the Rancilio tool from the group head.
# t. Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and
# actuate for 5s.
# u. Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and
# actuate for 5s.
# v. Return the Rancilio tool to the tool stand.



#helper fcns



RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
tls = tools.Tools(RDK)

#get all the frames
points_df = tf.create_points_df()




# reset the sim
robot_program = RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()


#TODO a) Pick up the Rancilio tool and place it on the Mazzer Scale pan.
# tls.rancilio_tool_attach_l_ati()
UR5.MoveJ([107.580000, -93.770000, 117.650000, -113.010000, -89.130000, -204.190000]) 
UR5.MoveJ([112.080000, -102.170000, 119.960000, -107.180000, -65.270000, -164.280000]) # re check these need some work 
UR5.MoveJ([146.390000, -80.000000, 137.570000, -62.720000, 116.640000, 140.000000])#intermeidate point to avoid the mazzer
UR5.MoveJ(tf.pose(points_df, id.Mazzer_Scale_Ball, tool=id.Rancillio_Indent, theta_y=90, theta_z=180))
basket_drop_pose = tf.pose(points_df, id.Mazzer_Scale_Ball, tool=id.Rancillio_Indent, theta_y=90, theta_z=180, pos_z=-2)
UR5.MoveL(basket_drop_pose, blocking=True)
tls.student_tool_detach()
UR5.MoveJ([146.390000, -80.000000, 137.570000, -62.720000, 116.640000, 140.000000])#intermeidate point to avoid the mazzer

 
# # #TODO b) Use the Mazzer tool to unlock the Mazzer Scale.
tls.mazzer_tool_attach_l_ati()

UR5.MoveJ([117.580000, -93.770000, 117.650000, -113.010000, -89.130000, -204.190000]) #intermeidate point to avoid the mazzer
UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=50,theta_x=-120), blocking=True)
print("move to above the lock")
UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=4,theta_x=-120), blocking=True)
print("down to unlock the lock")
UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -22, pos_z=-8,theta_x=-120), blocking=True)
print("down and across to unlock the lock")
UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -22, pos_z=50,theta_x=-120), blocking=True)
print("leave lock")


# #TODO c) Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.
# UR5.MoveJ([117.580000, -93.770000, 117.650000, -113.010000, -89.130000, -204.190000]) #intermeidate point (not needed atm) to avoid the mazzer
UR5.MoveJ(tf.pose(points_df, id.Mazzer_On_Button, tool=id.Mazzer_Tip_Tool, pos_y=10,theta_y=-110, off_z= 0), blocking=True) #go to on button
UR5.MoveJ(tf.pose(points_df, id.Mazzer_On_Button, tool=id.Mazzer_Tip_Tool, pos_y=10,theta_y=-110, off_z= -10), blocking=True) #push on button
the_time = time.time()
UR5.MoveJ(tf.pose(points_df, id.Mazzer_On_Button, tool=id.Mazzer_Tip_Tool, pos_y=10,theta_y=-110, off_z= 0), blocking=True)
# UR5.MoveJ(tf.pose(points_df, id.Mazzer_Off_Button, tool=id.Mazzer_Tip_Tool, pos_y=15,theta_y=-115, off_x = -5, off_z=4, off_y=10), blocking=True) #go to off button
# while (time.time() - the_time) < 1: # wait 15 secs
#     # print(f"Waited {time.time() - the_time:.1f} s")
#     pass # do nothing
# UR5.MoveJ(tf.pose(points_df, id.Mazzer_Off_Button, tool=id.Mazzer_Tip_Tool, pos_y=15,theta_y=-115, off_x = -5, off_z=6, off_y=10), blocking=True) #push off button
# UR5.MoveJ(tf.pose(points_df, id.Mazzer_Off_Button, tool=id.Mazzer_Tip_Tool, pos_y=15,theta_y=-115, off_x = -5, off_z=-2, off_y=10), blocking=True) #go to off button



# # #TODO d) Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of 
# #          coffee grounds has been deposited in the Rancilio tool.
# circle_start_pose = tf.pose(points_df, id.Mazzer_Lever, tool=id.Mazzer_Bar_Tool,theta_x=-200,theta_z=-50, off_x=-10, off_z=0,off_y=-40,off_theta_z=40)
# UR5.MoveL(circle_start_pose, blocking=True)
# #idk why MoveC doesnt work but sure. 
# circular_path = tf.generate_circular_path(circle_start_pose, tf.pose(points_df, id.Mazzer), -60, n_steps=10)
# for pose in circular_path:
#     # print(circle_start_pose)
#     # print(pose)
#     # UR5.MoveC(circle_start_pose, pose, blocking=True)
#     UR5.MoveL(pose, blocking=True)


# # TODO e) Use the Mazzer tool to lock the Mazzer Scale.
# UR5.MoveJ([117.580000, -93.770000, 117.650000, -113.010000, -89.130000, -204.190000]) #intermeidate point to avoid the mazzer
# UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=50,theta_x=-120), blocking=True)
# print("move to above the lock")
# UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= -10, pos_z=4,theta_x=-120), blocking=True)
# print("down to unlock the lock")
# UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Lock, tool=id.Mazzer_Tip_Tool, pos_x= 5, pos_z=-8,theta_x=-120), blocking=True)
# print("down and across to unlock the lock")

# # TODO f) Remove the Rancilio tool from the Mazzer.
# tls.mazzer_tool_detach_l_ati()
# UR5.MoveJ([146.390000, -80.000000, 137.570000, -62.720000, 116.640000, 140.000000])#intermeidate point to avoid the mazzer
# UR5.MoveL(basket_drop_pose, blocking=True)
# tls.student_tool_attach()


# #TODO g) Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
# tls.wdt_open()
# UR5.MoveL(tf.pose(points_df, id.Mazzer_Scale_Ball, tool=id.Rancillio_Indent, theta_y=90, theta_z=180, pos_y = 25, pos_z=75))
# above_wdt_pose = tf.pose(points_df, id.WDT, tool=id.Rancillio_Basket_Tool, theta_y=-90, off_x=75, off_y = -25)
# UR5.MoveL(above_wdt_pose, blocking=True)





#TODO h) Release the Rancilio tool and close the WDT fixture.
# tls.student_tool_detach()

#TODO i) Use the Mazzer tool to turn the WDT rotor five full revolutions.
# tls.mazzer_tool_attach_l_ati()
# UR5.MoveJ([146.390000, -80.000000, 137.570000, -62.720000, 116.640000, 140.000000])#intermeidate point to avoid the mazzer

#TODO j) Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.




# # #TODO m) Use the Mazzer tool to operate the cup dispenser.
# UR5.MoveJ([112.020000, -64.930000, 130.100000, -223.900000, -70.020000, 230.210000]) #another intermeidiate point so we dont hit the tool holder
# UR5.MoveJ([71.680000, -64.390000, 129.270000, -262.950000, -88.330000, 214.230000]) #another intermeidiate point so we dont hit the tool holder
# UR5.MoveJ([9.594445, -72.709382, 129.539942, -234.755204, -96.626572, 259.018886])
# UR5.MoveJ(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool, pos_x=50), blocking=True)
# print("move to above the latch")
# time.sleep(1)
# UR5.MoveL(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool), blocking=True)
# print("into latch")
# time.sleep(1)
# UR5.MoveL(tf.pose(points_df, id.Cup_Open, tool=id.Mazzer_Tip_Tool), blocking=True)
# print("open") 
# time.sleep(1)
# UR5.MoveL(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool), blocking=True)
# print("close")
# time.sleep(1)
# UR5.MoveL(tf.pose(points_df, id.Cup_Closed, tool=id.Mazzer_Tip_Tool, pos_x=50), blocking=True)
# print("move to above the latch")
# time.sleep(1)
# UR5.MoveJ([47.010000, -72.190000, 130.980000, -238.790000, -96.780000, 269.990000]) #another intermeidiate point so we dont hit the cup stack


# # put mazzer back away
# tls.mazzer_tool_detach_l_ati()
# # go back home
# UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)

