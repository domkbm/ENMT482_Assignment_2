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

from robodk.robodialogs import *
from modbus_scale_client import modbus_scale_client

import tools
import indices as id
import transforms as tf

CORRECT_WEIGHT = 32 # Grams



#DOM 
# a. Pick up the Rancilio tool and place it on the Mazzer Scale pan.
# b. Use the Mazzer tool to unlock the Mazzer Scale.
# c. Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.
# d. Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of
# coffee grounds has been deposited in the Rancilio tool.
# e. Use the Mazzer tool to lock the Mazzer Scale.
# f. Remove the Rancilio tool from the Mazzer.
# g. Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
# h. Release the Rancilio tool and close the WDT fixture.
# i. Use the Mazzer tool to turn the WDT rotor five full revolutions.
# j. Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.

#Ollie
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



#Setup
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
    RDK.ShowMessage("Mazzer scale not detected, output will be simulated.")
rancilio_scale =  modbus_scale_client.ModbusScaleClient(host = id.IP_RANCILIO_3)
if mazzer_scale.server_exists() == False:
    RDK.ShowMessage("Mazzer scale not detected, output will be simulated.")

#for visuals
mazzer_tool = RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL) 
rancilio_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL) 

# reset the sim
robot_program = RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

#get all the frames
points_df = tf.create_points_df()


#each step (so you can turn them off quick)

def o(): # Use the Mazzer tool to unlock the Rancilio Scale.
    # PART O - Ready to Test
    # intermeidate point to avoid the tool holder
    UR5.MoveJ([85.45, -105, 80.20, -81.3, -90, -173.26])
    # Postion before unlocking scales
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5, pos_y=21.53, pos_z=0, theta_x = 180), blocking=True)
    # Flipping Ranccilio Scale Switch
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5-20, pos_y=21.53, pos_z=-25, theta_x = 180), blocking=True)

def p(): # Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports 32±0.1g of water has been dispensed in the cup.

    # PART P - Ready to Test
    UR5.MoveJ([16.840846, -94.755605, 114.632103, -26.459245, 35.176778, -42.243328])

    # Press Switch in the Rancillio Pose
    UR5.MoveJ(tf.pose(points_df, id.Rancillio, tool=id.Mazzer_Tip_Tool, pos_x=48.6, pos_y=38.5, pos_z=-58.2, theta_x=0, theta_y=-90, theta_z=180), blocking=True)

    # Press Button
    UR5.MoveJ(tf.pose(points_df, id.Rancillio, tool=id.Mazzer_Tip_Tool, pos_x=48.6, pos_y=38.5, pos_z=-58.2, theta_y=-90, theta_z=180, off_z=10), blocking=True)

    weight = 0
    TIMEOUT = 0
    while (weight >= CORRECT_WEIGHT or TIMEOUT):
        # do weight thing
        time.sleep(5)
        break

    # Slide tool down to flick switch back off
    UR5.MoveJ(tf.pose(points_df, id.Rancillio, tool=id.Mazzer_Tip_Tool, pos_x=48.6, pos_y=38.5, pos_z=-58.2-10, theta_y=-90, theta_z=180, off_z=10), blocking=True)

    # Pull Tool Back
    UR5.MoveJ(tf.pose(points_df, id.Rancillio, tool=id.Mazzer_Tip_Tool, pos_x=48.6, pos_y=38.5, pos_z=-58.2, theta_y=-90, theta_z=180), blocking=True)
        
def q(): #Use the Mazzer tool to lock the Rancilio Scale.
    # PART Q - Ready to Test

    UR5.MoveJ([16.840846, -94.755605, 114.632103, -26.459245, 35.176778, -42.243328])

    # Postion before unlocking scales
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5, pos_y=21.53, pos_z=0, theta_x = 180), blocking=True)
    # Flipping Ranccilio Scale Switch
    UR5.MoveJ(tf.pose(points_df, id.Rancillio_Scale, tool=id.Mazzer_Tip_Tool, pos_x=31.5+20, pos_y=+25, pos_z=-25, theta_x = 180), blocking=True)


    UR5.MoveJ([47.523236, -108.009605, 95.271845, -77.263055, -90.000451, -89.608515])

    tls.mazzer_tool_detach_l_ati()
        
def r(): #Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.
    # PART R - STARTED
    tls.cup_tool_attach_l_ati()

    # Go to the cup approach postison

    # move to postion to pick up cup

    # Actuate Gripper

    # Move cup up

    # Pull Cup Cout

    # Move to Customer Postion

    # Actuate Gripper

    # Pull Away

    # Move to twisted Rancillio tool aproach Postion

    # Go to tool

    # Connect

    # Rotate tool

    # Down Move

    # Pull Away

    # Clean Tool

    # Put Away

def s(): # Remove the Rancilio tool from the group head.

    #TODO go to the tool 
    tls.student_tool_attach()
    run_visual_program(RDK, 'Hide_Rancilio_Rancilio_Tool_Rotated', blocking=True) #hide the tool on the machine
    rancilio_tool.setVisible(True,False) #show it on the toolhead (visual)
    
    pass

def t(): # Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s.
    pass

def u(): # Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and actuate for 5s.
    pass

def v(): # Return the Rancilio tool to the tool stand.
    pass






#calls
tls.mazzer_tool_attach_l_ati()
run_visual_program(RDK, 'Show_Rancilio_Scale_Cup', blocking=True) #show the cup on the scales 

o()
p()
q()
r()
s()
t()
u()
v()


# tls.mazzer_tool_detach_l_ati()
# go back home
# UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)