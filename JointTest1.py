# -*- coding: utf-8 -*-
"""
Created on Fri May 27 22:22:56 2022

@author: 15056
"""

import pybullet as p
import pybullet_data
import time
import os
import numpy as np
import math
GRAVITY = -9.8
#observer view,1:following 0:free
follow_view = 1
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setGravity(0,0,GRAVITY)
planeID = p.loadURDF("plane.urdf",useMaximalCoordinates=True)
#Test="G:\Master\Semester2\Praktikum\Fusion360\Rollbody1_Test1\model.sdf"
#fffrobot = p.loadSDF(Test,globalScaling=1.0)
Test2 = "G:\Master\Semester2\Praktikum\Fusion360\Rollbody2.1_test2\model.sdf"
robot = p.loadSDF(Test2,globalScaling=1.0)
p.resetBasePositionAndOrientation(robot[0],[0,0,0.25],[0,0,0,1])
robot = robot[0]
#robot = p.loadURDF("r2d2.urdf")

#get the infomation about link and joint
LinkState = p.getLinkState(robot,0)
Pos_COM = LinkState[0]
Ori_COM = LinkState[1]
EulerAngleCOM = p.getEulerFromQuaternion(Ori_COM)
NumJoints = p.getNumJoints(robot)
Joint1 = p.getJointInfo(robot,0)
Joint2 = p.getJointInfo(robot,1)

#frame line
frame_o,frame_posture = LinkState[4:6]
Rot_Mat = np.array(p.getMatrixFromQuaternion(frame_posture)).reshape(3,3)
X_axis = Rot_Mat[:,0]
Y_axis = Rot_Mat[:,1]
Z_axis = Rot_Mat[:,2]
X_endPoint = (np.array(frame_o)+np.array(X_axis*5)).tolist()
Y_endPoint = (np.array(frame_o)+np.array(Y_axis*5)).tolist()
Z_endPoint = (np.array(frame_o)+np.array(Z_axis*5)).tolist()
X_line_id = p.addUserDebugLine(frame_o,X_endPoint,[1,0,0])
Y_line_id = p.addUserDebugLine(frame_o,Y_endPoint,[0,1,0])
Z_line_id = p.addUserDebugLine(frame_o,Z_endPoint,[0,0,1])


#get dynamics information
Dynamics = p.getDynamicsInfo(robot,-1)
Mass = Dynamics[0]

Wheels = [i for i in range(NumJoints) if p.getJointInfo(robot,i)[2]==0]
Wheels_Num = 2
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setRealTimeSimulation(0)

#control the robot
v = 20
maxForce = 100
mode = p.VELOCITY_CONTROL

for i in range(10000):
    p.stepSimulation()
    
    RT_Ori_COM = p.getLinkState(robot,0)[1]
    EulerAngleCOM = p.getEulerFromQuaternion(RT_Ori_COM)
    theta = math.degrees(EulerAngleCOM[0])
    if theta>=85:
        theta=85
    elif theta<=-85:
        theta=-85
    
    #debug text
    textColor = [1,0,0]
    debug_text_id = p.addUserDebugText(
    text="",
    textPosition=[0, 0, 1],
    textColorRGB=textColor,
    textSize=2,
    )
    
    COM_text_id = p.addUserDebugText(
        #text=str(np.round([EulerAngleCOM[0],EulerAngleCOM[1],EulerAngleCOM[2]],4)),
        text=str(theta),
        textPosition=[0, 0, 1],
        textColorRGB=textColor,
        textSize=2,
        lifeTime=1/24,
        replaceItemUniqueId=debug_text_id
        )
    
    p.applyExternalForce(objectUniqueId=robot,
                         linkIndex=-1,
                         forceObj=[0,0,1100],
                         posObj=p.getBasePositionAndOrientation(robot)[0],
                         flags=p.WORLD_FRAME
                         )
    
    Temp_Force = Mass*GRAVITY*math.tan(theta)
    p.applyExternalForce(objectUniqueId=robot,
                         linkIndex=-1,
                         forceObj=[0,Temp_Force,0],
                         posObj=p.getBasePositionAndOrientation(robot)[0],
                         flags=p.WORLD_FRAME
                         )
    
    #following view in 3rd person perspective
    if follow_view == True:
        location, _ = p.getBasePositionAndOrientation(robot)
        p.resetDebugVisualizerCamera(cameraDistance=2,
                                     cameraYaw=120,
                                     cameraPitch=-45,
                                     cameraTargetPosition=location)
        
    p.setJointMotorControlArray(
        bodyUniqueId=robot,
        jointIndices=Wheels[0:Wheels_Num],
        controlMode=p.VELOCITY_CONTROL,
        targetVelocities=[-v,v],
        forces=[maxForce for _ in range(Wheels_Num)]
    )
    time.sleep(1 / 240)  
    
p.disconnect(physicsClient)