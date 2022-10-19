# -*- coding: utf-8 -*-
"""
Created on Thu May 12 15:12:08 2022

@author: Nanxin Chen
"""

import pybullet as p
import pybullet_data
import time
import os
GRAVITY = -9.8
dt = 1e-3
iters = 2000
#basic modes
#observer view,1:following 0:free
follow_view=1
#control mode,1:automatic 2:manual
MODE = 2
#simulation mode, 1:real time simulation, 0:step simulation
Sim_mode = 1

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) #ban cpu graphics
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0) #set to 1 later

p.resetSimulation()
p.setGravity(0,0,GRAVITY)
p.setTimeStep(dt)
p.setRealTimeSimulation(Sim_mode)
#.configureDebugVisualizer(p.COV_ENABLE_GUI(),1)
#import the ground
planeID = p.loadURDF("plane100.urdf",useMaximalCoordinates=False)

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])

#robot creation
#import our design model from fusion360

#v2 = "C:\\Users\\15056\\anaconda3\\Lib\\site-packages\\pybullet_data\\Loomo_K2\\model.sdf"
#loomo
Test="G:\Master\Semester2\Praktikum\Fusion360\Rollbody2.1_test4\model.sdf"
robot = p.loadSDF(Test,globalScaling=1.0)
p.resetBasePositionAndOrientation(robot[0],[0,0,0.25],[0,0,0,1])
V2 = "G:\Master\Semester2\Praktikum\Fusion360\Rollbody v2.0 v4.obj"
#get the link and joint
NumJoints = p.getNumJoints(robot[0])
Joint1 = p.getJointInfo(robot[0],0)
Joint2 = p.getJointInfo(robot[0],1)

#control the robot
v = 1
maxForce = 100
mode = p.VELOCITY_CONTROL

#shared parameters for visual and collision model
shift = [0,0,0]
scale = [0.01,0.01,0.01]
#create obstacles
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName=V2,
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.4, 0.4, 0],
    visualFramePosition=shift,
    meshScale=scale
)

collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName=V2,
    collisionFramePosition=shift,
    meshScale=scale
)

p.createMultiBody(
    baseMass=10,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, -5, 5],
    baseOrientation=[-5,0,50,1],
    useMaximalCoordinates=True
)


#set up walls
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[500, 5, 200]
)

collison_box_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[500, 5, 200]
)

wall_id = p.createMultiBody(
    baseMass=10000,
    baseCollisionShapeIndex=collison_box_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, 500, 200]
)

#set the center of mass frame (loadURDF sets base link frame) 
#startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
while(1):
    if Sim_mode == 0:
        p.stepSimulation()
    time.sleep(1./240.)
    
    
    #following view in 3rd person perspective
    if follow_view == True:
        location, _ = p.getBasePositionAndOrientation(robot[0])
        p.resetDebugVisualizerCamera(cameraDistance=2,
                                     cameraYaw=120,
                                     cameraPitch=-45,
                                     cameraTargetPosition=location)
    
    #debug text
    textColor = [1,0,0]
    debug_text_id = p.addUserDebugText(
    text="",
    textPosition=[0, 0, 1],
    textColorRGB=textColor,
    textSize=2
    )
    
    #automatic moving
    if MODE == 1:
        p.setJointMotorControlArray(robot[0],
                                [Joint1[0],Joint2[0]],
                                controlMode = mode,
                                targetVelocities = [-v,v],
                                forces = [maxForce,maxForce])
    
    #control the robot with keyboard
    if MODE == 2:    
        key_dict = p.getKeyboardEvents()
        if len(key_dict):
            #left and forward
            if p.B3G_UP_ARROW in key_dict and p.B3G_LEFT_ARROW in key_dict:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[1,2],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="left and forward",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
            #right and forward
            elif p.B3G_UP_ARROW in key_dict and p.B3G_RIGHT_ARROW in key_dict:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[2,1],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="right and forward",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
            #forward
            elif p.B3G_UP_ARROW in key_dict:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[2,2],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="forward",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
            #backward
            elif p.B3G_DOWN_ARROW in key_dict:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[-2,-2],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="backward",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
            #left
            elif p.B3G_LEFT_ARROW in key_dict:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[0,1],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="left",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
            #right
            elif p.B3G_RIGHT_ARROW in key_dict:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[1,0],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="right",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
            #stop
            else:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot[0],
                    jointIndices=[0,1],
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocities=[0,0],
                    forces=[maxForce,maxForce]
                    )
                debug_text_id = p.addUserDebugText(
                    text="",
                    textPosition=[0, 0, 1],
                    textColorRGB=textColor,
                    textSize=3,
                    lifeTime=1/24,
                    replaceItemUniqueId=debug_text_id
                    )
#cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)
#print(cubePos,cubeOrn)
p.disconnect()
