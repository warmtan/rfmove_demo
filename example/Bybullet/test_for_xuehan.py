import sys

import numpy
import time
import math
sys.path.append("/home/robotflow/dual_flexiv_lees/rfmove/install/lib")
# import moveit_noros as moveit
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from moveit_noros import rfWaypoint,PlannerSpline
import moveit_noros as moveit
import multiprocessing as mp
from scipy.spatial.transform import Rotation as R
__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"


sys.path.insert(0, "/home/robotflow/dual_flexiv_lees/flexiv_rdk-0.6.1/lib_py")
import flexivrdk

joint_name_list=[   'base_joint',
                    'joint1',
                    'joint2',
                    'joint3',
                    'joint4',
                    'joint5',
                    'joint6',
                    'joint7',
                    'link7_to_flange',
                    'flange_fixed',
                    'rightgripper_base_joint',
                    'rightgripper_finger1_joint',
                    'rightgripper_finger1_finger_joint',
                    'rightgripper_finger2_joint',
                    'rightgripper_finger2_finger_joint',
                    'rightgripper_finger1_inner_knuckle_joint',
                    'rightgripper_finger1_finger_tip_joint',
                    'rightgripper_finger2_inner_knuckle_joint',
                    'rightgripper_finger2_finger_tip_joint',
                    'base_joint1',
                    'joint11',
                    'joint12',
                    'joint13',
                    'joint14',
                    'joint15',
                    'joint16',
                    'joint17',
                    'link7_to_flange1',
                    'flange_fixed1',
                    'leftgripper_base_joint',
                    'leftgripper_finger1_joint',
                    'leftgripper_finger1_finger_joint',
                    'leftgripper_finger2_joint',
                    'leftgripper_finger2_finger_joint',
                    'leftgripper_finger1_inner_knuckle_joint',
                    'leftgripper_finger1_finger_tip_joint',
                    'leftgripper_finger2_inner_knuckle_joint',
                    'leftgripper_finger2_finger_tip_joint']
print("== Initalize Pybullet ==")
physicsClient=p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId=p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/robotflow/dual_flexiv_lees/rfmove/resources/flexiv/flexiv_config/config")
# 设定位置起点
startPos=[0,0,0]
#从欧拉角设定四元素
startOrientation=p.getQuaternionFromEuler([0,0,0])

boxId=p.loadURDF("gazebo_rizon4.urdf",startPos,startOrientation,useFixedBase=1)

def createBoxForpyBullet(halfExtents,basePositon,baseOrientation):
    box_collision_id =p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = halfExtents,
                                        collisionFramePosition = [0,0,0])
    box_visual_id = p.createVisualShape(shapeType = p.GEOM_BOX,
                                        halfExtents = halfExtents,
                                        visualFramePosition = [0,0,0],
                                        rgbaColor = [1, 0.2, 0, 0.9],
                                        specularColor = [0, 0, 0])
    box_body_id = p.createMultiBody(baseMass = 0, # 0 means this object is fixed
                                        baseCollisionShapeIndex = box_collision_id,
                                        baseVisualShapeIndex = box_visual_id,
                                        basePosition = basePositon,  #都是在几何中心
                                        baseOrientation =baseOrientation,
                                        baseInertialFramePosition = [0,0,0])
createBoxForpyBullet([0.90,1.80,0.025],[0.56,0.0,0.015],[0,0,0,1])
createBoxForpyBullet([0.05,0.05,1.40],[1.06-0.16,0.91,0.70],[0,0,0,1])
createBoxForpyBullet([0.05,0.05,1.40],[0.0-0.16,-0.81,0.70],[0,0,0,1])
createBoxForpyBullet([0.05,0.05,1.40],[0.0-0.16,0.91,0.70],[0,0,0,1])
createBoxForpyBullet([0.05,0.05,1.40],[1.06-0.16,-0.81,0.70],[0,0,0,1])
createBoxForpyBullet([0.90,1.80,0.025],[0.56,0.0,1.385],[0,0,0,1])

def run_l(cond):
    robot_states_l = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode 
    # print("----------------------------------------------------")
    # print(robot_states_l.get_tcp_force())
    with cond:
        robot_l = flexivrdk.Robot("192.168.2.100","192.168.2.109")
        robot_l.enable()
        seconds_waited = 0
        while not robot_l.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational")
        
        robot_l.setMode(mode.MODE_JOINT_POSITION_NRT)
        while(robot_l.getMode()!=mode.MODE_JOINT_POSITION_NRT):
            time.sleep(1)

        period = 1.0/1000
        loop_time = 0
        robot_l.getRobotStates(robot_states_l)
        init_pos_l=robot_states_l.q.copy()
        
        DOF = len(robot_states_l.q)
        target_pos = init_pos_l.copy()
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF

         # Joint motion constraints
        MAX_VEL = [2.0] * DOF
        MAX_ACC = [3.0] * DOF
        MAX_JERK = [20.0] * DOF

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 0.4

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 1
        cond.wait()
        print("consumer after wait1")
        while True:
            time.sleep(0.002)

            if robot_l.isFault():
                raise Exception("Fault occurred on robot serve ,exiting ....")
            
            target_pos[5] = init_pos_l[5] + SWING_AMP *math.sin(2 * math.pi * SWING_FREQ * loop_time)

            # Send command
            robot_l.sendJointPosition(
                target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC, MAX_JERK)
            
            # Increment loop time
            loop_time += period

        
        # print("consumer after wait")

def run_r(cond):
    robot_states_r = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode 
    with cond:
        robot_r = flexivrdk.Robot("192.168.2.101","192.168.2.109")
        robot_r.enable()
        seconds_waited = 0
        while not robot_r.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational")
        
        robot_r.setMode(mode.MODE_JOINT_POSITION_NRT)
        while(robot_r.getMode()!=mode.MODE_JOINT_POSITION_NRT):
            time.sleep(1)

        period = 1.0/1000
        loop_time = 0
        robot_r.getRobotStates(robot_states_r)
        init_pos_r=robot_states_r.q.copy()
        
        DOF = len(robot_states_r.q)
        target_pos = init_pos_r.copy()
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF

         # Joint motion constraints
        MAX_VEL = [2.0] * DOF
        MAX_ACC = [3.0] * DOF
        MAX_JERK = [20.0] * DOF

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 0.4

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 1
        cond.wait()
        print("consumer after wait2")

        while True:
            time.sleep(0.002)
            if robot_r.isFault():
                raise Exception("Fault occurred on robot serve ,exiting ....")
            
            target_pos[5] = init_pos_r[5] + SWING_AMP *math.sin(2 * math.pi * SWING_FREQ * loop_time)

            # Send command
            robot_r.sendJointPosition(
                target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC, MAX_JERK)
            
            # Increment loop time
            loop_time += period

        print("consumer after wait")

def getPlanInfo(trans_r=[0.3,0.2,0.38],rot_r=[0,-math.pi,0],trans_l=[0.6,0.2,0.38],rot_l=[0,-math.pi,0]):
   
    # right absolute  in world frame
    waypoint1=rfWaypoint([trans_r[0],trans_r[1],trans_r[2]],[rot_r[0],rot_r[1],rot_r[2]])
    # left absolute in world frame 
    waypoint2=rfWaypoint([trans_l[0],trans_l[1],trans_l[2]],[rot_l[0],rot_l[1],rot_l[2]])
    # waypoint3=rfWaypoint([0.20,-0.2,0.54],[planning_scene->getCurrentStateNonConst();0,math.pi,math.pi/2])

    robot_l = flexivrdk.Robot("192.168.2.100", "192.168.2.109")
    robot_states_l = flexivrdk.RobotStates()
    robot_l.getRobotStates(robot_states_l)
    init_pos_l = robot_states_l.q.copy()
    # print("Initial positions set to: ", init_pos_l)

    robot_r = flexivrdk.Robot("192.168.2.101", "192.168.2.109")
    robot_states_r = flexivrdk.RobotStates()
    robot_r.getRobotStates(robot_states_r)
    init_pos_r = robot_states_r.q.copy()
    # print("Initial positions set to: ", init_pos_r)

    print("== Initalize Planner moveit model ==")
    plannerspline=PlannerSpline("rizon_arms")
    plannerspline.init("/home/robotflow/dual_flexiv_lees/rfmove/resources/flexiv/flexiv_config/config/gazebo_rizon4.urdf",
                       "/home/robotflow/dual_flexiv_lees/rfmove/resources/flexiv/flexiv_config/config/rizon4.srdf",
                       "/home/robotflow/dual_flexiv_lees/rfmove/resources/flexiv/flexiv_config/config/kinematics.yaml",
                       "/home/robotflow/dual_flexiv_lees/rfmove/resources/flexiv/flexiv_config/config/ompl_planning.yaml",
                       "/home/robotflow/dual_flexiv_lees/rfmove/resources/flexiv/flexiv_config/config/joint_limits.yaml")

   

    # [0.90,1.80,0.025]
    # [0.04,0.04,1.40]
    # [1.0,0.56,0.0,0.02]
    # [1.0,0.0,-0.86,0.70]
    # [1.0,0.0,0.86,0.70]
    # [1.0,1.06,-0.86,0.70]
   
    
    box_1=moveit.Box(0.90,1.80,0.025)
    box1_pose=moveit.EigenAffine3d()
    box1_pose.translation=[0.56,0.0,0.015]
    box1_pose.quaternion=[0,0,0,1]
    plannerspline.AddCollectionObject("1",box_1,box1_pose)

    box_2=moveit.Box(0.05,0.05,1.40)
    box2_pose=moveit.EigenAffine3d()
    box2_pose.translation=[1.04-0.16,0.91,0.70]
    box2_pose.quaternion=[0,0,0,1]
    plannerspline.AddCollectionObject("2",box_2,box2_pose)

    box_3=moveit.Box(0.05,0.05,1.40)
    box3_pose=moveit.EigenAffine3d()
    box3_pose.translation=[0.0-0.16,-0.81,0.70]
    box3_pose.quaternion=[0,0,0,1]
    plannerspline.AddCollectionObject("3",box_3,box3_pose)

    box_4=moveit.Box(0.05,0.05,1.40)
    box4_pose=moveit.EigenAffine3d()
    box4_pose.translation=[0.0-0.16,0.91,0.70]
    box4_pose.quaternion=[0,0,0,1]
    plannerspline.AddCollectionObject("4",box_4,box4_pose)

    box_5=moveit.Box(0.05,0.05,1.40)
    box5_pose=moveit.EigenAffine3d()
    box5_pose.translation=[1.04-0.16,-0.81,0.70]
    box5_pose.quaternion=[0,0,0,1]
    plannerspline.AddCollectionObject("5",box_5,box5_pose)

    box_6=moveit.Box(0.90,1.80,0.025)
    box6_pose=moveit.EigenAffine3d()
    box6_pose.translation=[0.56,0.0,1.385]
    box6_pose.quaternion=[0,0,0,1]
    plannerspline.AddCollectionObject("6",box_6,box6_pose)

    home=[0,
        init_pos_r[0],
        init_pos_r[1],
        init_pos_r[2],
        init_pos_r[3],
        init_pos_r[4],
        init_pos_r[5],
        init_pos_r[6],
        0,0,0,0,0,0,0,0,0,0,0,0,
        init_pos_l[0],
        init_pos_l[1],
        init_pos_l[2],
        init_pos_l[3],
        init_pos_l[4],
        init_pos_l[5],
        init_pos_l[6],
        0,0,0,0,0,0,0,0,0,0,0]


    home_pose=[ init_pos_r[0],
                init_pos_r[1],
                init_pos_r[2],
                init_pos_r[3],
                init_pos_r[4],
                init_pos_r[5],
                init_pos_r[6],
                init_pos_l[0],
                init_pos_l[1],
                init_pos_l[2],
                init_pos_l[3],
                init_pos_l[4],
                init_pos_l[5],
                init_pos_l[6]]


    initalizePybulletRobotState=list(home)
    p.setJointMotorControlArray(bodyIndex=boxId,
                            jointIndices=range(len(joint_name_list)),
                            targetPositions=initalizePybulletRobotState,
                            controlMode=p.POSITION_CONTROL) 
   

 
    plannerspline.InitRobotState(np.array(home_pose),"rizon_arms")


    plannerspline.dualControlSplineParameterization([waypoint1,waypoint2],
                                                    "rizon_arm",
                                                    "rizon_arm1",
                                                    "rizon_arms",
                                                    "base_link",
                                                    "link7",
                                                    "base_link1",
                                                    "link71",1,1)
    timeslist=plannerspline.getTimeList()
    plannerspline.sample_by_interval(0.001)
    ompltimelist=plannerspline.get_sample_by_interval_times()

    waypoints=[]
    joint1pos=plannerspline.get_ompl_sample(joint_name_list[1]).position
    joint2pos=plannerspline.get_ompl_sample(joint_name_list[2]).position
    joint3pos=plannerspline.get_ompl_sample(joint_name_list[3]).position
    joint4pos=plannerspline.get_ompl_sample(joint_name_list[4]).position
    joint5pos=plannerspline.get_ompl_sample(joint_name_list[5]).position
    joint6pos=plannerspline.get_ompl_sample(joint_name_list[6]).position
    joint7pos=plannerspline.get_ompl_sample(joint_name_list[7]).position

    joint11pos=plannerspline.get_ompl_sample(joint_name_list[20]).position
    joint12pos=plannerspline.get_ompl_sample(joint_name_list[21]).position
    joint13pos=plannerspline.get_ompl_sample(joint_name_list[22]).position
    joint14pos=plannerspline.get_ompl_sample(joint_name_list[23]).position
    joint15pos=plannerspline.get_ompl_sample(joint_name_list[24]).position
    joint16pos=plannerspline.get_ompl_sample(joint_name_list[25]).position
    joint17pos=plannerspline.get_ompl_sample(joint_name_list[26]).position

   
    for i in range(len(ompltimelist)):
        waypoints.append([home[0]      ,joint1pos[i],
                        joint2pos[i] ,joint3pos[i],
                        joint4pos[i] ,joint5pos[i],
                        joint6pos[i] ,joint7pos[i],
                        home[8]      ,home[9],
                        home[10]      ,home[11],
                        home[12]      ,home[13],
                        home[14]      ,home[15],
                        home[16]      ,home[17],
                        home[18]      ,home[19],
                        joint11pos[i],joint12pos[i],
                        joint13pos[i],joint14pos[i],
                        joint15pos[i],joint16pos[i],
                        joint17pos[i],home[17],
                        home[18]      ,home[19],
                        home[20]      ,home[22],
                        home[22]      ,home[23],
                        home[24]      ,home[25],
                        home[26]      ,home[27]])



    for i in range(len(ompltimelist)+10000):
        if i<len(ompltimelist):  
            p.stepSimulation()
            p.setJointMotorControlArray(bodyIndex=boxId,
                                        jointIndices=range(len(joint_name_list)),
                                        targetPositions=waypoints[i],
                                        controlMode=p.POSITION_CONTROL)
            time.sleep(1./1000.)
    
    return waypoints
def rpy_mat(rpy):
    num = [[0, 0, 0, rpy[0]],[0, 0, 0,rpy[1]],[0, 0, 0, rpy[2]],[0, 0, 0, 1]]
    rotation = rpy[3:6]
    r = R.from_euler('xyz',[rotation])
    rotation = r.as_matrix().reshape(-1)
    rotation = np.array(rotation)
    num = [[rotation[0], rotation[1],rotation[2], rpy[0]],
    [rotation[3], rotation[4],rotation[5],rpy[1]],
    [rotation[6], rotation[7],rotation[8], rpy[2]],
    [0, 0, 0, 1]]
    num = np.array(num)
    return num

def run_l_plan(cond,l_pos_values):
    robot_states_l = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode 
    with cond:
        robot_l = flexivrdk.Robot("192.168.2.100","192.168.2.109")
        robot_l.enable()
        seconds_waited = 0
        while not robot_l.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational -l")
        # robot_l.getRobotStates(robot_states_l)
        # goal_in_robot = [[robot_states_l.extForceInTcpFrame[0]],[robot_states_l.extForceInTcpFrame[1]],[robot_states_l.extForceInTcpFrame[2]],[1]]
        # # print("------标定算出来的结果------------")
        # robot_in_world = [0,0,0,0,0,57/180]
        # robot_in_world = np.array(robot_in_world)
        # robot_in_world = rpy_mat(robot_in_world)
        # print((robot_in_world @ goal_in_robot)[1])
        robot_l.setMode(mode.MODE_JOINT_POSITION_NRT)
        while(robot_l.getMode()!=mode.MODE_JOINT_POSITION_NRT):
            time.sleep(1)

        period = 1.0/1000
        loop_time = 0
        robot_l.getRobotStates(robot_states_l)
        init_pos_l=robot_states_l.q.copy()
        
        DOF = len(robot_states_l.q)
        target_pos = init_pos_l.copy()
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF

         # Joint motion constraints
        MAX_VEL = [2.0] * DOF
        MAX_ACC = [3.0] * DOF
        MAX_JERK = [20.0] * DOF
        # MAX_VEL = [0.2] * DOF
        # MAX_ACC = [0.3] * DOF
        # MAX_JERK = [20.0] * DOF

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 1.2

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 4
        cond.wait()
        print("consumer after wait1")
        for i in range(len(l_pos_values)):
            time.sleep(0.001)
            
            if robot_l.isFault():
                raise Exception("Fault occurred on robot serve ,exiting ....")
            robot_l.getRobotStates(robot_states_l)
            goal_in_robot = [[robot_states_l.extForceInTcpFrame[0]],[robot_states_l.extForceInTcpFrame[1]],[robot_states_l.extForceInTcpFrame[2]],[1]]
            # print("------标定算出来的结果------------")
            robot_in_world = [0,0,0,0,0,57/180]
            robot_in_world = np.array(robot_in_world)
            robot_in_world = rpy_mat(robot_in_world)
            if (robot_in_world @ goal_in_robot)[1] > 2.0:
                print((robot_in_world @ goal_in_robot)[1])
                break
            target_pos = l_pos_values[i]

            # Send command
            robot_l.sendJointPosition(
                target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC, MAX_JERK)
            
            # Increment loop time
            loop_time += period

def run_r_plan(cond,r_pos_values):
    robot_states_r = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode 
    with cond:
        robot_r = flexivrdk.Robot("192.168.2.101","192.168.2.109")
        robot_r.enable()
        seconds_waited = 0
        while not robot_r.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational -r")

        # robot_r.getRobotStates(robot_states_r)
        # goal_in_robot = [[robot_states_r.extForceInTcpFrame[0]],[robot_states_r.extForceInTcpFrame[1]],[robot_states_r.extForceInTcpFrame[2]],[1]]
        # # print("------标定算出来的结果------------")
        # robot_in_world = [0,0,0,0,0,57/180]
        # robot_in_world = np.array(robot_in_world)
        # robot_in_world = rpy_mat(robot_in_world)
        # print((robot_in_world @ goal_in_robot)[1])
        
        robot_r.setMode(mode.MODE_JOINT_POSITION_NRT)
        while(robot_r.getMode()!=mode.MODE_JOINT_POSITION_NRT):
            time.sleep(1)

        period = 1.0/1000
        loop_time = 0
        robot_r.getRobotStates(robot_states_r)
        init_pos_r=robot_states_r.q.copy()
        
        DOF = len(robot_states_r.q)
        target_pos = init_pos_r.copy()
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF

         # Joint motion constraints
        MAX_VEL = [2.0] * DOF
        MAX_ACC = [3.0] * DOF
        MAX_JERK = [20.0] * DOF
        # MAX_VEL = [0.2] * DOF
        # MAX_ACC = [0.3] * DOF
        # MAX_JERK = [20.0] * DOF


        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 1.2

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 4
        cond.wait()
        print("consumer after wait1")
        for i in range(len(r_pos_values)):
            time.sleep(0.001)
            
            if robot_r.isFault():
                raise Exception("Fault occurred on robot serve ,exiting ....")
            robot_r.getRobotStates(robot_states_r)
            goal_in_robot = [[robot_states_r.extForceInTcpFrame[0]],[robot_states_r.extForceInTcpFrame[1]],[robot_states_r.extForceInTcpFrame[2]],[1]]
            # print("------标定算出来的结果------------")
            robot_in_world = [0,0,0,0,0,57/180]
            robot_in_world = np.array(robot_in_world)
            robot_in_world = rpy_mat(robot_in_world)
            if (robot_in_world @ goal_in_robot)[1] > 2.0:
                break
            target_pos = r_pos_values[i]
            # Send command
            robot_r.sendJointPosition(
                target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC, MAX_JERK)
            
            # Increment loop time
            loop_time += period

def producer(cond1 ,cond2):
    with cond1:
        cond1.notify_all()
    with cond2:
        cond2.notify_all()

def main():
    # all_list=getPlanInfo(trans_r=[0.3,0.2,0.38],rot_r=[0,-math.pi,0],trans_l=[0.7,0.18,0.38],rot_l=[0,-math.pi,0])
    # r_list_all=[]
    # l_list_all=[]
    # for i in range(len(all_list)):
    #     r_list_all.append([all_list[i][1], all_list[i][2] ,
    #                        all_list[i][3], all_list[i][4] ,
    #                        all_list[i][5], all_list[i][6] ,
    #                        all_list[i][7]])
    #     l_list_all.append([all_list[i][20],all_list[i][21],
    #                        all_list[i][22],all_list[i][23],
    #                        all_list[i][24],all_list[i][25],
    #                        all_list[i][26]])
    
    # condition1 = mp.Condition()
    # condition2 = mp.Condition()

    # p1 = mp.Process(name = 'p1', target =run_r_plan, args=(condition1,r_list_all))
    # p2 = mp.Process(name = 'p2', target =run_l_plan, args=(condition2,l_list_all))
    # p3 = mp.Process(name = 'p3', target =producer ,args=(condition1,condition2,))
    
    # p1.start()
    # p2.start()
    # time.sleep(2)
    # p3.start()
    # p1.join()
    # p2.join()
    # time.sleep(2)

    # test2   l -
    all_list2=getPlanInfo(trans_r=[0.7,-0.30,0.63],rot_r=[0,-math.pi,0],trans_l=[0.7,0.30,0.63],rot_l=[0,-math.pi,0])
    r_list_all2=[]
    l_list_all2=[]
    for i in range(len(all_list2)):
        r_list_all2.append([all_list2[i][1], all_list2[i][2] ,
                           all_list2[i][3], all_list2[i][4] ,
                           all_list2[i][5], all_list2[i][6] ,
                           all_list2[i][7]])
        l_list_all2.append([all_list2[i][20],all_list2[i][21],
                            all_list2[i][22],all_list2[i][23],
                            all_list2[i][24],all_list2[i][25],
                            all_list2[i][26]])
    # # print(r_list_all2)
    # with open("r_list_all2.txt","a",encoding="UTF-8") as f:
    #     f.write(str(r_list_all2))
    # # print(l_list_all2)
    # with open("l_list_all2.txt","a",encoding="UTF-8") as f:
    #     f.write(str(l_list_all2))
    condition3 = mp.Condition()
    condition4 = mp.Condition()

    p11 = mp.Process(name = 'p11', target =run_r_plan, args=(condition3,r_list_all2))
    p22 = mp.Process(name = 'p22', target =run_l_plan, args=(condition4,l_list_all2))
    p33 = mp.Process(name = 'p33', target =producer ,args=(condition3,condition4,))
    
    p11.start()
    p22.start()
    time.sleep(2)
    p33.start()
    p11.join()
    p22.join()
    time.sleep(2)

    # # test3
    # all_list3=getPlanInfo(trans_r=[0.7,-0.4,0.58],rot_r=[0,-math.pi,0],trans_l=[0.7,0.4,0.58],rot_l=[0,-math.pi,0])
    # r_list_all3=[]
    # l_list_all3=[]
    # for i in range(len(all_list3)):
    #     r_list_all3.append([all_list3[i][1], all_list3[i][2] ,
    #                         all_list3[i][3], all_list3[i][4] ,
    #                         all_list3[i][5], all_list3[i][6] ,
    #                         all_list3[i][7]])
    #     l_list_all3.append([all_list3[i][20],all_list3[i][21],
    #                         all_list3[i][22],all_list3[i][23],
    #                         all_list3[i][24],all_list3[i][25],
    #                         all_list3[i][26]])
    
    # condition5 = mp.Condition()
    # condition6 = mp.Condition()

    # p111 = mp.Process(name = 'p111', target =run_r_plan, args=(condition5,r_list_all3))
    # p222 = mp.Process(name = 'p222', target =run_l_plan, args=(condition6,l_list_all3))
    # p333 = mp.Process(name = 'p333', target =producer ,args=(condition5,condition6,))
    
    # p111.start()
    # p222.start()
    # time.sleep(2)
    # p333.start()
    # p111.join()
    # p222.join()
    # time.sleep(2)

    # condition7 = mp.Condition()
    # condition8 = mp.Condition()

    # p1111 = mp.Process(name = 'p1111', target =run_r, args=(condition7,))
    # p2222 = mp.Process(name = 'p2222', target =run_l, args=(condition8,))
    # p3333 = mp.Process(name = 'p3333', target =producer ,args=(condition7,condition8,))
    
    # p1111.start()
    # p2222.start()
    # time.sleep(2)
    # p3333.start()
    # p1111.join()
    # p2222.join()
    # time.sleep(2)


if __name__=="__main__":
    main()
    

