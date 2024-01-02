import os, inspect
import pdb
import pybullet as p
import pybullet_data
import datetime
import time
import math

END_EFFECTOR_INDEX = 7
# RESET_JOINT_INDICES = [1, 2, 3, 4, 5, 6, 8, 10, 12, 13, 15, 17]
RESET_JOINT_INDICES = [1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14]
#   RESET_JOINT_VALUES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# RESET_JOINT_VALUES = [0.0, -1, 1.57, 0.0, 1.57, 0.0] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# RESET_JOINT_VALUES = [0.0, -1.3, 2, 0.0, 0.0, 0.0] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RESET_JOINT_VALUES = [0.0, -1.1898947954177856, 1.9831578731536865, 0.0, 0.0, 0.0]+[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RESET_JOINT_VALUES_GRIPPER_CLOSED = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [-0.45, 0.45, -0.45, 0.45, -0.45, 0.45]

JOINT_LIMIT_LOWER = [-3.14, -3.14, -3.14, -3.14, -3.14, -3.14] + [-0.45, -1.57, -1.57, -1.57, -1.57, -1.57]
JOINT_LIMIT_UPPER = [3.14, 3.14, 3.14, 3.14, 3.14, 3.14] + [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
JOINT_RANGE = []
for upper, lower in zip(JOINT_LIMIT_LOWER, JOINT_LIMIT_UPPER):
    JOINT_RANGE.append(upper - lower)

# GRIPPER_LIMITS_LOW = JOINT_LIMIT_LOWER[-1]
# GRIPPER_LIMITS_HIGH = JOINT_LIMIT_UPPER[-1]

# print("\n\n\n")
# print("GRIPPER_LIMITS_LOW", GRIPPER_LIMITS_LOW)
# print("GRIPPER_LIMITS_HIGH", GRIPPER_LIMITS_HIGH)
# print("\n\n\n")

num_sim_steps = 10

def printList(name, lst):
    print_lst = [f"{item:0.2f}" for item in lst]
    print(name, print_lst)

def deg_to_rad(deg):
    return [d * math.pi / 180. for d in deg]

def rad_to_deg(rad):
    return [r * 180. / math.pi for r in rad]

def quat_to_deg(quat):
    euler_rad = p.getEulerFromQuaternion(quat)
    euler_deg = rad_to_deg(euler_rad)
    return euler_deg

def deg_to_quat(deg):
    rad = deg_to_rad(deg)
    quat = p.getQuaternionFromEuler(rad)
    return quat

def apply_action_ik_ur5(target_ee_pos, target_ee_quat, target_gripper_state,
                    robot_id, end_effector_index, movable_joints,
                    lower_limit, upper_limit, rest_pose, joint_range,
                    num_sim_steps=5, param_joint_list=None, isJoint=False):

    joint_poses = p.calculateInverseKinematics(robot_id,
                                               end_effector_index,
                                               target_ee_pos,
                                               target_ee_quat,
                                               lowerLimits=lower_limit,
                                               upperLimits=upper_limit,
                                               jointRanges=joint_range,
                                               restPoses=rest_pose,
                                               jointDamping=[0.001] * len(
                                                   movable_joints),
                                               solver=0,
                                               maxNumIterations=100,
                                               residualThreshold=.01)

    
    movable_joints = movable_joints[0:6]
    movable_joints.extend([9,12])

    target_joint_poses = param_joint_list if isJoint else list(joint_poses[0:6])

    target_gripper_poses = [target_gripper_state, -target_gripper_state]
    target_joint_poses.extend(target_gripper_poses)

    # max_forces=[150,150,150,28,28,28]+ [500,500,500,500,500,500]
    max_forces=[500]*8

    p.setJointMotorControlArray(robot_id,
                                movable_joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=target_joint_poses,
                                # targetVelocity=0,
                                forces=max_forces,
                                positionGains=[0.03] * len(movable_joints),
                                # velocityGain=1
                                )

    # p.setJointMotorControl2(robotID, 10, p.VELOCITY_CONTROL, force=500)
    # p.setJointMotorControl2(robotID, 11, p.VELOCITY_CONTROL, force=500)
    # p.setJointMotorControl2(robotID, 13, p.VELOCITY_CONTROL, force=500)
    # p.setJointMotorControl2(robotID, 14, p.VELOCITY_CONTROL, force=500)

    for _ in range(num_sim_steps):
        p.stepSimulation()


SIMULATION_TIME_STEP = 0.02
# SIMULATION_TIME_STEP = 1.0/240

#Getting Absolute Path from Relative Path of URDF file
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
robotUrdfPath = os.path.join(currentdir, "./ur5_rg2/urdf/ur5_rg2.urdf")

# connect to engine servers
physicsClient = p.connect(p.GUI) # GUI/DIRECT
# add search path for loadURDF
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0,0,-10)
planeID = p.loadURDF("plane.urdf")

# Simulation time-step
p.setTimeStep(SIMULATION_TIME_STEP)

# define robot
robotStartPos = [0,0,0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(robotUrdfPath))
print("----------------------------------------")
robotID = p.loadURDF(robotUrdfPath)
# robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn)

print("------------loaded-------------")

# get joint information
numJoints = p.getNumJoints(robotID) 
jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
print("------------------------------------------")
print("Number of joints: {}".format(numJoints))
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotID, i)
    jointID = jointInfo[0]
    jointName = jointInfo[1].decode("utf-8")
    jointType = jointTypeList[jointInfo[2]]
    linkName = jointInfo[12].decode("utf-8")
    jointLowerLimit = jointInfo[8]
    jointUpperLimit = jointInfo[9]
    parentIndex = jointInfo[16]

    print("ID: {}".format(jointID))
    print("name: {}".format(jointName))
    print("type: {}".format(jointType))
    print("Link: {}".format(linkName))
    print("ParentID: {}".format(parentIndex))
    print("lower limit: {}".format(jointLowerLimit))
    print("upper limit: {}".format(jointUpperLimit))
print("------------------------------------------")

# get links
linkInfo = p.getVisualShapeData(robotID)
linkIDs = list(map(lambda linkInfo: linkInfo[1], p.getVisualShapeData(robotID)))
print(linkIDs)
linkNum = len(linkIDs)
print(linkNum)


# tool1_pos, tool1_rot, _, _, _, _ = p.getLinkState(bodyUniqueId=robotID, linkIndex=15)
# cid = p.createConstraint(
#                         robotID, 11,robotID, 10,
#                         jointType=p.JOINT_POINT2POINT,
#                         jointAxis=[0, 0, 0],
#                         parentFramePosition=[0, 0, 0],
#                         childFramePosition=[0, 0, 0]
#                         )
# p.changeConstraint(cid, maxForce=500)

cid = p.createConstraint(
                        robotID, 14,robotID, 13,
                        jointType=p.JOINT_POINT2POINT,
                        jointAxis=[0, 0, 1],
                        parentFramePosition=[0.026 -0.015, 0],
                        childFramePosition=[-0.08, 0, 0]
                        )
p.changeConstraint(cid, maxForce=500)


# p.setJointMotorControl2(robotID, 10, p.VELOCITY_CONTROL, force=500)
# p.setJointMotorControl2(robotID, 11, p.VELOCITY_CONTROL, force=500)
# p.setJointMotorControl2(robotID, 13, p.VELOCITY_CONTROL, force=500)
# p.setJointMotorControl2(robotID, 14, p.VELOCITY_CONTROL, force=500)   


textPose = list(p.getBasePositionAndOrientation(robotID)[0])
textPose[2] += 1
x_c = 0.003
z_c = 0.047

lineID1 = p.addUserDebugLine([0,0,0], [0,0,0], [255,0,0])
lineID2 = p.addUserDebugLine([0,0,0], [0,0,0], [255,0,0])
lineID3 = p.addUserDebugLine([0,0,0], [0,0,0], [255,0,0])
lineID4 = p.addUserDebugLine([0,0,0], [0,0,0], [255,0,0])

p.addUserDebugText("Press \'w\' and magic!!", textPose, [255,0,0], 1)

prevLinkID = 0
linkIDIn = p.addUserDebugParameter("linkID", 0, linkNum-1e-3, 0)

ur5_joints = [1, 2, 3, 4, 5, 6]
gripper_joints = [9, 10, 11, 12, 13, 14]
movable_joints = ur5_joints + gripper_joints

ee_pos = [0.37, 0.11, 0.14]
ee_rot = [0, 0, 0]
param_names = ["X", "Y", "Z", "R", "P", "Y"]
param_ids = []
param_ids.append(p.addUserDebugParameter(param_names[0], -0.5, 0.5, startValue=ee_pos[0]))
param_ids.append(p.addUserDebugParameter(param_names[1], -0.5, 0.5, startValue=ee_pos[1]))
param_ids.append(p.addUserDebugParameter(param_names[2], -0.5, 0.5, startValue=ee_pos[2]))
param_ids.append(p.addUserDebugParameter(param_names[3], -179, 179, startValue=ee_rot[0]))
param_ids.append(p.addUserDebugParameter(param_names[4], -179, 179, startValue=ee_rot[1]))
param_ids.append(p.addUserDebugParameter(param_names[5], -179, 179, startValue=ee_rot[2]))



init_joint_val = [0.0, -1.32, 1.57, 1.22, 1.57, 1.57]
param_names = ["1", "2", "3", "4", "5", "6"]
for i in range(len(init_joint_val)):
    param_ids.append(p.addUserDebugParameter(param_names[i], -3.14, 3.14, startValue=init_joint_val[i]))

param_gripper_id = p.addUserDebugParameter(f"gripper:", -1.57, 1.57)
# param_gripper_id = p.addUserDebugParameter(f"gripper:", -0.45, 0.45)
flag = True
i_step = 0


######## Reset #########
for i, value in zip(RESET_JOINT_INDICES, RESET_JOINT_VALUES):
        p.resetJointState(robotID, i, value)



while(flag):
    # t = datetime.datetime.now()
    
    ee_pos = [p.readUserDebugParameter(param_ids[0]), p.readUserDebugParameter(param_ids[1]), p.readUserDebugParameter(param_ids[2])]
    ee_rot = [p.readUserDebugParameter(param_ids[3]), p.readUserDebugParameter(param_ids[4]), p.readUserDebugParameter(param_ids[5])]

    # P: (0.36823178249361377, 0.1090871886985425, 0.13537283344309153)
    # O: [179.73381141093887, -43.42871083888508, -179.62179031929355]
    # T: [0.0, -1.1898947954177856, 1.9831578731536865, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, -0.0, 0.0]

    # ee_pos = [0.36823178249361377, 0.1090871886985425, 0.13537283344309153]
    # ee_rot = [179.73381141093887, -43.42871083888508, -179.62179031929355]

    target_ee_pos = ee_pos
    target_ee_quat = deg_to_quat(ee_rot)  
    target_gripper_state = p.readUserDebugParameter(param_gripper_id)

    param_joint_list = []
    for i in range(len(init_joint_val)):
        param_joint_list.append(p.readUserDebugParameter(param_ids[i+6]))

    ee_position, ee_orientation, _, _, _, _ = p.getLinkState(bodyUniqueId=robotID, linkIndex=6)
    tool1_position, tool1_orientation, _, _, _, _ = p.getLinkState(bodyUniqueId=robotID, linkIndex=7)

    # ee_position, ee_orientation, _, _, _, _ = p.getLinkState(bodyUniqueId=robotID, linkIndex=END_EFFECTOR_INDEX)

    point1S = ee_position
    point1E = tool1_position
    # 

    line_len = 0.008
    line_ang = 180.0

    line_x = line_len*math.cos(line_ang*math.pi / 180.)
    line_y = line_len*math.sin(line_ang*math.pi / 180.)
    # print(line_x,line_y)
    point1S = [0,0,0]
    point1E = [point1S[0]+line_x, point1S[1]+line_y, point1S[2]]

    point2S = point1E
    point2E = [point1E[0]+0.1, point1E[1], point1E[2]]

    point3S = point1E
    point3E = [point1E[0], point1E[1]+0.1, point1E[2]]

    point4S = point1E
    point4E = [point1E[0], point1E[1], point1E[2]-0.1]

    # lineID1 = p.addUserDebugLine(point1S, point1E, [255,255,0], lineWidth =2, replaceItemUniqueId=lineID1)
    # lineID2 = p.addUserDebugLine(point2S, point2E, [255,0,0], lineWidth =2, replaceItemUniqueId=lineID2)
    # lineID3 = p.addUserDebugLine(point3S, point3E, [0,255,0], lineWidth =2, replaceItemUniqueId=lineID3)
    # lineID4 = p.addUserDebugLine(point4S, point4E, [0,0,255], lineWidth =2, replaceItemUniqueId=lineID4)

    parentIDX = 13
    lineWidth = 2

    lineID1 = p.addUserDebugLine(point1S, point1E, [255,255,0], lineWidth =lineWidth, parentObjectUniqueId=robotID, parentLinkIndex=parentIDX, replaceItemUniqueId=lineID1)
    lineID2 = p.addUserDebugLine(point2S, point2E, [255,0,0], lineWidth =lineWidth, parentObjectUniqueId=robotID, parentLinkIndex=parentIDX, replaceItemUniqueId=lineID2)
    lineID3 = p.addUserDebugLine(point3S, point3E, [0,255,0], lineWidth =lineWidth, parentObjectUniqueId=robotID, parentLinkIndex=parentIDX, replaceItemUniqueId=lineID3)
    lineID4 = p.addUserDebugLine(point4S, point4E, [0,0,255], lineWidth =lineWidth, parentObjectUniqueId=robotID, parentLinkIndex=parentIDX, replaceItemUniqueId=lineID4)
    # 
    apply_action_ik_ur5(
            target_ee_pos, target_ee_quat, target_gripper_state, robotID, END_EFFECTOR_INDEX, movable_joints,
            lower_limit=JOINT_LIMIT_LOWER,
            upper_limit=JOINT_LIMIT_UPPER,
            rest_pose=RESET_JOINT_VALUES,
            joint_range=JOINT_RANGE,
            num_sim_steps=num_sim_steps,
            param_joint_list=param_joint_list,
            isJoint = True)

    # p.setJointMotorControlArray(
    #     bodyUniqueId=robotID,
    #     jointIndices=movable_joints,
    #     controlMode=p.POSITION_CONTROL,
    #     targetPositions = target_positions)

    linkID = p.readUserDebugParameter(linkIDIn)
    if linkID!=prevLinkID:
        p.setDebugObjectColor(robotID, linkIDs[int(prevLinkID)], [255,255,255])
        p.setDebugObjectColor(robotID, linkIDs[int(linkID)], [255,0,0])
    prevLinkID = linkID

    # p.stepSimulation()
    # diff = (datetime.datetime.now() - t).total_seconds()
    # sleep_time = SIMULATION_TIME_STEP-diff
    # print(f"t:{diff}, st:{sleep_time}")
    # if sleep_time > 0:
    #     time.sleep(sleep_time)
    # i_step += 1

p.disconnect()