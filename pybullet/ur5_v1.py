import os, inspect
import pdb
import pybullet as p
import pybullet_data
import datetime
import time
import math

END_EFFECTOR_INDEX = 6
# RESET_JOINT_INDICES = [1, 2, 3, 4, 5, 6, 8, 10, 12, 13, 15, 17]
RESET_JOINT_INDICES = [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13]
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

def apply_action_ik(target_ee_pos, target_ee_quat, target_gripper_state,
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

    target_gripper_poses = [target_gripper_state, -target_gripper_state, target_gripper_state, -target_gripper_state, target_gripper_state, -target_gripper_state]
    joint_pose_list = []
    for joint_idx in range(6):
        joint_pose_list.append(joint_poses[joint_idx])

    if(isJoint):
        target_joint_poses = param_joint_list + target_gripper_poses
    else:
        target_joint_poses = joint_pose_list + target_gripper_poses
    max_forces=[5] * (len(movable_joints)-6) + [500]*6

    printList("\nP:", target_ee_pos)
    printList("T:", target_joint_poses)

    p.setJointMotorControlArray(robot_id,
                                movable_joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=target_joint_poses,
                                # targetVelocity=0,
                                # forces=max_forces,
                                # positionGains=[0.03] * len(movable_joints),
                                # velocityGain=1
                                )

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
    print("ID: {}".format(jointID))
    print("name: {}".format(jointName))
    print("type: {}".format(jointType))
    print("Link: {}".format(linkName))
    print("lower limit: {}".format(jointLowerLimit))
    print("upper limit: {}".format(jointUpperLimit))
print("------------------------------------------")

# get links
linkIDs = list(map(lambda linkInfo: linkInfo[1], p.getVisualShapeData(robotID)))
linkNum = len(linkIDs)


textPose = list(p.getBasePositionAndOrientation(robotID)[0])
textPose[2] += 1
x_c = 0.003
z_c = 0.047
# p.addUserDebugLine([x_c,-1,z_c], [x_c,1,z_c], [255,0,0])
p.addUserDebugText("Press \'w\' and magic!!", textPose, [255,0,0], 1)

prevLinkID = 0
linkIDIn = p.addUserDebugParameter("linkID", 0, linkNum-1e-3, 0)

ur5_joints = [1, 2, 3, 4, 5, 6]
gripper_joints = [8, 9, 10, 11, 12, 13]
movable_joints = ur5_joints + gripper_joints

ee_pos = [0.36823178249361377, 0.1090871886985425, 0.13537283344309153]
ee_rot = [179.73381141093887, -43.42871083888508, -179.62179031929355]
param_names = ["X", "Y", "Z", "R", "P", "Y"]
param_ids = []
param_ids.append(p.addUserDebugParameter(param_names[0], -0.5, 0.5, startValue=ee_pos[0]))
param_ids.append(p.addUserDebugParameter(param_names[1], -0.5, 0.5, startValue=ee_pos[1]))
param_ids.append(p.addUserDebugParameter(param_names[2], -0.5, 0.5, startValue=ee_pos[2]))
param_ids.append(p.addUserDebugParameter(param_names[3], -179, 179, startValue=ee_rot[0]))
param_ids.append(p.addUserDebugParameter(param_names[4], -179, 179, startValue=ee_rot[1]))
param_ids.append(p.addUserDebugParameter(param_names[5], -179, 179, startValue=ee_rot[2]))

init_joint_val = [0.0, -1.1898947954177856, 1.9831578731536865, 0.0, 0.0, 0.0]
param_names = ["1", "2", "3", "4", "5", "6"]
for i in range(len(init_joint_val)):
    param_ids.append(p.addUserDebugParameter(param_names[i], -3.14, 3.14, startValue=init_joint_val[i]))

param_gripper_id = p.addUserDebugParameter(f"gripper:", -0.45, 0.45)
flag = True
i_step = 0


######## Reset #########
for i, value in zip(RESET_JOINT_INDICES, RESET_JOINT_VALUES):
        p.resetJointState(robotID, i, value)



while(flag):
    # t = datetime.datetime.now()

    linkID = p.readUserDebugParameter(linkIDIn)
    
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

    apply_action_ik(
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

    if linkID!=prevLinkID:
        p.setDebugObjectColor(robotID, int(prevLinkID), [255,255,255])
        p.setDebugObjectColor(robotID, int(linkID), [255,0,0])
        print(int(linkID))
    prevLinkID = linkID

    # p.stepSimulation()
    # diff = (datetime.datetime.now() - t).total_seconds()
    # sleep_time = SIMULATION_TIME_STEP-diff
    # print(f"t:{diff}, st:{sleep_time}")
    # if sleep_time > 0:
    #     time.sleep(sleep_time)
    # i_step += 1

p.disconnect()