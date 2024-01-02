import os, inspect
import pdb
import pybullet as p
import pybullet_data
import datetime
import time

SIMULATION_TIME_STEP = 0.02
# SIMULATION_TIME_STEP = 1.0/240

#Getting Absolute Path from Relative Path of URDF file
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

# robotUrdfPath = os.path.join(currentdir, "./urdf/exp.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/robotiq_2f_85.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/robotiq_2f_85_actual_mesh.urdf")
robotUrdfPath = os.path.join(currentdir, "./ur5_robotiq/urdf/ur5_gripper.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/ur5.urdf")
# robotUrdfPath = os.path.join(currentdir, "./urdf/gripper_model_precise.urdf")

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
    print("\tID: {}".format(jointID))
    print("\tname: {}".format(jointName))
    print("\ttype: {}".format(jointType))
    print("\tLink: {}".format(linkName))
    print("\tlower limit: {}".format(jointLowerLimit))
    print("\tupper limit: {}".format(jointUpperLimit))
print("------------------------------------------")

# get links
linkIDs = list(map(lambda linkInfo: linkInfo[1], p.getVisualShapeData(robotID)))
linkNum = len(linkIDs)

# start simulation
try:
    flag = True

    textPose = list(p.getBasePositionAndOrientation(robotID)[0])
    textPose[2] += 1
    x_c = 0.003
    z_c = 0.047
    p.addUserDebugLine([x_c,-1,z_c], [x_c,1,z_c], [255,0,0])
    p.addUserDebugText("Press \'w\' and magic!!", textPose, [255,0,0], 1)

    prevLinkID = 0
    linkIDIn = p.addUserDebugParameter("linkID", 0, linkNum-1e-3, 0)

    ur5_joints = [1, 2, 3, 4, 5, 6]
    gripper_joints = [8, 10, 12, 13, 15, 17]
    movable_joints = ur5_joints + gripper_joints

    param_ur5joints_id = []
    for joint_num in ur5_joints:
        jointInfo = p.getJointInfo(robotID, joint_num)
        jointName = jointInfo[1].decode("utf-8")
        jointName = jointName[0:-5]
        param_ur5joints_id.append(p.addUserDebugParameter(f"{joint_num}: {jointName}: ", -3.14, 3.14))

    param_gripper_id = p.addUserDebugParameter(f"gripper:", 0, 0.7)

    i_step = 0
    while(flag):
        t = datetime.datetime.now()

        linkID = p.readUserDebugParameter(linkIDIn)
        ur5_target_positions = []
        for param_angle_id in param_ur5joints_id:
            theta = p.readUserDebugParameter(param_angle_id)
            ur5_target_positions.append(theta)
        theta = p.readUserDebugParameter(param_gripper_id)
        gripper_target_positions = [theta, -theta, theta, theta, -theta, theta]
        target_positions = ur5_target_positions+gripper_target_positions

        p.setJointMotorControlArray(
            bodyUniqueId=robotID,
            jointIndices=movable_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions = target_positions)

        if linkID!=prevLinkID:
            p.setDebugObjectColor(robotID, int(prevLinkID), [255,255,255])
            p.setDebugObjectColor(robotID, int(linkID), [255,0,0])
            print(int(linkID))
        prevLinkID = linkID

        p.stepSimulation()
        diff = (datetime.datetime.now() - t).total_seconds()
        sleep_time = SIMULATION_TIME_STEP-diff
        # print(f"t:{diff}, st:{sleep_time}")
        if sleep_time > 0:
            time.sleep(sleep_time)
        # i_step += 1

    p.disconnect()

except Exception as e:

    print("\n############### Something Went Wrong in Execution ################\n")

    print("------------------ Error Message ----------------")
    print(e)
    print("-------------------------------------------------")

    print("------------------ Disconnecting Pybullet ----------------")
    if(p.isConnected()):
        p.disconnect()
    print("----------------------------------------------------------")

    print("\n############### Program Ended with Error ###############\n")