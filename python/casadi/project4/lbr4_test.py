"""
This is a test script to ensure basic communication with the V-REP simulation
is working. Running the script should case the robot arm to move through a
pre-defined sequence of arm positions.
"""
import sys
import time
import numpy as np
try:
    import vrep
except:
    print(
        "Import of vrep failed. Make sure the 'vrep.py' file is in this directory."
    )
    sys.exit(1)

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID != -1:
    print("Successfully connected to remote API server.")
    vrep.simxSynchronous(clientID, True)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

    joint_names = ["LBR4p_joint{}".format(i) for i in range(1, 8)]
    joint_handles = [
        vrep.simxGetObjectHandle(clientID, joint, vrep.simx_opmode_blocking)[1]
        for joint in joint_names
    ]

    print("Arm should be moving right now...")
    angles = np.linspace(0, np.pi / 2, 200)
    for angle in angles:
        for i in range(7):
            vrep.simxSetJointTargetPosition(clientID, joint_handles[i], angle,
                                            vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(clientID)
        time.sleep(0.01)
    for angle in np.flip(angles, axis=0):
        for i in range(7):
            vrep.simxSetJointTargetPosition(clientID, joint_handles[i], angle,
                                            vrep.simx_opmode_oneshot)
        vrep.simxSynchronousTrigger(clientID)
        time.sleep(0.01)

    print("Simulation complete.")
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
    vrep.simxGetPingTime(clientID)
    vrep.simxFinish(clientID)
else:
    print("Failed connecting to remote API server")
