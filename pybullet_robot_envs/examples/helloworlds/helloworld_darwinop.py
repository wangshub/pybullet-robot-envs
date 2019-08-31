import os, inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data
import robot_data
import time
import math as m


print('robot_data.getDataPath()', robot_data.getDataPath())


def main():

    p.connect(p.GUI)
    p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    # Set gravity for simulation
    p.setGravity(0, 0, -9.8)

    # load robot model
    cube_start_pos = [0, 0, 1]
    robot_id = p.loadURDF(os.path.join(robot_data.getDataPath(), "darwinop/darwinOP.urdf"),
                          cube_start_pos,
                          globalScaling=2)
    grav_id = p.addUserDebugParameter("gravity", -10, 10, -10)

    joint_ids = []
    param_ids = []

    for joint in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint)
        print('{num} | {name}'.format(num=joint_info[0],
                                      name=joint_info[1].decode('utf-8')))

        joint_name = joint_info[1]
        joint_type = joint_info[2]
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
            joint_ids.append(joint)
            param_ids.append(p.addUserDebugParameter(joint_name.decode("utf-8"),
                                                     joint_lower_limit, joint_upper_limit, 0))

    p.setRealTimeSimulation(1)
    while True:
        time.sleep(0.01)
        p.setGravity(0, 0, p.readUserDebugParameter(grav_id))
        for i in range(len(param_ids)):
            config = param_ids[i]
            target_pos = p.readUserDebugParameter(config)
            p.setJointMotorControl2(robot_id, joint_ids[i], p.POSITION_CONTROL, target_pos, force=5 * 240.)


if __name__ == '__main__':
    main()
