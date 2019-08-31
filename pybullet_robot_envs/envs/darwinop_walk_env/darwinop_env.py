import os
import pybullet as p
from pybullet_robot_envs import robot_data


class DarwinopEnv:
    def __init__(self, urdf_root_path=robot_data.getDataPath(),
                 time_step=0.01,
                 global_scale=2):
        self.urdf_root_path = os.path.join(urdf_root_path, 'darwinop/darwinOP.urdf')
        self.time_step = time_step
        self.global_scale = global_scale
        self.darwin_id = None
        self.start_position = [0, 0, 1]
        self.num_joints = 20
        self.reset()

    def reset(self):
        self.darwin_id = p.loadURDF(self.urdf_root_path,
                                    self.start_position,
                                    globalScaling=self.global_scale)
        self.num_joints = p.getNumJoints(self.darwin_id)

        # set all joints to initial values
        pass


if __name__ == '__main__':
    darwinop = DarwinopEnv()
    # pass
