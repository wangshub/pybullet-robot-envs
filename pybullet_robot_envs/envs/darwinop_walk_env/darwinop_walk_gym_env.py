import os
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
import pybullet_data

from pybullet_robot_envs import robot_data
from pybullet_robot_envs.envs.darwinop_walk_env.darwinop_env import DarwinopEnv


def camera_look_at(target, distance=10, yaw=10):
    assert len(target) == 3, 'need [x, y, z]'
    p.resetDebugVisualizerCamera(distance, yaw, -20, target)


class DarwinopWalkGymEnv(gym.Env):
    def __init__(self, is_render=False):
        self.np_random = 0
        self.is_render = is_render

        self._p = p
        self._cid = self.init_simulator()
        self._time_step = 1. / 240.
        self.terminated = 0
        self._darwin = None
        self._observation = []

        self.seed()
        self.reset()

    def init_simulator(self):
        """ initialize simulation environment

        :return: cid
        """
        if self.is_render:
            p.connect(p.SHARED_MEMORY)
            cid = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(2.5, 90, -60, [0.0, -0.0, -0.0])
        else:
            cid = p.connect(p.DIRECT)
        return cid

    def reset(self):
        """reset gym pybullet simulation"""
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(self._time_step)
        p.setGravity(0, 0, -9.8)

        # load plane
        p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0, 0, 0])
        # load robot
        self._darwin = DarwinopEnv()

        # Let the world run for a bit
        for _ in range(20):
            p.stepSimulation()

    def step(self, action):
        pass

    def render(self, mode='human'):
        pass

    def get_observation(self):
        """get robot observations"""
        return None

    def _compute_reward(self):
        return 0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def debug_gui(self):
        pass


if __name__ == '__main__':
    env = DarwinopWalkGymEnv()
