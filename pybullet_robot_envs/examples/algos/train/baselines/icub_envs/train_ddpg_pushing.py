#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print(currentdir)
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)


from pybullet_robot_envs.envs.icub_envs.icub_push_gym_env import iCubPushGymEnv
from pybullet_robot_envs import robot_data

import math as m
import numpy as np
import random

import tensorflow as tf
# import RL agent
from stable_baselines.ddpg.noise import NormalActionNoise
from stable_baselines import DDPG
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.bench import Monitor


def set_global_seed(seed):
    """
    set the seed for python random, numpy and tensorflow
    :param seed: (int) the seed
    """
    tf.set_random_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

log_dir = '../pybullet_logs/icubpush_ddpg'
best_mean_reward, n_steps = -np.inf, 0
def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 1000 calls
    if (n_steps + 1) % 500 == 0:
        # Evaluate policy training performance
        x, y = ts2xy(load_results(os.path.join(log_dir,'log')), 'timesteps')
        if len(x) > 0:
            mean_reward = np.mean(y[-100:])
            print(x[-1], 'timesteps')
            print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(best_mean_reward, mean_reward))

            # New best model, you could save the agent here
            if mean_reward > best_mean_reward:
                best_mean_reward = mean_reward
                print("Saving new best model")
                _locals['self'].save(os.path.join(log_dir,'best_model.pkl'))
    n_steps += 1
    # Returning False will stop training early
    return True

def main():

    # create Environment
    env = iCubPushGymEnv(urdfRoot=robot_data.getDataPath(), renders=False, useIK=1,
                        isDiscrete=0, rnd_obj_pose=0, maxSteps=2000, reward_type=0)

    # set seed
    seed = 1
    tf.reset_default_graph()
    set_global_seed(seed)
    env.seed(seed)

    # set log
    monitor_dir = os.path.join(log_dir,'log')
    os.makedirs(monitor_dir, exist_ok=True)
    env = Monitor(env, monitor_dir+'/', allow_early_resets=True)

    # create agent model
    nb_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(nb_actions), sigma=float(0.5373) * np.ones(nb_actions))

    model = DDPG('LnMlpPolicy', env, action_noise=action_noise, gamma=0.99, batch_size=16,
                normalize_observations=True,normalize_returns=False, memory_limit=100000,
                verbose=1, tensorboard_log=os.path.join(log_dir,'tb'),full_tensorboard_log=False)

    #start learning
    model.learn(total_timesteps=500000, seed=seed, callback=callback)

    # save model
    print("Saving model.pkl to ",log_dir)
    act.save(log_dir+"/final_model.pkl")

if __name__ == '__main__':
  main()
