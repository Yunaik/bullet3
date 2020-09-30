from pybullet_envs.gym_locomotion_envs import HumanoidBulletEnv

env = HumanoidBulletEnv()

env.reset()

while True:
    env.render()