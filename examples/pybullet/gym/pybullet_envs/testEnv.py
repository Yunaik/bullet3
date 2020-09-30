from pybullet_envs.gym_locomotion_envs import HumanoidBulletEnv

# print("JAEKJ")
env = HumanoidBulletEnv(render=True)
# print("OK")
env.reset()

while True:
    env.render()