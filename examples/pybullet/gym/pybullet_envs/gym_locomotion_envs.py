from .scene_stadium import SinglePlayerStadiumScene, MultiplayerStadiumScene
from .env_bases import MJCFBaseBulletEnv
import numpy as np
import pybullet
from robot_locomotors import Hopper, Walker2D, HalfCheetah, Ant, AntMJC, AntMJC_physx, Humanoid, HumanoidFlagrun, HumanoidFlagrunHarder


class WalkerBaseBulletEnv(MJCFBaseBulletEnv):
    def __init__(self, robot, render=False, client=None, isMultiplayer=False, time_step=0.01):
        # print("WalkerBase::__init__ start")
        self.timestep = time_step
        collection_rate = 50 # samples per second
        self.frame_skip = int(1/(collection_rate*self.timestep))
        MJCFBaseBulletEnv.__init__(self, robot, render, client=client, timestep=self.timestep, frame_skip=self.frame_skip)

        self.camera_x = 0
        self.walk_target_x = 1e3  # kilometer away
        self.walk_target_y = 0
        self.stateId=-1
        self.isMultiplayer=isMultiplayer
        # self._alive = True
        self.printed_warning = False
    def create_scene(self, bullet_client):
        if self.isMultiplayer:
            self.stadium_scene = SinglePlayerStadiumScene(bullet_client, gravity=9.8, timestep=self.timestep, frame_skip=self.frame_skip)
        else:
            self.stadium_scene = MultiplayerStadiumScene(bullet_client, gravity=9.8, timestep=self.timestep, frame_skip=self.frame_skip)
        return self.stadium_scene

    def reset(self):
        # print("Reset")
        self.printed_warning = False

        # if (self.stateId>=0):
            #print("restoreState self.stateId:",self.stateId)
            # self._p.restoreState(self.stateId)

        r = MJCFBaseBulletEnv.reset(self)

        self._p.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING,0)

        # self.parts, self.jdict, self.ordered_joints, self.robot_body = self.robot.addToScene(self._p, )
        # self.ground_ids = set([(self.parts[f].bodies[self.parts[f].bodyIndex], self.parts[f].bodyPartIndex) for f in
        #                         self.foot_ground_object_names])
        self._p.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING,1)
        # if (self.stateId<0):
            # self.stateId=self._p.saveState()
            #print("saving state self.stateId:",self.stateId)
        # print("COol3")


        return r

    def _isDone(self):
        # print("ALIVE: %.3f" % self._alive )
        return self._alive < 0

    # def move_robot(self, init_x, init_y, init_z):
    #     "Used by multiplayer stadium to move sideways, to another running lane."
    #     self.cpp_robot.query_position()
    #     pose = self.cpp_robot.root_part.pose()
    #     # print("Pose: %.2f, %.2f, %.2f" % (init_x, init_y, init_z))
    #     pose.move_xyz(init_x, init_y, init_z)  # Works because robot loads around (0,0,0), and some robots have z != 0 that is left intact
    #     self.cpp_robot.set_pose(pose)

    electricity_cost     = -2.0    # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
    stall_torque_cost    = -0.1    # cost for running electric current through a motor even at zero rotational speed, small
    foot_collision_cost  = -1.0    # touches another leg, or other objects, that cost makes robot avoid smashing feet into itself
    foot_ground_object_names = set(["floor"])  # to distinguish ground and other objects
    joints_at_limit_cost = -0.1    # discourage stuck joints

    def step(self, a):
        if not self.printed_warning:
            self.printed_warning = True

            print("DONT USE FOR TRAINING. This should warning only appear at testing manually (not in evaluate env)\nStep is directly done in environment, not the parallel version")
        clipped_action = np.clip(a, self.action_space.low, self.action_space.high)
        self.robot.apply_action(clipped_action)

        for _ in range(self.frame_skip):
            self._p.stepSimulation()

        self.state = self.get_observation()  # also calculates self.joints_at_limit
        done = self.checkFall()
        reward = self.getReward()
        # self.HUD(state, a, done)
        # return -1
        return self.state, reward, done, {}

    def getReward(self):
        potential_old = self.potential
        self.potential = self.robot.calc_potential()
        progress = float(self.potential - potential_old)
        # print("Progress: ", progress)
        # print("Potential: ", self.potential)
        # print("Old Potential: ", potential_old)
        feet_collision_cost = 0.0


        # print("robot actioN: ", self.robot.action)
        # print("joitn speeds: ", self.robot.joint_speeds)
        electricity_cost  = self.electricity_cost  * float(np.abs(self.robot.action*self.robot.joint_speeds).mean())  # let's assume we have DC motor with controller, and reverse current braking
        electricity_cost += self.stall_torque_cost * float(np.square(self.robot.action).mean())
        # print("self.electricity_cost: ", self.electricity_cost)
        # print("electricity_cost: ", electricity_cost)
        # print("self.reward: ", self.reward)
        joints_at_limit_cost = float(self.joints_at_limit_cost * self.robot.joints_at_limit)

        debugmode=0
        if(debugmode):
            print("alive=")
            print(self._alive)
            print("progress")
            print(progress)
            print("electricity_cost")
            print(electricity_cost)
            print("joints_at_limit_cost")
            print(joints_at_limit_cost)
            print("feet_collision_cost")
            print(feet_collision_cost)

        self.rewards = [
            self._alive,
            progress,
            electricity_cost,
            joints_at_limit_cost,
            feet_collision_cost
            ]
        if (debugmode):
            print("rewards=")
            print(self.rewards)
            print("sum rewards")
            print(sum(self.rewards))
        # self.reward += sum(self.rewards)
        # print("Reward: ", sum(self.rewards))
        return sum(self.rewards), {}

    def checkFall(self):
        done = self._isDone()

        # check for simulation error where the robot penetrates the ground

        
        # parts_z = np.array([p.pose().xyz()[2] for p in self.robot.parts.values()]).flatten()
        # print("parts z: ", parts_z)
        # if np.any(np.less(parts_z, 0)):
        #     done = True
        #     print("PARTS BELOW: ", parts_z)

        if not np.isfinite(self.state).all():
            print("~INF~", self.state)
            done = True
        return done
    def get_observation(self):

        self.state = self.robot.calc_state()
        self._alive = float(self.robot.alive_bonus(self.state[0]+self.robot.initial_z, self.robot.body_rpy[1]))   # state[0] is body height above ground, body_rpy[1] is pitch
        # print("ALive: ", self._alive)
        return self.robot.calc_state()  # also calculates self.joints_at_limit

    def camera_adjust(self):
        x, y, z = self.body_xyz
        self.camera_x = 0.98*self.camera_x + (1-0.98)*x
        self.camera.move_and_look_at(self.camera_x, y-2.0, 1.4, x, y, 1.0)

class HopperBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False, time_step=0.01):
        self.robot = Hopper(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision)
        WalkerBaseBulletEnv.__init__(self, self.robot, render, client=client, time_step=time_step)

class Walker2DBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False, time_step=0.01):
        self.robot = Walker2D(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision)
        WalkerBaseBulletEnv.__init__(self, self.robot, render, client=client, time_step=time_step)

class HalfCheetahBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False, time_step=0.01):
        self.robot = HalfCheetah(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision)
        WalkerBaseBulletEnv.__init__(self, self.robot, render, client=client, time_step=time_step)

    def _isDone(self):
        return False

class AntBulletEnvMJC(WalkerBaseBulletEnv):

  def __init__(self, render=False):
    self.robot = AntMJC()
    WalkerBaseBulletEnv.__init__(self, self.robot, render)

class AntBulletEnvMJC_physx(WalkerBaseBulletEnv):

  def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False):
    self.robot = AntMJC_physx(isPhysx=isPhysx, self_collision=self_collision)
    WalkerBaseBulletEnv.__init__(self, self.robot, render, client=client)


class AntBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, client, robot_setup=None, render=False, pos = [0,0,0], isPhysx=False,self_collision=False, time_step=0.01):
        self.robot = Ant(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision, robot_setup=robot_setup)
        WalkerBaseBulletEnv.__init__(self, self.robot, render, client=client, time_step=time_step)

class HumanoidBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False, robot=None, time_step=0.01):
        self.robot = Humanoid(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision) if robot is None else robot
        # self.robot = Humanoid()
        WalkerBaseBulletEnv.__init__(self, self.robot, render, client=client, time_step=time_step)
        self.electricity_cost  = 4.25*WalkerBaseBulletEnv.electricity_cost
        self.stall_torque_cost = 4.25*WalkerBaseBulletEnv.stall_torque_cost

class HumanoidFlagrunBulletEnv(HumanoidBulletEnv):
    random_yaw = True

    def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False):
        self.robot = HumanoidFlagrun(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision)
        HumanoidBulletEnv.__init__(self, robot=self.robot, render=render, client=client)

    def create_single_player_scene(self, bullet_client):
        s = HumanoidBulletEnv.create_scene(self, bullet_client)
        s.zero_at_running_strip_start_line = False
        return s

class HumanoidFlagrunHarderBulletEnv(HumanoidBulletEnv):
    random_lean = True  # can fall on start

    def __init__(self, client, render=False, pos = [0,0,0], isPhysx=False,self_collision=False):
        self.robot = HumanoidFlagrunHarder(basePosition=pos, fixed_base=False, isPhysx=isPhysx, self_collision=self_collision) if robot is None else robot
        self.electricity_cost /= 4   # don't care that much about electricity, just stand up!
        HumanoidBulletEnv.__init__(self, robot=self.robot, render=render, client=client)

    def create_single_player_scene(self, bullet_client):
        s = HumanoidBulletEnv.create_scene(self, bullet_client)
        s.zero_at_running_strip_start_line = False
        return s
