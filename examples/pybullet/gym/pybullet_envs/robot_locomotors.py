from robot_bases import XmlBasedRobot, MJCFBasedRobot, URDFBasedRobot
import numpy as np
import pybullet 
import os
import pybullet_data
from robot_bases import BodyPart
import gym

class WalkerBaseURDF(URDFBasedRobot):
    def __init__(self,  fn, robot_name, action_dim, obs_dim, power, 
                player_n=0, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], 
                self_collision=False, fixed_base=False, isPhysx=False, robot_setup=None):
        URDFBasedRobot.__init__(self, fn, robot_name, action_dim, obs_dim, 
                                basePosition=basePosition, baseOrientation=baseOrientation, 
                                self_collision=self_collision, fixed_base=fixed_base, 
                                isPhysx=isPhysx, robot_setup=robot_setup)
        self.power = power
        self.camera_x = 0
        self.start_pos_x, self.start_pos_y, self.start_pos_z = 0, 0, 0
        self.walk_target_x = 1e3  # kilometer away
        self.walk_target_y = 0
        self.body_xyz=[0,0,0]
        self.player_n = player_n
        self.joint_limits = {}

    def robot_specific_reset(self, bullet_client):
        # print("RESET")
        self._p = bullet_client
        for j in self.ordered_joints:
            j.reset_current_position(self.np_random.uniform(low=-0.1, high=0.1), 0)
            j.maxForce = self.power * j.power_coef
            # print("Power coeff: ", j.power_coef)
            j.set_PD_gains(j.maxForce)
            # print("Max force: ", j.maxForce)
            # print("Joint name: %s" % j.joint_name)
            if self.joint_limits:
                j.lowerLimit = self.joint_limits[j.joint_name][0]*3.14/180.
                j.upperLimit = self.joint_limits[j.joint_name][1]*3.14/180.
            # print("MAX FORCE: %.2f. Kp: %.1f, Kd: %.1f, Joint limit: [%.2f, %.2f]" % (j.maxForce, j.Kp, j.Kd, self.joint_limits[j.joint_name][0], self.joint_limits[j.joint_name][1]))
        limit_low = []
        limit_high = []
        for joint in self.ordered_joints:
            name = joint.joint_name
            limit_low.append(self.joint_limits[name][0])
            limit_high.append(self.joint_limits[name][1])

        self.action_space = gym.spaces.Box(np.array(limit_low)*3.14/180., np.array(limit_high)*3.14/180.)
        
        self.feet = [self.parts[f] for f in self.foot_list]
        self.feet_contact = np.array([0.0 for f in self.foot_list], dtype=np.float32)
        self.scene.actor_introduce(self)
        self.initial_z = None

    def apply_action(self, a):
        # print("A raw: ", a)
        self.action = a
        assert (np.isfinite(a).all())
        for n, j in enumerate(self.ordered_joints):
            # j.set_velocity(a[n])
            # j.set_motor_torque(self.power * j.power_coef * float(np.clip(a[n], -1, +1)))
            j.set_position(a[n])
            
            # print("J lower: %.2f, j upper: %.2f" % (j.lowerLimit, j.upperLimit))
            # j.set_velocity(self.power * j.power_coef * float(np.clip(a[n], -1, +1)))
        # print("Action: ", self.action)
        # print("Action applied: ", a)

    def calc_state(self):
        j = np.array([j.current_relative_position() for j in self.ordered_joints], dtype=np.float32).flatten()
        self.joint_position = j
        # even elements [0::2] position, scaled to -1..+1 between limits
        # odd elements  [1::2] angular speed, scaled to show -1..+1
        self.joint_speeds = j[1::2]
        self.joints_at_limit = np.count_nonzero(np.abs(j[0::2]) > 0.99)

        body_pose = self.robot_body.pose()
        # print("Body pos: ", body_pose)
        parts_xyz = np.array([p.pose().xyz() for p in self.parts.values()]).flatten()
        self.body_xyz = (
        parts_xyz[0::3].mean(), parts_xyz[1::3].mean(), body_pose.xyz()[2])  # torso z is more informative than mean z
        self.body_rpy = body_pose.rpy()
        # print("body rpy: ", self.body_rpy)
        # print("Body xyz: ", self.body_xyz)

        z = self.body_xyz[2]
        if self.initial_z == None:
            self.initial_z = z
        r, p, yaw = self.body_rpy
        self.walk_target_theta = np.arctan2(self.walk_target_y - self.body_xyz[1],
                                            self.walk_target_x - self.body_xyz[0])
        self.walk_target_dist = np.linalg.norm(
            [self.walk_target_y - self.body_xyz[1], self.walk_target_x - self.body_xyz[0]])
        angle_to_target = self.walk_target_theta - yaw

        rot_speed = np.array(
            [[np.cos(-yaw), -np.sin(-yaw), 0],
             [np.sin(-yaw), np.cos(-yaw), 0],
             [        0,             0, 1]]
        )
        vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view

        more = np.array([ z-self.initial_z,
            np.sin(angle_to_target), np.cos(angle_to_target),
            0.3* vx , 0.3* vy , 0.3* vz ,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
            r, p], dtype=np.float32)
        # print("Feet contact: ", self.feet_contact)
        return np.clip( np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

    def calc_potential(self):
        # progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
        # all rewards have rew/frame units and close to 1.0
        debugmode=0
        if (debugmode):
            print("calc_potential: self.walk_target_dist")
            print(self.walk_target_dist)
            print("self.scene.dt")
            print(self.scene.dt)
            print("self.scene.frame_skip")
            print(self.scene.frame_skip)
            print("self.scene.timestep")
            print(self.scene.timestep)
        return - self.walk_target_dist / self.scene.dt

class WalkerBase(MJCFBasedRobot):
    def __init__(self,  fn, robot_name, action_dim, obs_dim, power, player_n=0, isPhysx=False):
        MJCFBasedRobot.__init__(self, fn, robot_name, action_dim, obs_dim, isPhysx=isPhysx)
        self.power = power
        self.camera_x = 0
        self.start_pos_x, self.start_pos_y, self.start_pos_z = 0, 0, 0
        self.walk_target_x = 1e3  # kilometer away
        self.walk_target_y = 0
        self.body_xyz=[0,0,0]
        self.player_n = player_n

    def robot_specific_reset(self, bullet_client):
        self._p = bullet_client
        for j in self.ordered_joints:
            j.reset_current_position(self.np_random.uniform(low=-0.1, high=0.1), 0)

        self.feet = [self.parts[f] for f in self.foot_list]
        self.feet_contact = np.array([0.0 for f in self.foot_list], dtype=np.float32)
        self.scene.actor_introduce(self)
        self.initial_z = None

    def apply_action(self, a):

        self.action = a
        assert (np.isfinite(a).all())
        for n, j in enumerate(self.ordered_joints):
            j.set_motor_torque(self.power * j.power_coef * float(np.clip(a[n], -1, +1)))

    def calc_state(self):
        j = np.array([j.current_relative_position() for j in self.ordered_joints], dtype=np.float32).flatten()
        # even elements [0::2] position, scaled to -1..+1 between limits
        # odd elements  [1::2] angular speed, scaled to show -1..+1
        self.joint_speeds = j[1::2]
        self.joints_at_limit = np.count_nonzero(np.abs(j[0::2]) > 0.99)

        body_pose = self.robot_body.pose()
        parts_xyz = np.array([p.pose().xyz() for p in self.parts.values()]).flatten()
        self.body_xyz = (
        parts_xyz[0::3].mean(), parts_xyz[1::3].mean(), body_pose.xyz()[2])  # torso z is more informative than mean z
        self.body_rpy = body_pose.rpy()
        z = self.body_xyz[2]
        if self.initial_z == None:
            self.initial_z = z
        r, p, yaw = self.body_rpy
        self.walk_target_theta = np.arctan2(self.walk_target_y - self.body_xyz[1],
                                            self.walk_target_x - self.body_xyz[0])
        self.walk_target_dist = np.linalg.norm(
            [self.walk_target_y - self.body_xyz[1], self.walk_target_x - self.body_xyz[0]])
        angle_to_target = self.walk_target_theta - yaw

        rot_speed = np.array(
            [[np.cos(-yaw), -np.sin(-yaw), 0],
             [np.sin(-yaw), np.cos(-yaw), 0],
             [        0,             0, 1]]
        )
        vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view

        more = np.array([ z-self.initial_z,
            np.sin(angle_to_target), np.cos(angle_to_target),
            0.3* vx , 0.3* vy , 0.3* vz ,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
            r, p], dtype=np.float32)
        return np.clip( np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

    def calc_potential(self):
        # progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
        # all rewards have rew/frame units and close to 1.0
        debugmode=0
        if (debugmode):
            print("calc_potential: self.walk_target_dist")
            print(self.walk_target_dist)
            print("self.scene.dt")
            print(self.scene.dt)
            print("self.scene.frame_skip")
            print(self.scene.frame_skip)
            print("self.scene.timestep")
            print(self.scene.timestep)
        return - self.walk_target_dist / self.scene.dt


class Hopper(WalkerBaseURDF):
    foot_list = ["foot"]

    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False):
        WalkerBaseURDF.__init__(self, "hopper.urdf", "torso", action_dim=3, obs_dim=15, power=0.75, self_collision=self_collision, 
                            basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx)
        self.joint_limits = {
            "thigh_joint":   [-150, 0],
            "leg_joint"  :   [-150, 0],
            "foot_joint" :   [-45 ,45],}
    def alive_bonus(self, z, pitch):
        return +1 if z > 0.8 and abs(pitch) < 1.0 else -1


class Walker2D(WalkerBaseURDF):
    foot_list = ["foot", "foot_left"]

    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False):
        WalkerBaseURDF.__init__(self,  "walker2d.urdf", "torso", action_dim=6, obs_dim=22, power=0.40, self_collision=self_collision, 
                            basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx)
        self.joint_limits = {
            "thigh_joint":        [  -150, 0],
            "leg_joint":          [  -150, 0],
            "foot_joint":         [ -45 ,45],
            "thigh_left_joint":   [  -150, 0],
            "leg_left_joint":     [  -150, 0],
            "foot_left_joint":    [  -45, 45],}
    def alive_bonus(self, z, pitch):
        return +1 if z > 0.8 and abs(pitch) < 1.0 else -1

    def robot_specific_reset(self, bullet_client):
        WalkerBaseURDF.robot_specific_reset(self, bullet_client)
        for n in ["foot_joint", "foot_left_joint"]:
            self.jdict[n].power_coef = 30.0


class HalfCheetah(WalkerBaseURDF):
    foot_list = ["ffoot", "fshin", "fthigh",  "bfoot", "bshin", "bthigh"]  # track these contacts with ground

    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False):
        WalkerBaseURDF.__init__(self, "half_cheetah.urdf", "torso", action_dim=6, obs_dim=26, power=0.9, self_collision=self_collision, 
                            basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx)
        self.joint_limits = {
            "bthigh":    [-.52  * 180/3.14, 180/3.14* 1.05],
            "bshin" :    [-.785 * 180/3.14, 180/3.14* .785],
            "bfoot" :    [-.4   * 180/3.14, 180/3.14*.785 ],
            "fthigh":    [-1.5  * 180/3.14, 180/3.14* 0.8],
            "fshin" :    [-1.2  * 180/3.14, 180/3.14* 1.1],
            "ffoot" :    [-3.1  * 180/3.14, 180/3.14* -0.3],}
    def alive_bonus(self, z, pitch):
        # Use contact other than feet to terminate episode: due to a lot of strange walks using knees
        return +1 if np.abs(pitch) < 1.0 and not self.feet_contact[1] and not self.feet_contact[2] and not self.feet_contact[4] and not self.feet_contact[5] else -1

    def robot_specific_reset(self, bullet_client):
        # print("HI")
        WalkerBaseURDF.robot_specific_reset(self, bullet_client)
        # print("HI2")
        self.jdict["bthigh"].power_coef = 120.0
        self.jdict["bshin"].power_coef  = 90.0
        self.jdict["bfoot"].power_coef  = 60.0
        self.jdict["fthigh"].power_coef = 140.0
        self.jdict["fshin"].power_coef  = 60.0
        self.jdict["ffoot"].power_coef  = 30.0


class AntMJC(WalkerBase):
  foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']

  def __init__(self):
    WalkerBase.__init__(self, "ant.xml", "torso", action_dim=8, obs_dim=28, power=2.5)

  def alive_bonus(self, z, pitch):
    return +1 if z > 0.26 else -1  # 0.25 is central sphere rad, die if it scrapes the ground

class AntMJC_physx(WalkerBase):
  foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']

  def __init__(self, isPhysx=False,self_collision=False):
    WalkerBase.__init__(self, "ant.xml", "torso", action_dim=8, obs_dim=28, power=2.5, isPhysx=isPhysx)

  def alive_bonus(self, z, pitch):
    return +1 if z > 0.26 else -1  # 0.25 is central sphere rad, die if it scrapes the ground


class Ant(WalkerBaseURDF):
    foot_list = ['front_left_foot', 'front_right_foot', 'left_back_foot', 'right_back_foot']

    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False, robot_setup=None):
        WalkerBaseURDF.__init__(self, "ant_torso.urdf", "torso", action_dim=8, obs_dim=28, power=2.5, self_collision=self_collision, 
                            basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx, robot_setup=robot_setup)
        self.joint_limits = {"hip_1": [-40, 40],
                            "ankle_1": [30 ,100],
                            "hip_2": [-40, 40],
                            "ankle_2": [-100 ,-30],
                            "hip_3": [-40, 40],
                            "ankle_3": [-100 ,-30],
                            "hip_4": [-40, 40],
                            "ankle_4": [30 ,100],}

    def alive_bonus(self, z, pitch):
        alive_reward = 1 if ( z > 0.26 and (abs(pitch) < 45*3.14/180)) else -1
        # if alive_reward == -1:
        # print("Alive: %d, z: %.2f, pitch: %.2f" % (z > 0.26, z, pitch*180/3.14))
        return alive_reward  # 0.25 is central sphere rad, die if it scrapes the ground


class Humanoid(WalkerBaseURDF):
    foot_list = ["right_foot", "left_foot"]  # "left_hand", "right_hand"

    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False):
        WalkerBaseURDF.__init__(self,  'humanoid_torso.urdf', 'torso', action_dim=17, obs_dim=44, power=0.41, self_collision=self_collision, 
                            basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx)
        self.joint_limits = {
            "right_knee":      [-160  , -2],
            "left_knee":       [-160  , -2],
            "abdomen_z":       [-45   , 45],
            "abdomen_y":       [-75   , 30],
            "abdomen_x":       [-35   , 35],
            "right_hip_x":     [-25   , 5],
            "right_hip_z":     [-60   , 35],
            "right_hip_y":     [-120  , 20],
            "left_hip_x":      [-25   , 5],
            "left_hip_z":      [-60   , 35],
            "left_hip_y":      [-120  , 20],
            "right_elbow":     [-90   , 50],
            "right_shoulder1": [-85, 60],
            "right_shoulder2": [-85, 60],
            "left_shoulder1":  [-60, 85],
            "left_shoulder2":  [-60, 85],
            "left_elbow":      [-90, 50],}

    def robot_specific_reset(self, bullet_client):
        # print("Hello1")
        

        

        # WalkerBase.robot_specific_reset(self, bullet_client)
        WalkerBaseURDF.robot_specific_reset(self, bullet_client)
        # print("Hello2")
        self.motor_names  = ["abdomen_z", "abdomen_y", "abdomen_x"]
        self.motor_power  = [100, 100, 100]
        self.motor_names += ["right_hip_x", "right_hip_z", "right_hip_y", "right_knee"]
        self.motor_power += [100, 100, 300, 200]
        self.motor_names += ["left_hip_x", "left_hip_z", "left_hip_y", "left_knee"]
        self.motor_power += [100, 100, 300, 200]
        self.motor_names += ["right_shoulder1", "right_shoulder2", "right_elbow"]
        self.motor_power += [75, 75, 75]
        self.motor_names += ["left_shoulder1", "left_shoulder2", "left_elbow"]
        self.motor_power += [75, 75, 75]
        self.motors = [self.jdict[n] for n in self.motor_names]

        self.motor_power_dict = dict(zip( self.motor_names, self.motor_power))
        if self.random_yaw:
            position = [0,0,0]
            orientation = [0,0,0]
            yaw = self.np_random.uniform(low=-3.14, high=3.14)
            if self.random_lean and self.np_random.randint(2)==0:
                cpose.set_xyz(0, 0, 1.4)
                if self.np_random.randint(2)==0:
                    pitch = np.pi/2
                    position = [0, 0, 0.45]
                else:
                    pitch = np.pi*3/2
                    position = [0, 0, 0.25]
                roll = 0
                orientation = [roll, pitch, yaw]
            else:
                position = [0, 0, 1.4]
                orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
            self.robot_body.reset_position(position)
            self.robot_body.reset_orientation(orientation)
        self.initial_z = 0.8

        for name in self.motor_names:
            j = self.jdict[name]
            j.maxForce = self.motor_power_dict[name]
            # print("Power coeff: ", j.power_coef)
            j.set_PD_gains(j.maxForce)


    random_yaw = False
    random_lean = False

    def apply_action(self, a):
        self.action = a
        assert( np.isfinite(a).all() )
        for n, j in enumerate(self.ordered_joints):
            # print("%s: " %j.joint_name)
            j.set_position(a[n])
        # force_gain = 1
        # for i, m, power in zip(range(17), self.motors, self.motor_power):
        
            # m.set_motor_torque(float(force_gain * power * self.power * np.clip(a[i], -1, +1)))

    def alive_bonus(self, z, pitch):
        return +2 if z > 0.78 else -1   # 2 here because 17 joints produce a lot of electricity cost just from policy noise, living must be better than dying




def get_cube(_p, x, y, z):    
    body = _p.loadURDF(os.path.join(pybullet_data.getDataPath(),"cube_small.urdf"), [x, y, z])
    _p.changeDynamics(body,-1, mass=1.2)#match Roboschool
    part_name, _ = _p.getBodyInfo(body)
    part_name = part_name.decode("utf8")
    bodies = [body]
    return BodyPart(_p, part_name, bodies, 0, -1)


def get_sphere(_p, x, y, z):
    body = _p.loadURDF(os.path.join(pybullet_data.getDataPath(),"sphere2red_nocol.urdf"), [x, y, z])
    part_name, _ = _p.getBodyInfo(body)
    part_name = part_name.decode("utf8")
    bodies = [body]
    return BodyPart(_p, part_name, bodies, 0, -1)

class HumanoidFlagrun(Humanoid):
    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False):
        Humanoid.__init__(self, basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx,self_collision=self_collision)
        self.flag = None
        
    def robot_specific_reset(self, bullet_client):
        Humanoid.robot_specific_reset(self, bullet_client)
        self.flag_reposition()

    def flag_reposition(self):
        self.walk_target_x = self.np_random.uniform(low=-self.scene.stadium_halflen,   high=+self.scene.stadium_halflen)
        self.walk_target_y = self.np_random.uniform(low=-self.scene.stadium_halfwidth, high=+self.scene.stadium_halfwidth)
        more_compact = 0.5  # set to 1.0 whole football field
        self.walk_target_x *= more_compact
        self.walk_target_y *= more_compact
        
        if (self.flag):
            #for b in self.flag.bodies:
            #    print("remove body uid",b)
            #    p.removeBody(b)
            self._p.resetBasePositionAndOrientation(self.flag.bodies[0],[self.walk_target_x, self.walk_target_y, 0.7],[0,0,0,1])
        else:
            self.flag = get_sphere(self._p, self.walk_target_x, self.walk_target_y, 0.7)
        self.flag_timeout = 600/self.scene.frame_skip #match Roboschool 

    def calc_state(self):
        self.flag_timeout -= 1
        state = Humanoid.calc_state(self)
        if self.walk_target_dist < 1 or self.flag_timeout <= 0:
            self.flag_reposition()
            state = Humanoid.calc_state(self)  # caclulate state again, against new flag pos
            self.potential = self.calc_potential()       # avoid reward jump
        return state


class HumanoidFlagrunHarder(HumanoidFlagrun):
    def __init__(self, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1], player_n=0, fixed_base=False, isPhysx=False,self_collision=False):
        HumanoidFlagrun.__init__(self, basePosition=basePosition, baseOrientation=baseOrientation, fixed_base=fixed_base, isPhysx=isPhysx,self_collision=self_collision)
        self.flag = None
        self.aggressive_cube = None
        self.frame = 0

    def robot_specific_reset(self, bullet_client):

        HumanoidFlagrun.robot_specific_reset(self, bullet_client)
        
        self.frame = 0
        if (self.aggressive_cube):
            self._p.resetBasePositionAndOrientation(self.aggressive_cube.bodies[0],[-1.5,0,0.05],[0,0,0,1])
        else:
            self.aggressive_cube = get_cube(self._p, -1.5,0,0.05)
        self.on_ground_frame_counter = 0
        self.crawl_start_potential = None
        self.crawl_ignored_potential = 0.0
        self.initial_z = 0.8

    def alive_bonus(self, z, pitch):
        if self.frame%30==0 and self.frame>100 and self.on_ground_frame_counter==0:
            target_xyz  = np.array(self.body_xyz)
            robot_speed = np.array(self.robot_body.speed())
            angle = self.np_random.uniform(low=-3.14, high=3.14)
            from_dist   = 4.0
            attack_speed   = self.np_random.uniform(low=20.0, high=30.0)  # speed 20..30 (* mass in cube.urdf = impulse)
            time_to_travel = from_dist / attack_speed
            target_xyz += robot_speed*time_to_travel  # predict future position at the moment the cube hits the robot
            position = [target_xyz[0] + from_dist*np.cos(angle),
                target_xyz[1] + from_dist*np.sin(angle),
                target_xyz[2] + 1.0]
            attack_speed_vector = target_xyz - np.array(position)
            attack_speed_vector *= attack_speed / np.linalg.norm(attack_speed_vector)
            attack_speed_vector += self.np_random.uniform(low=-1.0, high=+1.0, size=(3,))
            self.aggressive_cube.reset_position(position)
            self.aggressive_cube.reset_velocity(linearVelocity=attack_speed_vector)
        if z < 0.8:
            self.on_ground_frame_counter += 1
        elif self.on_ground_frame_counter > 0:
            self.on_ground_frame_counter -= 1
        # End episode if the robot can't get up in 170 frames, to save computation and decorrelate observations.
        self.frame += 1
        return self.potential_leak() if self.on_ground_frame_counter<170 else -1

    def potential_leak(self):
        z = self.body_xyz[2]          # 0.00 .. 0.8 .. 1.05 normal walk, 1.2 when jumping
        z = np.clip(z, 0, 0.8)
        return z/0.8 + 1.0            # 1.00 .. 2.0

    def calc_potential(self):
        # We see alive bonus here as a leak from potential field. Value V(s) of a given state equals
        # potential, if it is topped up with gamma*potential every frame. Gamma is assumed 0.99.
        #
        # 2.0 alive bonus if z>0.8, potential is 200, leak gamma=0.99, (1-0.99)*200==2.0
        # 1.0 alive bonus on the ground z==0, potential is 100, leak (1-0.99)*100==1.0
        #
        # Why robot whould stand up: to receive 100 points in potential field difference.
        flag_running_progress = Humanoid.calc_potential(self)

        # This disables crawl.
        if self.body_xyz[2] < 0.8:
            if self.crawl_start_potential is None:
                self.crawl_start_potential = flag_running_progress - self.crawl_ignored_potential
                #print("CRAWL START %+0.1f %+0.1f" % (self.crawl_start_potential, flag_running_progress))
            self.crawl_ignored_potential = flag_running_progress - self.crawl_start_potential
            flag_running_progress  = self.crawl_start_potential
        else:
            #print("CRAWL STOP %+0.1f %+0.1f" % (self.crawl_ignored_potential, flag_running_progress))
            flag_running_progress -= self.crawl_ignored_potential
            self.crawl_start_potential = None

        return flag_running_progress + self.potential_leak()*100

