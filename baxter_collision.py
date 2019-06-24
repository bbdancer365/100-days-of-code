#!/usr/bin/env python
from pprint import pprint as pp
import random
import struct
import sys
import copy
import numpy as np
import rospy
import genpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from baxter_interface import CHECK_VERSION
import os
import rospkg
from trajectory import *

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import (
    JointCommand,
)
from gazebo_msgs.msg import ContactsState

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from std_srvs.srv import Empty as placeholder
import tf_conversions
from robot_controller import *
from scene_controller import *
from moveit_msgs.msg import RobotState, RobotTrajectory
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from trajectory_msgs.msg import JointTrajectoryPoint

class Manager(object):
  def __init__(self, moveit=True):
    rospy.init_node("baxter_platform_manager")
    rospy.wait_for_message("/robot/sim/started", Empty)
    # self.robot_controller = RobotController(moveit=moveit, tolerance=0.1)
    self.robot_controller = RobotController(moveit=moveit)
    self.scene_controller = SceneController()

  def shutdown(self):
    self.robot_controller.shutdown()
    self.scene_controller.deleteAllModels()

  def _check_trajectory_validity(self, robot_trajectory, groups=['left_arm']):
        for traj_point in robot_trajectory.joint_trajectory.points:
            rs = RobotState()
            rs.joint_state.name = robot_trajectory.joint_trajectory.joint_names
            rs.joint_state.position = traj_point.positions
            for group in groups:
                gsvr = GetStateValidityRequest()
                gsvr.robot_state = rs
                gsvr.group_name = group
                result = self.robot_controller.collision_proxy.call(gsvr)
                for contact in result.contacts:
                    if 'table' in contact.contact_body_1 or \
                        'testObject2' in contact.contact_body_1 or \
                        'table' in contact.contact_body_2 or \
                        'testObject2' in contact.contact_body_2:

                        return False

        return True

  def _search_trajectory(self, robot_trajectory, groups=['left_arm']):
    new_trajectory = []
    if _check_trajectory_validity(self, robot_trajectory, groups):
      return robot_trajectory

    else:
      t_length = len(robot_trajectory)
      new_end = t_length/2
      while True:
        new_trajectory = robot_trajectory[0: new_end]
        if _check_trajectory_validity(self, new_trajectory, groups):
          new_end = new_end + ((t_length - new_end)/2)
          if new_end == t_length:
            print("Didn't find an invalid trajectory")
        else: 
          return new_trajectory
  

'''def interpolate(e, s, num):
    step = (np.array[e]-np.array[s])/num
    rtn = list()
    for i in xrange(num):
        rtn.append(s+(i*step))
    return rtn

  def gen_test_data(start_d, end_d, num):
    keys = start_d.keys()
    d = dict()
    for k in keys:
        d[k] = interpolate(start_d[k], end_d[k], num)
        
    rtn = list()
    # now that's
    # d["w_0"] = [0.1, 0.11, 0.12, ...]
    for i in xrange(num):
        # make a new dict for each column
        pt = dict()
        for k in d.keys():
            # make an entry for each key for column i
            pt[k] = d[k][i]
            # pt should look like { "w_0":0.1, "w_1": 0.79, ... }
        rtn.append(pt)

    return rtn'''

def main():

  manager = Manager(moveit=True)
  rospy.on_shutdown(manager.shutdown)

  dict1 = {
  'left_w0': 0.740299201640411, 
  'left_w1': 0.9680624138988025, 
  'left_w2': -0.7136691281471845, 
  'left_e0': -0.7285205258543106, 
  'left_e1': 1.5080107556603004, 
  'left_s0': -0.4963593155154979, 
  'left_s1': -0.622152160192126}

  dict2 = {
  'left_w0': 0.740299201640411,
  'left_w1': 0.98,
  'left_w2': -0.21366912814718450,
  'left_e0': -0.852052585431060, 
  'left_e1': 1.50801075566030040, 
  'left_s0': -0.996359315515497900,
  'left_s1': -0.7}

  starting_angles = {
  'left_w0': 0.6699952259595108,
  'left_w1': 1.030009435085784,
  'left_w2': -0.4999997247485215,
  'left_e0': -1.189968899785275,
  'left_e1': 1.9400238130755056,
  'left_s0': -0.08000397926829805,
  'left_s1': -0.9999781166910306}

  '''begin_angles = starting_angles.values()
  end_angles = dict2.values()
  new_points = interpolate(begin_angles, end_angles, 10)
  print(new_points)'''





  manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.8, y=0., z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1, ambient_a=0.1, mu1=1, mu2=1, reference_frame='')
  manager.scene_controller.makeModel(name='testObject', shape='box', size_x=0.1, size_y=0.1, size_z=0.1, x=0.8, y=0.3, z=0.75, mass=20000, mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.0, pitch=0.0, yaw=0.0, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1, diffuse_b=0, diffuse_a=1)
  manager.scene_controller.makeModel(name='testObject2', shape='box3', size_x=0.1, size_y=0.1, size_z=0.1, x=0.8, y=0.3, z=0.85, mass=20000, mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.0, pitch=0.0, yaw=0.0, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1, diffuse_b=0, diffuse_a=1)
  manager.scene_controller.makeModel(name='testObject3', shape='box4', size_x=0.1, size_y=0.1, size_z=0.1, x=0.8, y=0.3, z=0.95, mass=20000, mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.0, pitch=0.0, yaw=0.0, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1, diffuse_b=0, diffuse_a=1)
  manager.scene_controller.spawnAllModels()
 
  manager.robot_controller.moveToStart()
  #put all of the positions in one list 

  rt = RobotTrajectory()

  rt.joint_trajectory.joint_names = manager.robot_controller.getJointNames()


  jtp1 = JointTrajectoryPoint()

  #jtp1.positions = state_to_list(node1)
  jtp1.positions = dict1.values()
  jtp2 = JointTrajectoryPoint()

  #jtp2.positions = state_to_list(node2)
  #rt.joint_trajectory.points.append(jtp1)
  #rt.joint_trajectory.points.append(jtp2)

  valid = manager._check_trajectory_validity(rt)
  manager.robot_controller.followMoveItTrajectoryWithJointAngles([dict1, dict2])
  print("Sleeping valid= ", valid)
  rospy.sleep(5.)


if __name__ == '__main__':
  sys.exit(main())