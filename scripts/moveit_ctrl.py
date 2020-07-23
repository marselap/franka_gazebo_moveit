#!/usr/bin/env python

import sys
import copy
import rospy
import time
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal
import pickle

from transforms import quaternion_matrix, quaternion_from_matrix, euler_to_quaternion, quaternion_to_euler

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class plantRecord(object):

  def __init__(self):
    super(plantRecord, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plant_record',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    self.error_recovery_pub = rospy.Publisher('/franka_control/error_recovery/goal', 
                                                   ErrorRecoveryActionGoal, 
                                                  queue_size=1)

    self.start_recording_pub = rospy.Publisher("/record", Bool, queue_size=1)

    self.touch_id_pub = rospy.Publisher("/touch_id", String, queue_size=1)
    self.touch_id = 0

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.planning_frame = planning_frame

  def execute_plan(self, plan):
    group = self.group
    return group.execute(plan, wait=True)

  def plan_to_pose(self, p, q):

    group = self.group

    wpose = group.get_current_pose().pose
    wpose.position.x = p.x
    wpose.position.y = p.y
    wpose.position.z = p.z

    wpose.orientation.x = q.x
    wpose.orientation.y = q.y
    wpose.orientation.z = q.z
    wpose.orientation.w = q.w

    (plan, fraction) = group.compute_cartesian_path(
                                       [wpose],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    time.sleep(0.1)
    if fraction > 0.:
        return self.execute_plan(plan)
    else:
        
        return False


  def loops(self, yaws, z_levels, z_step = 0.001, reps = 0):

    err_rec_msg = ErrorRecoveryActionGoal()
    msg_true = Bool()
    msg_true.data = True

    no_error = True

    roll = -180.0
    pitch = 0.0
    yaw = -135.0
    yaw_initial = -135.0

    pos_up = self.group.get_current_pose().pose.position
    pos_up.z = pos_up.z + 0.03

    q = self.group.get_current_pose().pose.orientation

    pos_down = copy.deepcopy(pos_up)
    for dyaw in yaws:
        yaw_ = yaw + dyaw
        print([yaw_, pitch, roll])
        q = euler_to_quaternion(yaw_, pitch, roll)
        print(q)

        success_u = False
        success_u = self.plan_to_pose(pos_up, q)


        print(success_u)
        while (success_u == False):
            self.error_recovery_pub.publish(err_rec_msg)
            time.sleep(0.2)
            success_u = self.plan_to_pose(pos_up, q)
            time.sleep(0.4)
            fail = False
            msg_touchid = String()
            msg_touchid.data = str(-1)
        
        print("=========")
        print(yaw_)

        for iz in xrange(z_levels):
            pos_down.x = pos_up.x
            pos_down.y = pos_up.y
            delta_z = - (z_step * iz)
            pos_down.z = pos_up.z + delta_z - 0.03
            
            # print(pos_down.z)

            rep = 0
            fail = False
            success_d = False
            success_u = False
            while (rep < reps) and (fail == False):
                print('rep:')
                print(rep)
                self.start_recording_pub.publish(msg_true)

                success_d = self.plan_to_pose(pos_down, q)

                
                time.sleep(0.3)  #kad dode dole 

                success_u = self.plan_to_pose(pos_up, q)
                time.sleep(0.1) # kad dode gore
                
                fail = not success_u or not success_d

                if (fail == True):
                    self.error_recovery_pub.publish(err_rec_msg)
                    time.sleep(0.2)
                    self.go_to_pose(pos_up, q)
                    time.sleep(0.4)
                    fail = False
                    msg_touchid = String()
                    msg_touchid.data = str(-1)
                else:
                    rep += 1
                    self.touch_id += 1
                    msg_touchid = String()
                    msg_touchid.data = str(self.touch_id)
                self.touch_id_pub.publish(msg_touchid)
                time.sleep(0.1)
                self.stop_recording_pub.publish(msg_true)
                
                # outFolder = "/home/franka/ivona_dipl/labels/"

                # yaw_spremi= yaw_-yaw
                # print('yaw_ koji se sprema') 
                # print(yaw_spremi) 
                # yaw_depth_arr = [yaw_spremi, delta_z]
                # pathOut = outFolder + "yaw_depth_" + str(self.touch_id) + ".txt"
                # with open(pathOut, 'wb') as fp:
                #     pickle.dump(yaw_depth_arr, fp)

    # except KeyboardInterrupt:
    #     return


def main():

  try:
    
    experiment = plantRecord()

    experiment.group.set_max_velocity_scaling_factor(0.5)

    roll = 180
    pitch = 0
    yaw = 0
    q = euler_to_quaternion(yaw, pitch, roll)

    pose_start = experiment.group.get_current_pose().pose

    pose_start.orientation.x = q.x
    pose_start.orientation.y = q.y
    pose_start.orientation.z = q.z
    pose_start.orientation.w = q.w
    
    pose_start.position.x = 0.3
    pose_start.position.y = 0.0
    pose_start.position.z = 0.5

    plan, fraction = experiment.group.compute_cartesian_path([pose_start],   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    print("fraction:")
    print(fraction)
    execute = raw_input("execute?")
    if (execute == 'y') or (execute == 'Y'):
        experiment.execute_plan(plan)

    time.sleep(2.)

    # while True:
    #     pose = experiment.group.get_current_pose().pose
    #     print pose 
    #     # q = pose.orientation
    #     # print quaternion_to_euler(q.x, q.y, q.z, q.w)
    #     time.sleep(1.)

    yaws = []
    # for i in range(0,180,5):
    #    yaws.append(i)
    # yaws = [0]

    # z_levels = 2 # kod mene 5mm/0.5mm = 10
    # reps = 1   #kolko istih slucajeva kuta i dubine zelim

    # z_step = 0.5e-3  #metri #za tolko se spusti dole, kod mene 0.5mm = 0.5e-3m

    # try:
    #     res = experiment.loops(yaws, z_levels, z_step, reps)
    #     time.sleep(1)

    # except ValueError as e:
    #     return
    # except rospy.ROSInterruptException:
    #     return
    # except KeyboardInterrupt:
    #     return

  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      print"whaaat"
  print("============ Python experiment demo complete!")

  return


if __name__ == '__main__':
    main()
