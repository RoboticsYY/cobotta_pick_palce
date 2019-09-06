#! /usr/bin/env python

import rospy
import IPython
import argparse
import actionlib
import numpy as np
import control_msgs.msg
from math import degrees, radians
from sensor_msgs.msg import JointState
from bcap_service.msg import variant
from tf.transformations import euler_matrix, euler_from_matrix
from bcap_service.srv import bcap, bcapRequest, bcapResponse

def parse_args():
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Default info')
  parser.add_argument('--IPython', action="store_true",
              help='If set, will embed IPython for deeper debugging')
  parser.add_argument('--release', action="store_true",
              help='If set, will release controller/robot')
  parser.add_argument('--controller', type=str, default="",
              help='Controller handle number. (default: empty string)')
  parser.add_argument('--robot', type=str, default="",
              help='Robot handle number. (default: empty string)')
  args = parser.parse_args(rospy.myargv()[1:])
  return args

class CobottaGripperAction(object):
  _feedback = control_msgs.msg.GripperCommandFeedback()
  _result = control_msgs.msg.GripperCommandResult()

  def __init__(self, namespace="/cobotta/", service_name = "/bcap_service", release=False, controller="", robot=""):
    self._node_name = namespace
    rospy.loginfo("Gripper server name: {}".format(self._node_name))

    self._rate = rospy.Rate(1)
    self._move = False
    self._bcap_handle = ""
    self._robot_handle = ""

    try:
      rospy.wait_for_service(service_name, timeout=3)
      self._bcap_srv = rospy.ServiceProxy(service_name, bcap)
      self._init = True
      rospy.loginfo("Bcap server initialized successfully.")
    except rospy.ServiceException as e:
      self._init = False
      rospy.logerr("Call service {} failed: {}".format(service_name, e))
    except rospy.exceptions.ROSException as e:
      self._init = False
      rospy.logerr("Call service {} failed: {}".format(service_name, e))

    if release:
      if robot != "":
        self.release_robot(robot)
        rospy.signal_shutdown("Robot released.")

      if controller != "":
        self.release_controller(controller)
        rospy.signal_shutdown("Bcap controller released.")

      exit(-1)

    if self._init:
      if not (self.powerup_controller()):
        rospy.signal_shutdown("Motion controller init failed.")
      # else:
      #   self.move(192.06,-51.07,10,-179.45,0.25,179.54)
    else:
      rospy.signal_shutdown("Motion controller init failed.")

  def start(self):
    """
    start the control loop
    """
    self.motion_ready()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
      self.pick(192.06,-51.07,10,-179.45,0.25,179.54, 50)
      self._rate.sleep()
      r.sleep()
    self.motion_down()
    exit(-1)

  def execute_cb(self, goal):
    """
    Gripper action server callback
    """
    pass
    # if self._init:
    #   if goal.command.position != 0:
    #     if self.open_gripper():
    #       self._result.reached_goal = True
    #       self._as.set_succeeded(self._result)
    #     else:
    #       self._result.reached_goal = False
    #       self._as.set_aborted(self._result)
    #   else:
    #     if self.close_gripper():
    #       self._result.reached_goal = True
    #       self._as.set_succeeded(self._result)
    #     else:
    #       self._result.reached_goal = False
    #       self._as.set_aborted(self._result)
    # else:
    #   rospy.logerr("Hitbot action server not initialized.")

  def powerup_controller(self):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 3

      req.vntArgs = [variant() for _ in range(4)]
      
      req.vntArgs[0].vt = 8
      req.vntArgs[0].value = "b-CAP"

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "CaoProv.DENSO.VRC" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = "localhost" 

      req.vntArgs[3].vt = 8
      req.vntArgs[3].value = ""

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to get controller handle: {}.".format(resp.HRESULT))
        return False
      else:
        self._bcap_handle = resp.vntRet.value
        rospy.loginfo("Get the controller handle: {}".format(self._bcap_handle))
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to get controller handle: {}.".format(e))
      return False

  def add_robot(self):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 7

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = self._bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "Robot0" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = ""

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to add robot: {}.".format(resp.HRESULT))
        return False
      else:
        self._robot_handle = resp.vntRet.value
        rospy.loginfo("Get the robot handle: {}".format(self._robot_handle))
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to add robot: {}.".format(e))
      return False

  def take_arm(self, robot_handle):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 64

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = robot_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "TakeArm" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = ""

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to take arm: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Take arm")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to take arm: {}.".format(e))
      return False

  def give_arm(self, robot_handle):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 64

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = robot_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "TakeArm" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = ""

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to give arm: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Give arm")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to give arm: {}.".format(e))
      return False

  def motor_on(self, robot_handle):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 64

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = robot_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "Motor" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = "1"

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to bring up motor: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Motor on")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to bring up motor: {}.".format(e))
      return False

  def motor_off(self, robot_handle):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 64

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = robot_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "Motor" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = "0"

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to bring down motor: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Motor off")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to bring down motor: {}.".format(e))
      return False

  def move_arm(self, x, y, z, alpha, beta, gamma, robot_handle):
    try:
      # Get bcap service handle
      req = bcapRequest()

      req.func_id = 72

      req.vntArgs = [variant() for _ in range(4)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = robot_handle

      req.vntArgs[1].vt = 3
      req.vntArgs[1].value = "1" 

      req.vntArgs[2].vt = 8
      req.vntArgs[2].value = "P(" + str(x) + "," + str(y) + "," + str(z) + "," \
                                  + str(alpha) + "," + str(beta) + "," + str(gamma) + ")";

      req.vntArgs[3].vt = 8
      req.vntArgs[3].value = ""

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to move arm: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Move arm")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to move arm: {}.".format(e))
      return False

  def motion_ready(self):
    self.add_robot()
    self.take_arm(self._robot_handle)
    self.motor_on(self._robot_handle)

  def motion_down(self):
    self.motor_off(self._robot_handle)
    self.give_arm(self._robot_handle)
    self.release_robot(self._robot_handle)
    self.release_controller(self._bcap_handle)
  
  def move(self, x, y, z, alpha, beta, gamma):
    self.move_arm(x, y, z, alpha, beta, gamma, self._robot_handle)

  def pick(self, x, y, z, alpha, beta, gamma, approach):
    # find pre-grasp pose
    T_grasp = euler_matrix(radians(alpha), radians(beta), radians(gamma), axes="sxyz")
    T_grasp[:3, 3] = np.array([x, y, z])
    T_trans = np.identity(4)
    T_trans[2,3] = -approach

    T_pre_grasp = np.dot(T_grasp, T_trans)
    alpha_pre, beta_pre, gamma_pre = euler_from_matrix(T_pre_grasp)
    alpha_pre, beta_pre, gamma_pre = (degrees(alpha_pre), degrees(beta_pre), degrees(gamma_pre))
    x_pre, y_pre, z_pre = T_pre_grasp[:3, 3]
    
    # move to pre-grasp
    self.move(x_pre, y_pre, z_pre, alpha_pre, beta_pre, gamma_pre)

    # move to grasp
    self.move(x, y, z, alpha, beta, gamma)

    # grasp

    # move back to pre-grasp
    self.move(x_pre, y_pre, z_pre, alpha_pre, beta_pre, gamma_pre)

  def clear_error(self):
    try:
      # clear error
      req = bcapRequest()

      req.func_id = 17

      req.vntArgs = [variant() for _ in range(2)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = self.bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "ClearError"

      self._bcap_srv(req)
      rospy.loginfo("Clear error.")
      return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to clear error: {}.".format(e))
      return False   

  def release_robot(self, robot_handle):
    try:
      # release controller
      req = bcapRequest()

      req.func_id = 84

      req.vntArgs = [variant() for _ in range(1)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = robot_handle

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to release robot: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Release robot")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to release robot: {}.".format(e))
      return False

  def release_controller(self, controller_handle):
    try:
      # release controller
      req = bcapRequest()

      req.func_id = 4

      req.vntArgs = [variant() for _ in range(1)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = controller_handle

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to release controller: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Release controller")
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to release controller: {}.".format(e))
      return False

if __name__ == "__main__":
  args = parse_args()
  rospy.init_node("cobotta_motion_controller")
  ns = rospy.get_namespace()
  rospy.loginfo("Controller node namespace: {}".format(ns))
  server = CobottaGripperAction(namespace=ns, service_name="/bcap_service", 
                                release=args.release, controller=args.controller, robot=args.robot)
  server.start()
