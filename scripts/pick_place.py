#! /usr/bin/env python

import time
import rospy
import tf2_ros
import IPython
import argparse
import actionlib
import threading
import numpy as np
import control_msgs.msg
from math import degrees, radians
from bcap_service.msg import variant
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from gpd.msg import GraspConfig, GraspConfigList
from tf.transformations import euler_matrix, euler_from_matrix, quaternion_matrix, quaternion_from_matrix
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
    self._tfBuffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._tfBuffer)
    self._br = tf2_ros.TransformBroadcaster()
    self._pub = rospy.Publisher("/cobotta/"+"joint_states", JointState, queue_size=1)
    self._grasp_candidates = []

    self._joint_thread = threading.Thread(target=self.update_joint_state, name='joint_thread')

    self._rate = rospy.Rate(1)
    self._move = False
    self._open = True
    self._bcap_handle = ""
    self._robot_handle = ""

    self._boundary = {"x":[100, 314], "y":[-55, 200], "z":[70, 108]}

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
      if (self.powerup_controller()):
        self._sub = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.grasp_cb)
      else:  
        rospy.signal_shutdown("Motion controller init failed.")
      # else:
      #   self.move(192.06,-51.07,10,-179.45,0.25,179.54)
    else:
      rospy.signal_shutdown("Motion controller init failed.")

  def start(self):
    """
    start the control loop
    """
    self._joint_thread.start()
    self.motion_ready()
    r = rospy.Rate(50)
    while not rospy.is_shutdown():

      if self._move:
        for grasp in self._grasp_candidates:
          if self.pick(grasp[0], grasp[1], grasp[2], grasp[3], grasp[4], grasp[5], 35):
            break
          else:
            self.clear_error()
            self.motion_down()
            self.motion_ready()

        self._rate.sleep()
        if self.place(30.22,238.63,30,-179.45,4.95,-86.08, 50):
          self._rate.sleep()
          self._grasp_candidates = []
          self._move = False
      # self.pick(192.06,-51.07,10,-179.45,0.25,179.54, 50)
      # self._rate.sleep()
      # self.place(30.22,238.63,20,-179.45,4.95,-86.08, 50)
      # self._rate.sleep()
      r.sleep()
    self._joint_thread.join()
    self.motion_down()
    self.release_controller(self._bcap_handle)
    exit(-1)

  def update_joint_state(self):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name += ["joint_gripper"]
    js.name += ["joint_gripper_mimic"]
    js.effort += [0.0, 0.0]
    js.velocity += [0.0, 0.0]
    if self._open:
      js.position += [0.015, -0.015]
    else:
      js.position += [0.01, -0.01]
    self._pub.publish(js)
    time.sleep(0.004)
    return True

  def grasp_cb(self, msg):
    """
    Gripper action server callback
    """
    if not self._move:
      rospy.loginfo("Grasp frame: {}".format(msg.header.frame_id))

      # get camera transform
      try:
        T_camera = self._tfBuffer.lookup_transform("base_link", msg.header.frame_id, rospy.Time())
        # rospy.loginfo("Camera pose: {}".format(T_camera.transform.translation))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("TF lookup failed.")
        return

      # get camera transform matrix
      T_camera_matrix = quaternion_matrix([T_camera.transform.rotation.x,
                                          T_camera.transform.rotation.y,
                                          T_camera.transform.rotation.z,
                                          T_camera.transform.rotation.w])
      T_camera_matrix[:3, 3] = [T_camera.transform.translation.x,
                                T_camera.transform.translation.y,
                                T_camera.transform.translation.z]

      for grasp in msg.grasps:
        T_hand_in_camera = np.identity(4)
        T_hand_in_camera[:3, 0] = [grasp.binormal.x, grasp.binormal.y, grasp.binormal.z]
        T_hand_in_camera[:3, 1] = [grasp.axis.x, grasp.axis.y, grasp.axis.z]
        T_hand_in_camera[:3, 2] = [grasp.approach.x, grasp.approach.y, grasp.approach.z]
        T_hand_in_camera[:3, 3] = [grasp.bottom.x, grasp.bottom.y, grasp.bottom.z]

        T_hand_in_base = np.dot(T_camera_matrix, T_hand_in_camera)

        tf_grasp = self.to_transform(T_hand_in_base)
        self._br.sendTransform(tf_grasp)

        self._grasp_candidates = []
        if (T_hand_in_base[0, 3]*1000 > self._boundary["x"][0] and T_hand_in_base[0, 3]*1000 < self._boundary["x"][1]
            and T_hand_in_base[1, 3]*1000 > self._boundary["y"][0] and T_hand_in_base[1, 3]*1000 < self._boundary["y"][1]
            and T_hand_in_base[2, 3]*1000 > self._boundary["z"][0] and T_hand_in_base[2, 3]*1000 < self._boundary["z"][1]):
          rospy.loginfo("Valid hand position: {}".format(T_hand_in_base[:3,3]))
          alpha, beta, gamma = euler_from_matrix(T_hand_in_base)
          self._grasp_candidates.append([T_hand_in_base[0, 3]*1000, T_hand_in_base[1, 3]*1000, T_hand_in_base[2, 3]*1000,
                                          degrees(alpha), degrees(beta), degrees(gamma)])
        
        if len(self._grasp_candidates) > 0:
          self._move = True

  def to_transform(self, matrix):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "grasp"
    t.transform.translation.x = matrix[0, 3]
    t.transform.translation.y = matrix[1, 3]
    t.transform.translation.z = matrix[2, 3]
    q = quaternion_from_matrix(matrix)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

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
  
  def move(self, x, y, z, alpha, beta, gamma):
    if self.move_arm(x, y, z, alpha, beta, gamma, self._robot_handle):
      return True
    else:
      return False

  def pick(self, x, y, z, alpha, beta, gamma, approach):

    T_grasp = euler_matrix(radians(alpha), radians(beta), radians(gamma), axes="sxyz")
    T_grasp[:3, 3] = np.array([x, y, z])

    # tune grasp pose
    T_trans = np.identity(4)
    T_trans[2,3] = approach
    T_grasp = np.dot(T_grasp, T_trans)
    alpha, beta, gamma = euler_from_matrix(T_grasp)
    alpha, beta, gamma = (degrees(alpha), degrees(beta), degrees(gamma))
    x, y, z = T_grasp[:3, 3]

    # find pre-grasp pose
    T_trans = np.identity(4)
    T_trans[2,3] = - 50

    T_pre_grasp = np.dot(T_grasp, T_trans)
    alpha_pre, beta_pre, gamma_pre = euler_from_matrix(T_pre_grasp)
    alpha_pre, beta_pre, gamma_pre = (degrees(alpha_pre), degrees(beta_pre), degrees(gamma_pre))
    x_pre, y_pre, z_pre = T_pre_grasp[:3, 3]
    
    # move to pre-grasp
    if (self.move(x_pre, y_pre, z_pre, alpha_pre, beta_pre, gamma_pre) and
      # open gripper
      self.open_gripper() and
      # move to grasp
      self.move(x, y, z, alpha, beta, gamma) and
      # close gripper
      self.close_gripper() and
      # move back to pre-grasp
      self.move(x_pre, y_pre, z_pre, alpha_pre, beta_pre, gamma_pre)):
        rospy.loginfo("Pick finished")
        return True
    else:
      rospy.logerr("Pick failed")
      self.clear_error()
      return False

  def place(self, x, y, z, alpha, beta, gamma, approach):
    # find pre-place pose
    T_grasp = euler_matrix(radians(alpha), radians(beta), radians(gamma), axes="sxyz")
    T_grasp[:3, 3] = np.array([x, y, z])
    T_trans = np.identity(4)
    T_trans[2,3] = -approach

    T_pre_grasp = np.dot(T_grasp, T_trans)
    alpha_pre, beta_pre, gamma_pre = euler_from_matrix(T_pre_grasp)
    alpha_pre, beta_pre, gamma_pre = (degrees(alpha_pre), degrees(beta_pre), degrees(gamma_pre))
    x_pre, y_pre, z_pre = T_pre_grasp[:3, 3]
    
    # move to pre-place
    if (self.move(x_pre, y_pre, z_pre, alpha_pre, beta_pre, gamma_pre) and
      # move to place
      self.move(x, y, z, alpha, beta, gamma) and
      # open gripper
      self.open_gripper() and
      # move back to pre-grasp
      self.move(x_pre, y_pre, z_pre, alpha_pre, beta_pre, gamma_pre)):
        rospy.loginfo("Place finished")
        return True
    else:
      rospy.logerr("Place failed")
      self.clear_error()
      return False

  def open_gripper(self):
    try:
      # open gripper
      req = bcapRequest()

      req.func_id = 17

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = self._bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "HandMoveA"

      req.vntArgs[2].vt = 8195
      req.vntArgs[2].value = "30, 100"

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to open gripper: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Open gripper.")
        self._rate.sleep()
        self._open = True
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to open gripper: {}.".format(e))
      return False

  def close_gripper(self):
    try:
      # close gripper
      req = bcapRequest()

      req.func_id = 17

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = self._bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "HandMoveA"

      req.vntArgs[2].vt = 8195
      req.vntArgs[2].value = "20, 100"

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to close gripper: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Close gripper.")
        self._rate.sleep()
        self._open = False
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to close gripper: {}.".format(e))
      return False

  def clear_error(self):
    try:
      # clear error
      req = bcapRequest()

      req.func_id = 17

      req.vntArgs = [variant() for _ in range(2)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = self._bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "ClearError"

      resp = self._bcap_srv(req)
      if resp.HRESULT != 0:
        rospy.logerr("Failed to clear error: {}.".format(resp.HRESULT))
        return False
      else:
        rospy.loginfo("Clear error.")
        self._rate.sleep()
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
