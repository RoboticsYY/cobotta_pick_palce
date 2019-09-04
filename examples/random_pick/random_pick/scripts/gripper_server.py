#! /usr/bin/env python

import rospy
import IPython
import argparse
import actionlib
import control_msgs.msg
from sensor_msgs.msg import JointState
from bcap_service.msg import variant
from bcap_service.srv import bcap, bcapRequest, bcapResponse

def parse_args():
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Default info')
  parser.add_argument('--IPython', action="store_true",
              help='If set, will embed IPython for deeper debugging')
  parser.add_argument('--release', action="store_true",
              help='If set, will release controller')
  parser.add_argument('--controller', type=str, default="",
              help='Controller handle number. (default: empty string)')
  args = parser.parse_args(rospy.myargv()[1:])
  return args

class CobottaGripperAction(object):
  _feedback = control_msgs.msg.GripperCommandFeedback()
  _result = control_msgs.msg.GripperCommandResult()

  def __init__(self, namespace="/cobotta/", action_name = "gripper_action", service_name = "/bcap_service", release=False, controller=""):
    self._action_name = namespace + action_name
    rospy.loginfo("Gripper server name: {}".format(self._action_name))
    self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, 
                                                               execute_cb=self.execute_cb, 
                                                               auto_start=False)
    self._rate = rospy.Rate(1)
    self._open = True
    self._gripper_bcap_handle = ""
    self._pub = rospy.Publisher(namespace+"joint_states", JointState, queue_size=1)

    try:
      rospy.wait_for_service(service_name, timeout=3)
      self._bcap_srv = rospy.ServiceProxy(service_name, bcap)
      self._as.start()
      self._init = True
      rospy.loginfo("Bcap server initialized successfully.")
    except rospy.ServiceException as e:
      self._init = False
      rospy.logerr("Call service {} failed: {}".format(service_name, e))
    except rospy.exceptions.ROSException as e:
      self._init = False
      rospy.logerr("Call service {} failed: {}".format(service_name, e))

    if release:
      self.release_controller(controller)
      rospy.signal_shutdown("Gripper init failed.")
      exit(-1)

    if self._init:
      if not (self.powerup_gripper() and self.close_gripper() and self.open_gripper()):
      # if not self.powerup_gripper():
        rospy.signal_shutdown("Gripper init failed.")
    else:
      rospy.signal_shutdown("Gripper init failed.")

  def start(self):
    """
    start the control loop
    """
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
      js = JointState()
      js.header.stamp = rospy.Time.now()
      js.name += ["joint_gripper"]
      js.name += ["joint_gripper_mimic"]
      js.effort += [0.0, 0.0]
      js.velocity += [0.0, 0.0]
      if self._open:
        js.position += [0.015, -0.015]
      else:
        js.position += [0.0, 0.0]
      self._pub.publish(js)
      r.sleep()
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

  def powerup_gripper(self):
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
        rospy.logerr("Failed to get gripper handle: {}.".format(resp.HRESULT))
        return False
      else:
        self._gripper_bcap_handle = resp.vntRet.value
        rospy.loginfo("Get the gripper handle: {}".format(self._gripper_bcap_handle))
        self._rate.sleep()
        return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to set gripper voltage: {}.".format(e))
      return False

  def open_gripper(self):
    try:
      # open gripper
      req = bcapRequest()

      req.func_id = 17

      req.vntArgs = [variant() for _ in range(3)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = self._gripper_bcap_handle

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
      req.vntArgs[0].value = self._gripper_bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "HandMoveA"

      req.vntArgs[2].vt = 8195
      req.vntArgs[2].value = "0, 100"

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
      req.vntArgs[0].value = self._gripper_bcap_handle

      req.vntArgs[1].vt = 8
      req.vntArgs[1].value = "ClearError"

      self._bcap_srv(req)
      rospy.loginfo("Clear error.")
      self._rate.sleep()
      return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to clear error: {}.".format(e))
      return False  

  def release_controller(self, controller_handle):
    try:
      # release controller
      req = bcapRequest()

      req.func_id = 4

      req.vntArgs = [variant() for _ in range(1)]
      
      req.vntArgs[0].vt = 3
      req.vntArgs[0].value = controller_handle

      self._bcap_srv(req)
      rospy.loginfo("release controller.")
      self._rate.sleep()
      return True
    except rospy.ServiceException as e:
      rospy.logerr("Failed to release controller: {}.".format(e))
      return False

if __name__ == "__main__":
  args = parse_args()
  rospy.init_node("cobotta_gripper")
  ns = rospy.get_namespace()
  rospy.loginfo("Gripper node namespace: {}".format(ns))
  server = CobottaGripperAction(namespace=ns, action_name="gripper_action", service_name="/bcap_service", 
                                release=args.release, controller=args.controller)
  server.start()
