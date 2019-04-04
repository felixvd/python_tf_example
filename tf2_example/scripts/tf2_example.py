#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import tf_conversions
import tf
# import tf2
from math import pi
# import numpy as np

import math

# LOG_LEVEL = log_level = rospy.DEBUG
LOG_LEVEL = log_level = rospy.INFO

class TF2_Example():
  """
  Contains some example usages of manual TF operations.
  """
  def __init__(self):
    rospy.init_node('tf2_example', anonymous=False)

    self.listener = tf.TransformListener()

  def transformTargetPoseFromTipLinkToEE(self, ps, robot_name, end_effector_link):
    """
    Example of a function using TF copied from another project.
    """
    rospy.logdebug("Received a pose to transform to EE link:")
    rospy.logdebug(str(ps.pose.position.x) + ", " + str(ps.pose.position.y)  + ", " + str(ps.pose.position.z))
    rospy.logdebug(str(ps.pose.orientation.x) + ", " + str(ps.pose.orientation.y)  + ", " + str(ps.pose.orientation.z)  + ", " + str(ps.pose.orientation.w))

    t = self.listener.lookupTransform(end_effector_link, robot_name + "_tool0", rospy.Time())

    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = ps.header.frame_id
    m.child_frame_id = "temp_goal_pose"
    m.transform.translation.x = ps.pose.position.x
    m.transform.translation.y = ps.pose.position.y
    m.transform.translation.z = ps.pose.position.z
    m.transform.rotation.x = ps.pose.orientation.x
    m.transform.rotation.y = ps.pose.orientation.y
    m.transform.rotation.z = ps.pose.orientation.z
    m.transform.rotation.w = ps.pose.orientation.w
    self.listener.setTransform(m)

    m.header.frame_id = "temp_goal_pose"
    m.child_frame_id = "temp_wrist_pose"
    m.transform.translation.x = t[0][0]
    m.transform.translation.y = t[0][1]
    m.transform.translation.z = t[0][2]
    m.transform.rotation.x = t[1][0]
    m.transform.rotation.y = t[1][1]
    m.transform.rotation.z = t[1][2]
    m.transform.rotation.w = t[1][3]
    self.listener.setTransform(m)

    ps_wrist = geometry_msgs.msg.PoseStamped()
    ps_wrist.header.frame_id = "temp_wrist_pose"
    ps_wrist.pose.orientation.w = 1.0

    ps_new = self.listener.transformPose(ps.header.frame_id, ps_wrist)

    rospy.logdebug("New pose:")
    rospy.logdebug(str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y)  + ", " + str(ps_new.pose.position.z))
    rospy.logdebug(str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y)  + ", " + str(ps_new.pose.orientation.z)  + ", " + str(ps_new.pose.orientation.w))

    return ps_new
  
  def set_transformations(self):
    '''This sets up a TF tree for testing. Normally, users would rarely set their own transformations. They would all be received from the robot state publishers/controllers and/or a URDF.'''

    m = geometry_msgs.msg.TransformStamped()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = "world"
    m.child_frame_id = "robot_tool0"
    m.transform.translation.z = 5.0
    m.transform.rotation.w = 1.0
    self.listener.setTransform(m)


    self.points = [ [0.0,0.0,1.0], [0.0,0.0,1.0], [0.0,0.0,1.0],
               [0.0,1.0,0.0], [0.0,1.0,0.0], [0.0,1.0,0.0],
               [1.0,0.0,0.0], [1.0,0.0,0.0], [1.0,0.0,0.0],
               [1.0,0.0,1.0], [1.0,0.0,1.0], [1.0,0.0,1.0],
               [0.0,1.0,2.0], [0.0,1.0,2.0], [0.0,1.0,2.0]]
    self.rpys = [ [pi, pi*45/180, pi/2], [pi/2, pi, pi*45/180], [pi*45/180, pi/2, pi], 
             [0, pi*555/180, pi/2], [pi/2, 0, pi*555/180], [pi*555/180, pi/2, 0],
             [pi*30/180, pi*45/180, pi/2], [pi/2, pi*30/180, pi*45/180], [pi*45/180, pi/2, pi*30/180],
             [0, pi*15/180, 0], [0, 0, pi*15/180], [pi*15/180, 0, 0],
             [pi/2, pi/2, pi/2], [pi, pi, pi], [2*pi, 2*pi, 2*pi] ]
    
    for i in range(len(self.points)):
      m.transform.translation = geometry_msgs.msg.Point(*self.points[i])
      m.transform.rotation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(*self.rpys[i]))
      m.child_frame_id = "frame" + str(i)
      self.listener.setTransform(m)
    
    m.header.frame_id = "frame5"
    for i in range(len(self.points)):
      m.transform.translation = geometry_msgs.msg.Point(*self.points[i])
      m.transform.rotation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(*self.rpys[i]))
      m.child_frame_id = "frame" + str(i + len(self.points))
      self.listener.setTransform(m)
    return

  def test_transformations(self):
    ps = geometry_msgs.msg.PoseStamped()
    ps.pose.orientation.w = 1.0

    for i in range(len(self.points) * 2):
      ps.header.frame_id = "frame"+str(i)
      ps_new = self.listener.transformPose("world", ps)
      rospy.loginfo("Pose nr. " + str(i) + ": " \
              + str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y)  + ", " + str(ps_new.pose.position.z) + "; " \
              + str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y)  + ", " + str(ps_new.pose.orientation.z)  + ", " + str(ps_new.pose.orientation.w))
    
    ps.header.frame_id = "world"
    ps.pose.position.z = 8.0
    ps_new = self.transformTargetPoseFromTipLinkToEE(ps, "robot", "frame0")
    rospy.loginfo("Pose nr. " + str(len(self.points) * 2) + ": " \
              + str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y)  + ", " + str(ps_new.pose.position.z) + "; " \
              + str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y)  + ", " + str(ps_new.pose.orientation.z)  + ", " + str(ps_new.pose.orientation.w))
    return

if __name__ == '__main__':
  try:
    c = TF2_Example()
    c.set_transformations()
    c.test_transformations()
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
