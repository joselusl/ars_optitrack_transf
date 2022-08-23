#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped



import tf2_ros



#
from ars_optitrack_transf_robot import *


#
import ars_lib_helpers





class ArsOptitrackTransfRobotRos:

  #######


  # Meas robot frame
  meas_robot_frame = None
  # Meas World frame
  meas_world_frame = None


  # Estim Robot frame
  estim_robot_frame = None
  # Estim World frame
  estim_world_frame = None


  # State Estim loop freq 
  # time step
  state_estim_loop_freq = None
  # Timer
  state_estim_loop_timer = None


  # Meas
  # tf2 listener
  tf2_buffer = None
  tf2_listener = None


  # Estim Robot pose pub
  estim_robot_pose_pub = None
  estim_robot_pose_cov_pub = None
  # Estim Robot velocity pub
  estim_robot_vel_robot_pub = None
  estim_robot_vel_robot_cov_pub = None
  #
  estim_robot_vel_world_pub = None
  estim_robot_vel_world_cov_pub = None


  # tf2 broadcaster
  tf2_broadcaster = None


  # Robot state estimator
  robot_state_estimator = None
  


  #########

  def __init__(self):

    # Meas robot frame
    self.meas_robot_frame = 'meas_robot_base_link'
    # Meas World frame
    self.meas_world_frame = 'meas_world'

    # Estim robot frame
    self.estim_robot_frame = 'robot_base_link'
    # Estim World frame
    self.estim_world_frame = 'world'

    # State Estim loop freq
    self.state_estim_loop_freq = 50.0

    # Motion controller
    self.robot_state_estimator = ArsOptitrackTransfRobot()


    # end
    return


  def init(self, node_name='ars_optitrack_transf_robot_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_optitrack_transf')
    

    #### READING PARAMETERS ###
    
    # meas robot frame
    self.meas_robot_frame = rospy.get_param('~meas_robot_frame', 'optitrack_robot_base_link')
    # meas world frame
    self.meas_world_frame = rospy.get_param('~meas_world_frame', 'optitrack_world')

    ###

    
    # End
    return


  def open(self):



    # Publishers

    # 
    self.estim_robot_pose_pub = rospy.Publisher('estim_robot_pose', PoseStamped, queue_size=1)
    # 
    self.estim_robot_pose_cov_pub = rospy.Publisher('estim_robot_pose_cov', PoseWithCovarianceStamped, queue_size=1)
    #
    self.estim_robot_vel_robot_pub = rospy.Publisher('estim_robot_velocity_robot', TwistStamped, queue_size=1)
    #
    self.estim_robot_vel_robot_cov_pub = rospy.Publisher('estim_robot_velocity_robot_cov', TwistWithCovarianceStamped, queue_size=1)
    #
    self.estim_robot_vel_world_pub = rospy.Publisher('estim_robot_velocity_world', TwistStamped, queue_size=1)
    #
    self.estim_robot_vel_world_cov_pub = rospy.Publisher('estim_robot_velocity_world_cov', TwistWithCovarianceStamped, queue_size=1)


    # tf2 listener
    self.tf2_buffer = tf2_ros.Buffer()
    self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)


    # Tf2 broadcasters
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster()


    # Timers
    #
    self.state_estim_loop_timer = rospy.Timer(rospy.Duration(1.0/self.state_estim_loop_freq), self.stateEstimLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def measRobotPose(self, time_stamp_current=rospy.Time()):

    # Get tf
    try:
      robot_pose_tf = self.tf2_buffer.lookup_transform(self.meas_world_frame, self.meas_robot_frame, time_stamp_current, rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print("Error receiving TF")
      return

    # Transform tf to PoseStamped
    robot_position_msg = robot_pose_tf.transform.translation
    robot_attitude_msg = robot_pose_tf.transform.rotation

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_position_msg.x
    robot_posi[1] = robot_position_msg.y
    robot_posi[2] = robot_position_msg.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_attitude_msg.w
    robot_atti_quat[1] = robot_attitude_msg.x
    robot_atti_quat[2] = robot_attitude_msg.y
    robot_atti_quat[3] = robot_attitude_msg.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)


    # Set meas: Position
    self.robot_state_estimator.setMeasRobotPosition(time_stamp_current, robot_posi)

    # Set meas: Attitude quat simp
    self.robot_state_estimator.setMeasRobotAttitude(time_stamp_current, robot_atti_quat_simp)


    # End
    return


  def estimRobotPosePublish(self):

    #
    header_msg = Header()
    header_msg.stamp = self.robot_state_estimator.estim_state_timestamp
    header_msg.frame_id = self.estim_world_frame

    #
    robot_pose_msg = Pose()
    #
    robot_pose_msg.position.x = self.robot_state_estimator.estim_robot_posi[0]
    robot_pose_msg.position.y = self.robot_state_estimator.estim_robot_posi[1]
    robot_pose_msg.position.z = self.robot_state_estimator.estim_robot_posi[2]
    #
    robot_pose_msg.orientation.w = self.robot_state_estimator.estim_robot_atti_quat_simp[0]
    robot_pose_msg.orientation.x = 0.0
    robot_pose_msg.orientation.y = 0.0
    robot_pose_msg.orientation.z = self.robot_state_estimator.estim_robot_atti_quat_simp[1]

    #
    # Covariance
    covariance_pose = np.zeros((6,6), dtype=float)
    # Position - Position
    covariance_pose[0:3, 0:3] = self.robot_state_estimator.estim_state_cov[0:3, 0:3]
    # Position - Attitude
    covariance_pose[0:3, 5] = self.robot_state_estimator.estim_state_cov[0:3, 3]
    # Attitude - Attitude
    covariance_pose[5, 5] = self.robot_state_estimator.estim_state_cov[3, 3]
    # Attitude - Position
    covariance_pose[5, 0:3] = self.robot_state_estimator.estim_state_cov[3, 0:3]

    #
    robot_pose_stamped_msg = PoseStamped()
    #
    robot_pose_stamped_msg.header = header_msg
    robot_pose_stamped_msg.pose = robot_pose_msg

    #
    robot_pose_cov_stamped_msg = PoseWithCovarianceStamped()
    #
    robot_pose_cov_stamped_msg.header = header_msg
    robot_pose_cov_stamped_msg.pose.pose = robot_pose_msg
    robot_pose_cov_stamped_msg.pose.covariance = covariance_pose.reshape((36,))
  
    #
    self.estim_robot_pose_pub.publish(robot_pose_stamped_msg)
    # 
    self.estim_robot_pose_cov_pub.publish(robot_pose_cov_stamped_msg)


    # Tf2
    estim_robot_pose_tf2_msg = geometry_msgs.msg.TransformStamped()

    estim_robot_pose_tf2_msg.header.stamp = self.robot_state_estimator.estim_state_timestamp
    estim_robot_pose_tf2_msg.header.frame_id = self.estim_world_frame
    estim_robot_pose_tf2_msg.child_frame_id = self.estim_robot_frame

    estim_robot_pose_tf2_msg.transform.translation.x = self.robot_state_estimator.estim_robot_posi[0]
    estim_robot_pose_tf2_msg.transform.translation.y = self.robot_state_estimator.estim_robot_posi[1]
    estim_robot_pose_tf2_msg.transform.translation.z = self.robot_state_estimator.estim_robot_posi[2]

    estim_robot_pose_tf2_msg.transform.rotation.w = self.robot_state_estimator.estim_robot_atti_quat_simp[0]
    estim_robot_pose_tf2_msg.transform.rotation.x = 0.0
    estim_robot_pose_tf2_msg.transform.rotation.y = 0.0
    estim_robot_pose_tf2_msg.transform.rotation.z = self.robot_state_estimator.estim_robot_atti_quat_simp[1]

    # Broadcast
    self.tf2_broadcaster.sendTransform(estim_robot_pose_tf2_msg)


    # End
    return


  def estimRobotVelocityPublish(self):

    #
    # Robot Velocity Wrt world

    # Header
    header_wrt_world_msg = Header()
    header_wrt_world_msg.stamp = self.robot_state_estimator.estim_state_timestamp
    header_wrt_world_msg.frame_id = self.estim_world_frame

    # Twist
    robot_velocity_world_msg = Twist()
    #
    robot_velocity_world_msg.linear.x = self.robot_state_estimator.estim_robot_velo_lin_world[0]
    robot_velocity_world_msg.linear.y = self.robot_state_estimator.estim_robot_velo_lin_world[1]
    robot_velocity_world_msg.linear.z = self.robot_state_estimator.estim_robot_velo_lin_world[2]
    #
    robot_velocity_world_msg.angular.x = 0.0
    robot_velocity_world_msg.angular.y = 0.0
    robot_velocity_world_msg.angular.z = self.robot_state_estimator.estim_robot_velo_ang_world[0]
    
    # TwistStamped
    robot_velocity_world_stamp_msg = TwistStamped()
    robot_velocity_world_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_stamp_msg.twist = robot_velocity_world_msg

    # TwistWithCovarianceStamped
    # TODO JL Cov
    robot_velocity_world_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_world_cov_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_cov_stamp_msg.twist.twist = robot_velocity_world_msg
    # robot_velocity_world_cov_stamp_msg.twist.covariance



    #
    # Robot velocity wrt robot

    # computation estim robot velocity robot
    estim_robot_vel_lin_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.robot_state_estimator.estim_robot_velo_lin_world, self.robot_state_estimator.estim_robot_atti_quat_simp, flag_quat_simp=True)
    estim_robot_vel_ang_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.robot_state_estimator.estim_robot_velo_ang_world, self.robot_state_estimator.estim_robot_atti_quat_simp, flag_quat_simp=True)

    # Header
    header_wrt_robot_msg = Header()
    header_wrt_robot_msg.stamp = self.robot_state_estimator.estim_state_timestamp
    header_wrt_robot_msg.frame_id = self.estim_robot_frame

    # Twist
    robot_velocity_robot_msg = Twist()
    #
    robot_velocity_robot_msg.linear.x = estim_robot_vel_lin_robot[0]
    robot_velocity_robot_msg.linear.y = estim_robot_vel_lin_robot[1]
    robot_velocity_robot_msg.linear.z = estim_robot_vel_lin_robot[2]
    #
    robot_velocity_robot_msg.angular.x = 0.0
    robot_velocity_robot_msg.angular.y = 0.0
    robot_velocity_robot_msg.angular.z = estim_robot_vel_ang_robot[0]

    # TwistStamped
    robot_velocity_robot_stamp_msg = TwistStamped()
    robot_velocity_robot_stamp_msg.header = header_wrt_robot_msg
    robot_velocity_robot_stamp_msg.twist = robot_velocity_robot_msg

    # TwistWithCovarianceStamped
    # TODO JL Cov
    robot_velocity_robot_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_robot_cov_stamp_msg.header = header_wrt_robot_msg
    robot_velocity_robot_cov_stamp_msg.twist.twist = robot_velocity_robot_msg
    # robot_velocity_robot_cov_stamp_msg.twist.covariance


    # Publish
    #
    self.estim_robot_vel_world_pub.publish(robot_velocity_world_stamp_msg)
    # 
    self.estim_robot_vel_world_cov_pub.publish(robot_velocity_world_cov_stamp_msg)

    #
    self.estim_robot_vel_robot_pub.publish(robot_velocity_robot_stamp_msg)
    # 
    self.estim_robot_vel_robot_cov_pub.publish(robot_velocity_robot_cov_stamp_msg)

    # End
    return
  

  def stateEstimLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    # Read pose measurement
    self.measRobotPose(time_stamp_current)


    # Predict
    self.robot_state_estimator.predict(time_stamp_current)

    # Update
    self.robot_state_estimator.update()


    # Publish
    #
    self.estimRobotPosePublish()
    #
    self.estimRobotVelocityPublish()

     
    # End
    return

  