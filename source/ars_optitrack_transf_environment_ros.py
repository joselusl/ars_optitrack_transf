
import numpy as np
from numpy import *

import os

import yaml

import rospy
import rospkg

import tf_conversions
import tf2_ros

import visualization_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

import std_msgs.msg
from std_msgs.msg import Header


class ArsOptitrackTransfEnvironmentRos:

    obstacles_label = ''

    optitrack_transf_environment_descript_yaml_file_name = ''

    optitrack_transf_environment_descript = None
    optitrack_transf_environment_descript_label = None


    # TF
    tf2_listener = None
    tf2_buffer = None
    world_frame_name = "world"


    # Publishers
    obstacles_pub = None
    obstacles_msg = MarkerArray()


    # Timers
    pub_timer_freq = 100.
    pub_timer = None



    def __init__(self):

        return
        

    def init(self, node_name='ars_optitrack_transf_environment'):
        #
        rospy.init_node(node_name, anonymous=True)

        # Package path
        pkg_path = rospkg.RosPack().get_path('ars_optitrack_transf')
        

        #### READING PARAMETERS ###
        
        # Environment description
        default_optitrack_transf_environment_descript_yaml_file_name = os.path.join(pkg_path,'config','optitrack_transf_environment.yaml')
        optitrack_transf_environment_descript_yaml_file_name_str = rospy.get_param('~optitrack_transf_environment_description_yaml_file', default_optitrack_transf_environment_descript_yaml_file_name)
        print(optitrack_transf_environment_descript_yaml_file_name_str)
        self.optitrack_transf_environment_descript_yaml_file_name = os.path.abspath(optitrack_transf_environment_descript_yaml_file_name_str)


        # Obstacles label
        obstacles_label = rospy.get_param('~obstacles_label', 'static')


        # Load environment description
        with open(self.optitrack_transf_environment_descript_yaml_file_name,'r') as file:
            # The FullLoader parameter handles the conversion from YAML
            # scalar values to Python the dictionary format
            self.optitrack_transf_environment_descript = yaml.load(file)
        print(self.optitrack_transf_environment_descript)

        #
        self.optitrack_transf_environment_descript_label = self.optitrack_transf_environment_descript[obstacles_label]
        print(self.optitrack_transf_environment_descript_label)
       


        #
        self.tf2_buffer = tf2_ros.Buffer()
        
         
        #
        return
        

    def open(self):

        # Tf
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        # Publishers
        self.obstacles_pub = rospy.Publisher("obstacles", MarkerArray, queue_size=1, latch=True)

        # Timers
        self.pub_timer = rospy.Timer(rospy.Duration(1.0/self.pub_timer_freq), self.timer_pub_callback)

        # End
        return


    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            return False
        return True


    def timer_pub_callback(self, time_stamp):

        self.obstacles_msg.markers = []

        if self.optitrack_transf_environment_descript_label:

            for object_env in self.optitrack_transf_environment_descript_label:

                for circle in object_env['circles']:

                    obst_pos = self.read_tf_data("world", circle['child_frame_id'], rospy.Time().now())

                    if obst_pos is None:
                        continue

                    obstacle_i = Marker()

                    obstacle_i.header = Header()
                    obstacle_i.header.stamp = rospy.Time().now()
                    obstacle_i.header.frame_id = self.world_frame_name

                    obstacle_i.ns = obstacles_label
                    obstacle_i.id = circle['id']

                    obstacle_i.action = 0

                    obstacle_i.type = 3  # circle shape

                    obstacle_i.pose.position.x = obst_pos.translation.x
                    obstacle_i.pose.position.y = obst_pos.translation.y
                    obstacle_i.pose.position.z = obst_pos.translation.z

                    obstacle_i.pose.orientation.w = 1.0
                    obstacle_i.pose.orientation.x = 0.0
                    obstacle_i.pose.orientation.y = 0.0
                    obstacle_i.pose.orientation.z = 0.0

                    obstacle_i.scale.x = circle['sizes'][0]
                    obstacle_i.scale.y = circle['sizes'][1]
                    obstacle_i.scale.z = circle['sizes'][2]

                    obstacle_i.color.r = 0.0
                    obstacle_i.color.g = 1.0
                    obstacle_i.color.b = 0.0
                    obstacle_i.color.a = 0.3

                    obstacle_i.lifetime = rospy.Duration(self.pub_timer_interval)

                    self.obstacles_msg.markers.append(obstacle_i)

        self.obstacles_pub.publish(self.obstacles_msg)

        # end
        return


    def read_tf_data(self, parent, child, time):
        try:
            return self.tf2_buffer.lookup_transform(
                self.world_frame_name, child, time
            ).transform
        except tf2_ros.LookupException:
            return None

