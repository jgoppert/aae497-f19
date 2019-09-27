#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
import nav_msgs.msg


class RocketController:


    def __init__(self):
        rospy.init_node('rocket_controller')
        self.time_last_plan = rospy.Time.now()

        # fin torques
        self.pub_fin1 = rospy.Publisher('/rocket/fin1_torque', Wrench, queue_size=10)
        self.pub_fin2 = rospy.Publisher('/rocket/fin2_torque', Wrench, queue_size=10)
        self.pub_fin3 = rospy.Publisher('/rocket/fin3_torque', Wrench, queue_size=10)
        self.pub_fin4 = rospy.Publisher('/rocket/fin4_torque', Wrench, queue_size=10)

        # where rocket is at
        self.sub_ground_truth = rospy.Subscriber('/rocket/ground_truth', Odometry,
                self.control_callback)

    def control_callback(self, odom):
        # position
        position = odom.pose.pose.position

        # orientation
        orientation = odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))
        pitch = euler[1]
        yaw = euler[2]

        # rotation rate
        rates = odom.twist.twist.angular
        vel = odom.twist.twist.linear
        yaw_rate = rates.z

        # publish fram for transform tree
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w) ,
            rospy.Time.now(),
            "base_link",
            "map")

        # fin1
        msg = Wrench()
        msg.force.x = 0
        msg.force.y = 0
        msg.force.z = 0
        msg.torque.x = 10
        msg.torque.y = 0
        msg.torque.z = 0
        self.pub_fin1.publish(msg)


if __name__ == "__main__":
    RocketController()
    rospy.spin()
