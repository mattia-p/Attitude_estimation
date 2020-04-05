#!/usr/bin/env python
# coding: utf-8

import tf
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage


class Complementary_filter:

    def __init__(self):

        self.pub_theta = rospy.Publisher('theta', Float32, queue_size = 10)
        self.pub_psi   = rospy.Publisher('psi', Float32, queue_size = 10)
        self.pub_phi   = rospy.Publisher('phi', Float32, queue_size = 10)

        self.imu_subscriber = rospy.Subscriber("/android/imu", Imu, self.callback_imu)

        self.frequency = 10.0
        self.rate = rospy.Rate(self.frequency)
        self.angular_velocity = None
        self.linear_acceleration = None
        self.nb_iterations_calibration = 200
        self.acc_x_bias = 0.0
        self.acc_y_bias = 0.0
        self.acc_z_bias = 0.0
        self.ang_vel_x_bias = 0.0
        self.ang_vel_y_bias = 0.0
        self.ang_vel_z_bias = 0.0

        # self.calibration()

        # self.calibration()
        self.complementary_filter_loop()

    def callback_imu(self, msg):
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration
        # print self.angular_velocity


    # def calibration(self):


    def complementary_filter_loop(self):


        # While loop because the subscrition to topics takes some time
        print 'Waiting to subscribe to topics'
        while self.linear_acceleration == None and self.angular_velocity == None and not rospy.is_shutdown():
            self.rate.sleep()
        print 'Subscribed to topics'
        print ' '

        self.calibration()

        print ' ------------------------- '
        print 'Attitude estimator started'
        print ' ------------------------- '

        while not rospy.is_shutdown():
            print 'Inside while'
            self.rate.sleep()

    def calibration(self):
        print 'Starting calibration - Put sensor on a flat surface'
        print ' ... '

        acc_x, acc_y, acc_z, ang_vel_x, ang_vel_y, ang_vel_z = 0, 0, 0, 0, 0, 0
        number_of_points = 0

        while not rospy.is_shutdown() and (number_of_points < self.nb_iterations_calibration):
            number_of_points += 1
            acc_x += self.linear_acceleration.x
            acc_y += self.linear_acceleration.y
            acc_z += self.linear_acceleration.z
            # print 'acc_z = ', acc_z

            ang_vel_x += self.angular_velocity.x
            ang_vel_y += self.angular_velocity.y
            ang_vel_z += self.angular_velocity.z

            self.rate.sleep()

        number_of_points = float(number_of_points)
        self.acc_x_bias = acc_x / number_of_points
        self.acc_y_bias = acc_y / number_of_points
        self.acc_z_bias = acc_z / number_of_points

        self.ang_vel_x_bias = ang_vel_x / number_of_points
        self.ang_vel_y_bias = ang_vel_y / number_of_points
        self.ang_vel_z_bias = ang_vel_z / number_of_points

        print ' '
        print 'Calibration finished'

if __name__ == '__main__':
    rospy.init_node('complementary_filter')
    Complementary_filter()
    rospy.spin()
