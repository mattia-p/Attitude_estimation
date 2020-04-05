#!/usr/bin/env python
# coding: utf-8

import rospy

class Complementary_filter:

    def __init__(self):
        self.pub_theta = rospy.Publisher('theta', Float32, queue_size = 10)
        self.imu_subscriber = rospy.Subscriber("/android/imu", Imu, self.callback_imu)

        # self.calibration()
        self.complementary_filter_loop()

    def callback_imu(self, msg):

    def complementary_filter_loop():
        while not rospy.is_shutdown():

            




if __name__ == '__main__':
    rospy.init_node('complementary_filter')
    Complementary_filter()
    rospy.spin()
