#!/usr/bin/env python
# coding: utf-8

"""

Try other formula for angle estimation with accelerometer. Use atan2. Maybe our result is different because we do first order approximation
"""

import rospy
import numpy as np
import statistics
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


angular_velocity = None
linear_acceleration = None

def callback_imu(data):
    global angular_velocity, linear_acceleration
    angular_velocity = data.angular_velocity
    linear_acceleration = data.linear_acceleration

def sensor_analysis_loop():
    rospy.init_node('sensor_analysis', anonymous = True)
    rospy.Subscriber("/android/imu", Imu, callback_imu)

    # While loop because the subscrition to topics takes some time
    while linear_acceleration == None and angular_velocity == None and not rospy.is_shutdown():
        print 'Waiting to subscribe to topics'
        # print linear_acceleration
        rospy.Rate(10).sleep()
        print ' ... '

    print 'Subscribed to topics'
    print ' '

    # parameters
    freq = 10
    dt = 1.0 / freq
    nb_total_points = 300
    nb_of_recorded_points = 0
    list_acc_x = []

    print 'Acquiring data ...'
    print ' '
    rate = rospy.Rate(freq)
    while (not rospy.is_shutdown()) and (nb_of_recorded_points < nb_total_points):

        nb_of_recorded_points += 1
        acc_x = linear_acceleration.x
        list_acc_x.append(acc_x)

        rate.sleep()

    num_bins = 40
    n, bins, patches = plt.hist(list_acc_x, num_bins, facecolor='blue', alpha=0.5)

    mean = statistics.mean(list_acc_x)
    sigma = statistics.stdev(list_acc_x)
    y = mlab.normpdf(bins, mean, sigma)
    print 'mean = ', mean
    print 'Standard deviation = ', sigma

    plt.plot(bins, y, 'r--')
    plt.title('Histogram of linear acceleration in X direction')
    plt.xlabel('Linear acceleration in x direction')
    plt.ylabel('Number of appearances')
    plt.show()


if __name__ == '__main__':

    sensor_analysis_loop()
