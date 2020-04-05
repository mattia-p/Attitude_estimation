#!/usr/bin/env python
# coding: utf-8

"""

Try other formula for angle estimation with accelerometer. Use atan2. Maybe our result is different because we do first order approximation
"""
import tf
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage


angular_velocity = None
linear_acceleration = None


def calibration():

    print 'Starting calibration - Put sensor on a flat surface'
    print ' ... '

    nb_iterations = 200

    acc_x, acc_y, acc_z, ang_vel_x, ang_vel_y, ang_vel_z = 0, 0, 0, 0, 0, 0
    number_of_points = 0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and (number_of_points < nb_iterations):

        number_of_points += 1
        acc_x += linear_acceleration.x
        acc_y += linear_acceleration.y
        acc_z += linear_acceleration.z
        # print 'acc_z = ', acc_z

        ang_vel_x += angular_velocity.x
        ang_vel_y += angular_velocity.y
        ang_vel_z += angular_velocity.z

        rate.sleep()


    number_of_points = float(number_of_points)
    acc_x_bias = acc_x / number_of_points
    acc_y_bias = acc_y / number_of_points
    acc_z_bias = acc_z / number_of_points

    ang_vel_x_bias = ang_vel_x / number_of_points
    ang_vel_y_bias = ang_vel_y / number_of_points
    ang_vel_z_bias = ang_vel_z / number_of_points

    print ' '
    print 'Calibration finished'
    return acc_x_bias, acc_y_bias, acc_z_bias, ang_vel_x_bias, ang_vel_y_bias, ang_vel_z_bias


def callback_imu(data):
    global angular_velocity, linear_acceleration
    angular_velocity = data.angular_velocity
    linear_acceleration = data.linear_acceleration

def attitude_estimator_loop():
    rospy.init_node('attitude_estimator', anonymous = True)
    rospy.Subscriber("/android/imu", Imu, callback_imu)
    pub_theta = rospy.Publisher('theta', Float32, queue_size = 10)
    pub_phi   = rospy.Publisher('phi', Float32, queue_size = 10)
    pub_psi   = rospy.Publisher('psi', Float32, queue_size = 10)

    pub_theta_est = rospy.Publisher('theta_est', Float32, queue_size = 10)
    pub_theta_meas = rospy.Publisher('theta_meas', Float32, queue_size = 10)



    # While loop because the subscrition to topics takes some time
    print 'Waiting to subscribe to topics'
    print ' ... '
    while linear_acceleration == None and angular_velocity == None and not rospy.is_shutdown():
        rospy.Rate(10).sleep()


    print 'Subscribed to topics'
    print ' '

    # Calibration
    acc_x_bias, acc_y_bias, acc_z_bias, ang_vel_x_bias, ang_vel_y_bias, ang_vel_z_bias = calibration()

    print ' '
    # print 'acc_x_bias = ', acc_x_bias
    # print 'acc_y_bias = ', acc_y_bias
    # print 'acc_z_bias = ', acc_z_bias
    #
    #
    # print 'gyr_x_bias = ', ang_vel_x_bias
    # print 'gyr_y_bias = ', ang_vel_y_bias
    # print 'gyr_z_bias = ', ang_vel_z_bias
    # print ' '
    print ' ------------------------- '
    print 'Attitude estimator started'
    print ' ------------------------- '

    # parameters
    freq = 10
    rho_phi   = 0.05
    rho_theta = 0.05
    g = 9.81
    dt = 1.0 / freq
    phi_est_prev = 0
    theta_est_prev = 0
    psi_est_prev = 0

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        phi_meas = (linear_acceleration.y - acc_y_bias) / g
        phi_est  = phi_est_prev + (angular_velocity.x - ang_vel_x_bias) * dt
        phi = (1 - rho_phi) * phi_est + rho_phi * phi_meas
        phi_est_prev = phi_est

        theta_meas = - (linear_acceleration.x - acc_x_bias) / g
        theta_est = theta_est_prev + (angular_velocity.y - ang_vel_y_bias)* dt
        theta = (1 - rho_theta) * theta_est + rho_theta * theta_meas
        theta_est_prev = theta_est

        psi_est = psi_est_prev + (angular_velocity.z - ang_vel_z_bias) * dt
        psi = psi_est
        psi_est_prev = psi_est

        # theta_meas_msg = Float32()
        # theta_meas_msg.data = theta_meas
        # pub_theta_meas.publish(theta_meas_msg)
        #
        # theta_est_msg = Float32()
        # theta_est_msg.data = theta_est
        # pub_theta_est.publish(theta_est_msg)

        theta_msg = Float32()
        theta_msg.data = theta
        pub_theta.publish(theta_msg)

        phi_msg = Float32()
        phi_msg.data = phi
        pub_phi.publish(phi_msg)

        psi_msg = Float32()
        psi_msg.data = psi
        pub_psi.publish(psi_msg)

        roll = theta
        pitch = phi
        yaw = psi

        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(pitch, roll, yaw),
                     rospy.Time.now(),
                     "phone",
                     "world")

        rate.sleep()


if __name__ == '__main__':

    attitude_estimator_loop()
