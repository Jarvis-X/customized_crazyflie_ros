#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt32
from py6DOFTraj import trajectory
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    n = rospy.get_param("~Robot", 2)

    r = 50.
    t = 0.

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame

    pub = rospy.Publisher('goal', PoseStamped, queue_size=1)
    # sub_m0 = rospy.Subscriber('motorPower/values/m0', UInt32)
    # sub_m1 = rospy.Subscriber('motorPower/values/m1', UInt32)
    # sub_m2 = rospy.Subscriber('motorPower/values/m2', UInt32)
    # sub_m3 = rospy.Subscriber('motorPower/values/m3', UInt32)

    ttime = 5
    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    srv_update_params = rospy.ServiceProxy('update_params', UpdateParams)
    rospy.set_param("ctrlMel/trajectory_type", 7)
    srv_update_params(["ctrlMel/trajectory_type"])

    t_begin = rospy.Time.now().to_sec()
    old_pitch = 0
    old_time_instance = 0
    rospy.set_param("ctrlMel/time_instance", 0)
    srv_update_params(["ctrlMel/time_instance"])
    while not rospy.is_shutdown(): 
        # t_now = rospy.get_param("ctrlMel/time_instance")
        t_now = rospy.Time.now().to_sec()
        [x, y, z, roll, pitch, yaw, time_instance] = trajectory(0.2*(t_now-t_begin-15))
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            # rospy.set_param("ctrlMel/desiredPitch", pitch)
            # srv_update_params(["ctrlMel/desiredPitch"])
        #     old_pitch = pitch
        # else:
        #     quaternion = tf.transformations.quaternion_from_euler(0, np.pi*old_pitch/180, yaw)
        if time_instance - old_time_instance>0.1:
            # rospy.set_param("ctrlMel/time_instance", time_instance)
            # srv_update_params(["ctrlMel/time_instance"])
            old_time_instance = time_instance
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        #print(t, dst_xAd)
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
