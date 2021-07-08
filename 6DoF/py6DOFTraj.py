import numpy as np
import rospy
# from scipy import optimize
x_offset = 0.0
x_scale = 1.35
y_offset = 0.0
z_offset = 0.7
xy_mag = 0.60
# z_mag = 0.5
interval = 5.0
def trajectory(t):
    x = x_offset
    y = y_offset
    if t<0:
        time_instance = 0
    else:
        time_instance = t
    # if 0<=time_instance<interval:
    #     y = -0.5*xy_mag
    #     x = x_scale*(-0.5*xy_mag+(time_instance-0)*xy_mag/interval) + x_offset
    # elif interval<=time_instance<2*interval:
    #     y = -0.5*xy_mag+(time_instance-interval)*xy_mag/interval
    #     x = x_scale*(0.5*xy_mag) + x_offset
    # elif 2*interval<=time_instance<3*interval:
    #     y = 0.5*xy_mag
    #     x = x_scale*(0.5*xy_mag-(time_instance-2*interval)*xy_mag/interval) + x_offset
    # else:
    #     y = 0.5*xy_mag-(time_instance-3*interval)*xy_mag/interval
    #     x = x_scale*(-0.5*xy_mag) + x_offset

    # x = x_offset + -xy_mag/2*np.cos(np.pi*time_instance/10)
    # y = y_offset + xy_mag/2*np.sin(np.pi*time_instance/10)
    z = z_offset
    # yaw = 0.0*((time_instance*0.8+np.pi)%(2*np.pi)-np.pi)
    # yaw = 45.0*np.pi/180.0
    # if time_instance==0:
    #     # pitch = 0
    # else:
    pitch = 0.35*np.sin(18*time_instance*np.pi/180)
    # pitch = rospy.get_param("myTraj/oscmagnitude")
    roll = 0.0
    # pitch = -5.0*np.sin(time_instance/10*np.pi)
    # if time_instance<=0:
    #     pitch = 0
    # elif time_instance > 2*interval:
    #     pitch = -10
    # else:
    #     pitch = 10
    yaw = 0.0
    # return x, y, z, pitch, yaw
    return x, y, z, roll, pitch, yaw, time_instance
