#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class Controller():
    def __init__(self, use_controller, joy_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.set_param("commander/enHighLevel", 0)
        rospy.set_param("stabilizer/estimator", 2)  # Use EKF=2
        rospy.set_param("stabilizer/controller", 2)  # Use mellinger controller=2, PID=1
        rospy.set_param("kalman/resetEstimation", 1)  # reset kalman

        # # Mellinger control parameters
        rospy.set_param("myTraj/height", 0.7)
        rospy.set_param("ctrlMel/mass", 0.640)
        rospy.set_param("ctrlMel/massThrust", 62000)
        rospy.set_param("ctrlMel/i_range_z", 5.0)
        rospy.set_param("ctrlMel/i_range_x", 4.0)
        rospy.set_param("ctrlMel/i_range_y", 4.0)
        # rospy.set_param("ctrlMel/i_range_m_z", 1500.0)
        # rospy.set_param("ctrlMel/i_range_m_x", 1500.0)
        # rospy.set_param("ctrlMel/i_range_m_y", 1000.0)

        rospy.set_param("ctrlMel/i_range_m_z", 40.0)
        rospy.set_param("ctrlMel/i_range_m_x", 40.0)
        rospy.set_param("ctrlMel/i_range_m_y", 40.0)
        rospy.set_param("ctrlMel/kR_y", 28000.0)
        rospy.set_param("ctrlMel/kw_y", 8000.0)
        rospy.set_param("ctrlMel/kR_x", 28000.0)
        rospy.set_param("ctrlMel/kw_x", 8000.0)
        rospy.set_param("ctrlMel/kR_z", -22500.0)
        rospy.set_param("ctrlMel/kw_z", -8000.0)

        rospy.set_param("ctrlMel/kd_omega_rp", 100.0)

        rospy.set_param("ctrlMel/kp_x", 1.4)
        rospy.set_param("ctrlMel/kd_x", 0.9)
        rospy.set_param("ctrlMel/ki_x", 0.65)
        rospy.set_param("ctrlMel/kp_y", 1.4)
        rospy.set_param("ctrlMel/kd_y", 0.9)
        rospy.set_param("ctrlMel/ki_y", 0.65)

        rospy.set_param("ctrlMel/kp_z", 3.0)
        rospy.set_param("ctrlMel/kd_z", 1.7)
        rospy.set_param("ctrlMel/ki_z", 0.3)

        rospy.set_param("ctrlMel/ki_m_x", 2700.0)
        rospy.set_param("ctrlMel/ki_m_y", 2700.0)
        rospy.set_param("ctrlMel/ki_m_z", -2000.0)

        rospy.set_param("ctrlMel/time_instance", 0)
        rospy.set_param("myTraj/oscmagnitude", 0.35)

        # rospy.set_param("motorPowerSet/m1", 65535)
        # rospy.set_param("motorPowerSet/m2", 65535)
        # rospy.set_param("motorPowerSet/m3", 65535)
        # rospy.set_param("motorPowerSet/m4", 65535)

        ## update the params
        # self._update_params(["motorPowerSet/m1"])
        # self._update_params(["motorPowerSet/m2"])
        # self._update_params(["motorPowerSet/m3"])
        # self._update_params(["motorPowerSet/m4"])
        params_list = [["ctrlMel/massThrust"], ["commander/enHighLevel"], ["stabilizer/estimator"], ["stabilizer/controller"],
        ["kalman/resetEstimation"], ["ctrlMel/mass"], ["ctrlMel/i_range_z"], ["ctrlMel/i_range_y"], ["ctrlMel/i_range_x"], 
        ["ctrlMel/i_range_m_z"], ["ctrlMel/i_range_m_y"], ["ctrlMel/i_range_m_x"], ["ctrlMel/kR_x"], ["ctrlMel/kw_x"], ["myTraj/oscmagnitude"], 
        ["ctrlMel/kR_y"], ["ctrlMel/kw_y"], ["ctrlMel/kR_z"], ["ctrlMel/kw_z"], ["ctrlMel/kd_omega_rp"], ["ctrlMel/kp_x"],
        ["ctrlMel/kd_x"], ["ctrlMel/ki_x"], ["ctrlMel/kp_y"], ["ctrlMel/kd_y"], ["ctrlMel/ki_y"], ["ctrlMel/kp_z"], ["myTraj/height"],
        ["ctrlMel/kd_z"], ["ctrlMel/ki_z"], ["ctrlMel/ki_m_x"], ["ctrlMel/ki_m_y"], ["ctrlMel/ki_m_z"], ["ctrlMel/time_instance"]]
        for i in range(len(params_list)):
            self._update_params(params_list[i])
            time.sleep(0.05)

        time.sleep(1.0)

        rospy.loginfo(rospy.get_namespace()+": waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo(rospy.get_namespace()+": found emergency service")

        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
            rospy.loginfo(rospy.get_namespace()+": waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo(rospy.get_namespace()+": found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo(rospy.get_namespace()+": waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo(rospy.get_namespace()+": found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
            self._land = None
            self._takeoff = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        if data.axes[7] != 0:
            angle = rospy.get_param("myTraj/oscmagnitude")
            rospy.set_param("myTraj/oscmagnitude", angle+0.02*data.axes[7])
            self._update_params(["myTraj/oscmagnitude"])
            pass
            
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land != None:
                    self._land()
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
                    rospy.set_param("ctrlMel/i_m_err_z_init", 1.1)
                    rospy.set_param("ctrlMel/i_m_err_x_init", -3.8)
                    rospy.set_param("ctrlMel/i_m_err_y_init", 1.9)
                    rospy.set_param("ctrlMel/i_err_z_init", 3.5)
                    rospy.set_param("ctrlMel/i_err_x_init", 1.2)
                    rospy.set_param("ctrlMel/i_err_y_init", -1.2)
                    params_list = [["ctrlMel/i_m_err_z_init"], ["ctrlMel/i_m_err_x_init"],["ctrlMel/i_m_err_y_init"], 
                    ["ctrlMel/i_err_z_init"], ["ctrlMel/i_err_x_init"], ["ctrlMel/i_err_y_init"]]
                    for i in range(len(params_list)):
                        self._update_params(params_list[i])

                    self._takeoff()
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))



                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)

                    self._update_params(["ring/headlightEnable"])
                    print(not value)

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()