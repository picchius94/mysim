#!/usr/bin/env python
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


def sign_func(num):
    if (num >= 0):
        return 1
    else:
        return -1

class CuriosityMarsRoverAckerMan(object):
    def __init__(self):
        rospy.loginfo("CuriosityRoverAckerMan Initialising...")

        # TODO: Ackerman stuff
        self.distance_axis = 1.08
        self.distance_axis_middle = 1.2
        self.distance_front_center = 1.2
        self.distance_back_center = 1.2
        self.wheel_radius = 0.242647

        self.publishers_curiosity_d = {}
        self.controller_ns = "curiosity_mars_rover"
        self.controller_command = "command"
        self.controllers_list = [   "back_wheel_L_joint_velocity_controller",
                                    "back_wheel_R_joint_velocity_controller",
                                    "front_wheel_L_joint_velocity_controller",
                                    "front_wheel_R_joint_velocity_controller",
                                    "middle_wheel_L_joint_velocity_controller",
                                    "middle_wheel_R_joint_velocity_controller",
                                    "suspension_arm_B2_L_joint_position_controller",
                                    "suspension_arm_B2_R_joint_position_controller",
                                    "suspension_arm_B_L_joint_position_controller",
                                    "suspension_arm_B_R_joint_position_controller",
                                    "suspension_arm_F_L_joint_position_controller",
                                    "suspension_arm_F_R_joint_position_controller",
                                    "suspension_steer_B_L_joint_position_controller",
                                    "suspension_steer_B_R_joint_position_controller",
                                    "suspension_steer_F_L_joint_position_controller",
                                    "suspension_steer_F_R_joint_position_controller"
                                ]

        for controller_name in self.controllers_list:
            topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
            self.publishers_curiosity_d[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)

        self.wait_publishers_to_be_ready()
        self.init_publisher_variables()
        self.init_state()

        self.cmd_vel_msg = Twist()
        cmd_vel_topic = "/cmd_vel"
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        rospy.logwarn("CuriosityMarsRoverAckerMan...READY")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        for controller_name, publisher_obj in self.publishers_curiosity_d.iteritems():
            publisher_ready = False
            while not publisher_ready:
                rospy.logwarn("Checking Publisher for ==>"+str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """
        # Get the publishers for wheel speed
        self.back_wheel_L = self.publishers_curiosity_d[self.controllers_list[0]]
        self.back_wheel_R = self.publishers_curiosity_d[self.controllers_list[1]]
        self.front_wheel_L = self.publishers_curiosity_d[self.controllers_list[2]]
        self.front_wheel_R = self.publishers_curiosity_d[self.controllers_list[3]]
        self.middle_wheel_L = self.publishers_curiosity_d[self.controllers_list[4]]
        self.middle_wheel_R = self.publishers_curiosity_d[self.controllers_list[5]]
        # Get the publishers for suspension
        self.suspension_arm_B2_L = self.publishers_curiosity_d[self.controllers_list[6]]
        self.suspension_arm_B2_R = self.publishers_curiosity_d[self.controllers_list[7]]
        self.suspension_arm_B_L = self.publishers_curiosity_d[self.controllers_list[8]]
        self.suspension_arm_B_R = self.publishers_curiosity_d[self.controllers_list[9]]
        self.suspension_arm_F_L = self.publishers_curiosity_d[self.controllers_list[10]]
        self.suspension_arm_F_R = self.publishers_curiosity_d[self.controllers_list[11]]
        # Get the publishers for steering
        self.suspension_steer_B_L = self.publishers_curiosity_d[self.controllers_list[12]]
        self.suspension_steer_B_R = self.publishers_curiosity_d[self.controllers_list[13]]
        self.suspension_steer_F_L = self.publishers_curiosity_d[self.controllers_list[14]]
        self.suspension_steer_F_R = self.publishers_curiosity_d[self.controllers_list[15]]

        # Init Messages
        self.back_wheel_L_velocity_msg = Float64()
        self.back_wheel_R_velocity_msg = Float64()
        self.front_wheel_L_velocity_msg = Float64()
        self.front_wheel_R_velocity_msg = Float64()
        self.middle_wheel_L_velocity_msg = Float64()
        self.middle_wheel_R_velocity_msg = Float64()

        self.suspension_arm_B2_L_pos_msg = Float64()
        self.suspension_arm_B2_R_pos_msg = Float64()
        self.suspension_arm_B_L_pos_msg = Float64()
        self.suspension_arm_B_R_pos_msg = Float64()
        self.suspension_arm_F_L_pos_msg = Float64()
        self.suspension_arm_F_R_pos_msg = Float64()

        self.suspension_steer_B_L_pos_msg = Float64()
        self.suspension_steer_B_R_pos_msg = Float64()
        self.suspension_steer_F_L_pos_msg = Float64()
        self.suspension_steer_F_R_pos_msg = Float64()



    def init_state(self):
        self.set_suspension_mode("standard")
        self.set_turning_radius(None,0.0)
        self.set_wheels_speed(None,0.0)

    def set_suspension_mode(self, mode_name):

        if mode_name == "standard":

            self.suspension_arm_B2_L_pos_msg.data = -0.2
            self.suspension_arm_B2_R_pos_msg.data = -0.2
            self.suspension_arm_B_L_pos_msg.data = -0.2
            self.suspension_arm_B_R_pos_msg.data = -0.2
            self.suspension_arm_F_L_pos_msg.data = 0.2
            self.suspension_arm_F_R_pos_msg.data = 0.2

            self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
            self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
            self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
            self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
            self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
            self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)

    def set_turning_radius(self, w_ang, v_lin):

        if not w_ang:
            # We dont need Ackerman calculations, its not turn.
            self.suspension_steer_B_L_pos_msg.data = 0.0
            self.suspension_steer_B_R_pos_msg.data = 0.0
            self.suspension_steer_F_L_pos_msg.data = 0.0
            self.suspension_steer_F_R_pos_msg.data = 0.0
        else:
            if (0.0 <= abs(v_lin/w_ang-self.distance_axis) < 1e-10):
                self.suspension_steer_B_L_pos_msg.data = -math.pi/2
                self.suspension_steer_F_L_pos_msg.data = math.pi/2
            else:
                self.suspension_steer_B_L_pos_msg.data = -1*math.atan(self.distance_back_center/(v_lin/w_ang-self.distance_axis))
                self.suspension_steer_F_L_pos_msg.data = math.atan(self.distance_front_center/(v_lin/w_ang-self.distance_axis))

            if (0.0 <= abs(v_lin/w_ang+self.distance_axis) < 1e-10):
                self.suspension_steer_B_R_pos_msg.data = -math.pi/2
                self.suspension_steer_F_R_pos_msg.data = math.pi/2
            else:
               self.suspension_steer_B_R_pos_msg.data = -1*math.atan(self.distance_back_center/(v_lin/w_ang+self.distance_axis))
               self.suspension_steer_F_R_pos_msg.data = math.atan(self.distance_front_center/(v_lin/w_ang+self.distance_axis))

        self.suspension_steer_B_L.publish(self.suspension_steer_B_L_pos_msg)
        self.suspension_steer_B_R.publish(self.suspension_steer_B_R_pos_msg)
        self.suspension_steer_F_L.publish(self.suspension_steer_F_L_pos_msg)
        self.suspension_steer_F_R.publish(self.suspension_steer_F_R_pos_msg)

    def set_wheels_speed(self, w_ang, v_lin):
        """
        Sets the turning speed in radians per second
        :param turning_speed: In radians per second
        :return:
        """

        if not w_ang:
            # We dont need Ackerman calculations, its not turn.
            self.back_wheel_L_velocity_msg.data = v_lin/self.wheel_radius
            self.back_wheel_R_velocity_msg.data = -1*v_lin/self.wheel_radius
            self.front_wheel_L_velocity_msg.data = v_lin/self.wheel_radius
            self.front_wheel_R_velocity_msg.data = -1*v_lin/self.wheel_radius
            self.middle_wheel_L_velocity_msg.data = v_lin/self.wheel_radius
            self.middle_wheel_R_velocity_msg.data = -1*v_lin/self.wheel_radius
        else:
            self.back_wheel_L_velocity_msg.data = w_ang*sign_func(v_lin/w_ang-self.distance_axis)*((self.distance_back_center**2+(v_lin/w_ang-self.distance_axis)**2)**0.5)/self.wheel_radius
            self.front_wheel_L_velocity_msg.data = w_ang*sign_func(v_lin/w_ang-self.distance_axis)*((self.distance_front_center**2+(v_lin/w_ang-self.distance_axis)**2)**0.5)/self.wheel_radius
            self.back_wheel_R_velocity_msg.data = -1*w_ang*sign_func(v_lin/w_ang+self.distance_axis)*((self.distance_back_center**2+(v_lin/w_ang+self.distance_axis)**2)**0.5)/self.wheel_radius
            self.front_wheel_R_velocity_msg.data = -1*w_ang*sign_func(v_lin/w_ang+self.distance_axis)*((self.distance_front_center**2+(v_lin/w_ang+self.distance_axis)**2)**0.5)/self.wheel_radius
            self.middle_wheel_L_velocity_msg.data = (v_lin-w_ang*self.distance_axis_middle)/self.wheel_radius
            self.middle_wheel_R_velocity_msg.data = -1*(v_lin+w_ang*self.distance_axis_middle)/self.wheel_radius

        self.back_wheel_L.publish(self.back_wheel_L_velocity_msg)
        self.back_wheel_R.publish(self.back_wheel_R_velocity_msg)
        self.front_wheel_L.publish(self.front_wheel_L_velocity_msg)
        self.front_wheel_R.publish(self.front_wheel_R_velocity_msg)
        self.middle_wheel_L.publish(self.middle_wheel_L_velocity_msg)
        self.middle_wheel_R.publish(self.middle_wheel_R_velocity_msg)

    def move_forwards(self):
        self.set_wheels_speed(None,10.0)
        self.set_turning_radius(None,10.0)

    def move_backwards(self):
        self.set_wheels_speed(None,-10.0)
        self.set_turning_radius(None,-10.0)

    def move_turn_left(self):
        self.set_wheels_speed(1.0, 10.0)
        self.set_turning_radius(1.0, 10.0)

    def move_turn_right(self):
        self.set_wheels_speed(-1.0, 10.0)
        self.set_turning_radius(-1.0, 10.0)

    def move_turn_stop(self):
        self.set_wheels_speed(None, 0.0)
        self.set_turning_radius(None, 0.0)


    def move_with_cmd_vel(self):
        linear_speed = self.cmd_vel_msg.linear.x
        angular_speed = self.cmd_vel_msg.angular.z
        if angular_speed == 0.0:
            angular_speed = None

        rospy.logdebug("angular_speed="+str(angular_speed)+",linear_speed="+str(linear_speed))
        self.set_turning_radius(angular_speed, linear_speed)
        self.set_wheels_speed(angular_speed, linear_speed)



if __name__ == "__main__":
    rospy.init_node("CuriosityRoverAckerMan_node", log_level=rospy.INFO)
    curiosity_mars_rover_ackerman_object = CuriosityMarsRoverAckerMan()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        curiosity_mars_rover_ackerman_object.move_with_cmd_vel()
        rate.sleep()

