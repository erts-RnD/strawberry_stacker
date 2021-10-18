#!/usr/bin/env python3
import rospy
import math
from gazebo_msgs.msg import ModelState


class landing_mover():
    def __init__(self, model_name="landing_station", x_a=1, y_a=1, z_a=1, x_f=1, y_f=1, z_f=1, x_p=0, y_p=0, z_p=0):
        self.name = model_name
        self.x_a = x_a
        self.x_f = x_f
        self.x_p = x_p
        self.y_a = y_a
        self.y_f = y_f
        self.y_p = y_p
        self.z_a = z_a
        self.z_f = z_f
        self.z_p = z_p
        self.model_state = ModelState()
        self.model_state.model_name = self.name
    
    def update_model_state_pose_sin(self, time):
        self.model_state.pose.position.x = self.x_a * math.sin(self.x_f * time + self.x_p)
        self.model_state.pose.position.y = self.y_a * math.sin(self.y_f * time + self.y_p)
        self.model_state.pose.position.z = self.z_a * math.sin(self.z_f * time + self.z_p)

    def update_model_state_vel_sin(self, time):
        self.model_state.twist.linear.x = self.x_a * math.sin(self.x_f * time + self.x_p)
        self.model_state.twist.linear.y = self.y_a * math.sin(self.y_f * time + self.y_p)
        self.model_state.twist.linear.z = self.z_a * math.sin(self.z_f * time + self.z_p)


def hook():
    ls_1.update_model_state_pose_sin(0)
    model_state_pub.publish(ls_1.model_state)


if __name__ == '__main__':
    rate = 30
    rospy.init_node('moving_marker')
    model_name = rospy.get_param("~model_name", "landing_station")
    ls_1 = landing_mover(model_name, z_a=0, x_f=4)
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    r = rospy.Rate(rate)
    rospy.on_shutdown(hook)
    while not rospy.is_shutdown():
        try:
            time = rospy.get_time()
            ls_1.update_model_state_pose_sin(time)
            # ls_1.update_model_state_vel_sin(time)
            model_state_pub.publish(ls_1.model_state)
            r.sleep()
        except rospy.ROSInterruptException:
            pass
            # rospy.logerr("ROS Interrupt Exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Likely simulator reset")
