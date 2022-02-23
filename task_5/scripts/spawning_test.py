#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt8
import pandas as pd

clock_time = 0


def clock_cb(msg):
    global clock_time
    clock_time = msg.clock.secs


box_count = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
max_box_per_row = 10
rand = [-0.15, 0.44, 0.04, -0.84, -0.66, -0.1, 0.04, 0.46, -0.54, 0.19, 0.64, 0.32, -0.14, 0.22, -0.11, -0.84, 0.35, 0.46, -0.4, 0.81, 0.57, -0.86, 0.08, -0.92, -0.38, 0.9, -0.53, -0.85, -0.84, 0.3]
total_box_count = 0
total_blue_count = 0
total_red_count = 0


def spawn_box(row, box_number, color, state_msg):
    global total_box_count, total_blue_count, total_red_count
    if color == 'blue':
        state_msg.model_name = 'box_'+str(20+total_blue_count)
        total_blue_count = total_blue_count + 1
    elif color == 'red':
        state_msg.model_name = 'box_'+str(total_red_count)
        total_red_count = total_red_count + 1
    state_msg.pose.position.x = 2 + (box_number-1)*7 + rand[total_box_count]
    state_msg.pose.position.y = 1 + (row-1)*4
    state_msg.pose.position.z = 0.053
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state(state_msg)
    except rospy.ServiceException as e:
        print(str(e))
    total_box_count = total_box_count+1
    info_pub.publish(row)


if __name__ == '__main__':
    rospy.init_node('spawn_boxes')
    info_pub = rospy.Publisher('/spawn_info', UInt8, queue_size=1)
    rospy.Subscriber("/clock", Clock, clock_cb)
    rp = rospkg.RosPack()
    pkg_path = rp.get_path('task_5')
    csv_data = pd.read_csv(pkg_path+'/scripts/config.csv')
    csv_data = csv_data.sort_values(by=['time'])
    data = csv_data.values.tolist()
    print(pkg_path+'/scripts/config.csv')
    state_msg = ModelState()
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    while not rospy.is_shutdown():
        if data:
            if (clock_time == data[0][0]):
                box_count[data[0][1]] = box_count[data[0][1]]+1
                if (box_count[data[0][1]] < max_box_per_row):
                    spawn_box(data[0][1], box_count[data[0][1]], data[0][2], state_msg)
                    del data[0]
                else:
                    print("Box count for row exceeded")
            else:
                continue
        else:
            break
