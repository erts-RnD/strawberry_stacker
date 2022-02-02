#!/usr/bin/env python3

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt8
from copy import deepcopy
import pandas as pd
import random

rospy.init_node('spawn_boxes')
info_pub=rospy.Publisher('/spawn_info' ,UInt8, queue_size=1)
spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
spawn_srv.wait_for_service()
rospy.loginfo("Connected to service!")
clock_time=0

def clock_cb(msg):
  global clock_time
  clock_time = msg.clock.secs


box_count=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
max_count=10
rand=[-0.15, 0.44, 0.04, -0.84, -0.66, -0.1, 0.04, 0.46, -0.54, 0.19, 0.64, 0.32, -0.14, 0.22, -0.11, -0.84, 0.35, 0.46, -0.4, 0.81, 0.57, -0.86, 0.08, -0.92, -0.38, 0.9, -0.53, -0.85, -0.84, 0.3]
total_box_count = 0
req = SpawnModelRequest()



red_box_xml="""<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="red_box">
    <pose>0 0 0.078 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.000001423</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000002173</iyy>
          <iyz>0</iyz>
          <izz>0.0000034166</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.24 0.104</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 -0.052 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://red_box/meshes/parcel_box.obj</uri>
            <scale>0.6 0.6 0.4</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>"""

blue_box_xml="""<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="blue_box">
    <pose>0 0 0.078 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.000001423</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000002173</iyy>
          <iyz>0</iyz>
          <izz>0.0000034166</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.24 0.104</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 -0.052 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://blue_box/meshes/parcel_box.obj</uri>
            <scale>0.6 0.6 0.4</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

def spawn_box (row, box_number, color):
  global total_box_count

  if (color=='blue'):
    req.model_name = 'box_'+ str(total_box_count)
    req.model_xml = blue_box_xml
    req.initial_pose.position.x = 2 + (box_number-1)*7 + rand[total_box_count]
    req.initial_pose.position.y = 1 + (row-1)*4
    req.initial_pose.position.z = 0.053
    total_box_count=total_box_count+1
    spawn_srv.call(req)
    info_pub.publish(row)
  if (color=='red'):
    req.model_name = 'box_'+ str(total_box_count)
    req.model_xml = red_box_xml
    req.initial_pose.position.x =  2 + (box_number-1)*7 + rand[total_box_count]
    req.initial_pose.position.y = 1 + (row-1)*4 
    req.initial_pose.position.z = 0.053
    total_box_count=total_box_count+1
    spawn_srv.call(req)
    info_pub.publish(row)


if __name__ == '__main__':
    rospy.Subscriber("/clock", Clock, clock_cb)
    rp=rospkg.RosPack()
    pkg_path=rp.get_path('task_5')
    csv_data=pd.read_csv(pkg_path+'/scripts/config.csv')
    csv_data=csv_data.sort_values(by=['time'])
    data = csv_data.values.tolist()
    print(pkg_path+'/scripts/config.csv')

    while not rospy.is_shutdown():
      if data:
        if (clock_time==data[0][0]):
          box_count[data[0][1]]=box_count[data[0][1]]+1
          if (box_count[data[0][1]]<max_count):
            spawn_box(data[0][1],box_count[data[0][1]],data[0][2])
            del data[0]
            
          else :
            print("Box count for row exceeded")
        else:
          continue
      else:
        break 
