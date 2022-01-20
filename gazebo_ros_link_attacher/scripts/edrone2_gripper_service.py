#! /usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse, Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String


class edrone_gripper():

    # Constructor
    def __init__(self):

        rospy.init_node('edrone_1_service_server_gripper')
        self._attach_srv_a = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self._attach_srv_a.wait_for_service()

        self._attach_srv_d = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self._attach_srv_d.wait_for_service()
        self.model_state_msg = ModelStates()
        self.box_model_name_list = ['box_0', 'box_1', 'box_2', 'box_3', 'box_4', 'box_5', 'box_6', 'box_7', 'box_8', 'box_9', 'box_10', 'box_11', 'box_12', 'box_13', 'box_14', 'box_15', 'box_16', 'box_17', 'box_18']
        self.drone_model_name = 'edrone1'
        rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, self.model_state_callback)
        self.check_pub = rospy.Publisher('/edrone1/gripper_check', String, queue_size=1)
        self.gripper_service = rospy.Service('/edrone1/activate_gripper', Gripper, self.callback_service_on_request)

    # Destructor
    def __del__(self):
        rospy.loginfo('\033[94m' + " >>> Gripper Del." + '\033[0m')

    def model_state_callback(self, msg):
        self.model_state_msg.name = msg.name
        self.model_state_msg.pose = msg.pose
        self.model_state_msg.twist = msg.twist

    def callback_service_on_request(self, req):
        pickable, box_name = self.check()
        rospy.loginfo('\033[94m' + " >>> Gripper Activate: {}".format(req.activate_gripper) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Gripper Flag Pickable: {}".format(str(pickable)) + '\033[0m')
        if pickable:
            if(req.activate_gripper is True):
                self.activate_gripper(box_name)
                return GripperResponse(True)
            else:
                self.deactivate_gripper(box_name)
                return GripperResponse(False)
        else:
            return GripperResponse(False)

    def activate_gripper(self, model_name_2):
        rospy.loginfo("Attach request received")
        req = AttachRequest()
        req.model_name_1 = 'edrone1'
        req.link_name_1 = 'base_link'
        req.model_name_2 = model_name_2
        req.link_name_2 = 'link'
        self._attach_srv_a.call(req)

    def deactivate_gripper(self, model_name_2):
        rospy.loginfo("Detach request received")
        req = AttachRequest()
        req.model_name_1 = 'edrone1'
        req.link_name_1 = 'base_link'
        req.model_name_2 = model_name_2
        req.link_name_2 = 'link'
        self._attach_srv_d.call(req)

    def check(self):
        try:
            drone_index = self.model_state_msg.name.index(self.drone_model_name)
            dr_0 = self.model_state_msg.pose[drone_index].position.x
            dr_1 = self.model_state_msg.pose[drone_index].position.y
            dr_2 = self.model_state_msg.pose[drone_index].position.z
        except Exception as err:
            drone_index = -1
        pickable = False
        box_name = "None"
        if drone_index != -1:
            for box_model_name in self.box_model_name_list:
                try:
                    box_index = self.model_state_msg.name.index(box_model_name)
                    bx_0 = self.model_state_msg.pose[box_index].position.x
                    bx_1 = self.model_state_msg.pose[box_index].position.y
                    bx_2 = self.model_state_msg.pose[box_index].position.z
                except Exception as err:
                    box_index = -1
                if(box_index != -1):
                    if(abs(dr_0 - bx_0) < 0.15 and abs(dr_1 - bx_1) < 0.15 and (bx_2 - dr_2) < 0.25):
                        pickable = True
                        box_name = box_model_name
                        break
        return pickable, box_name

    def publish_check(self, pickable_flag):
        self.check_pub.publish(str(pickable_flag))


def main():
    eDrone_gripper = edrone_gripper()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            pickable, _ = eDrone_gripper.check()
            eDrone_gripper.publish_check(pickable)
            r.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("Shtdown Req")


if __name__ == "__main__":
    main()
