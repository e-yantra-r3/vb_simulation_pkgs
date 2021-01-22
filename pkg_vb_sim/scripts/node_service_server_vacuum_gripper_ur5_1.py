#! /usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.msg import LogicalCameraImage


class VacuumGripper():

    # Constructor
    def __init__(self):
        
        param_config_vacuum_gripper = rospy.get_param('config_vacuum_gripper_ur5_1')
        
        self._vacuum_gripper_model_name = param_config_vacuum_gripper['vacuum_gripper_model_name']
        self._vacuum_gripper_link_name = param_config_vacuum_gripper['vacuum_gripper_link_name']
        
        self._object_model_name = ""
        self._object_link_name = param_config_vacuum_gripper['attachable_object_link_name']
        
        self._attachable_object_prefix = param_config_vacuum_gripper['attachable_object_prefix']
        self._attachable_object_delimiter = param_config_vacuum_gripper['attachable_object_delimiter']
        self._logical_camera_topic_name = param_config_vacuum_gripper['logical_camera_topic_name']
        
        print(param_config_vacuum_gripper)
        
        self._flag_pickable = False
        self._flag_plugin_in_use = False
        self._count = 0

        

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        self._attach_srv_a = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
        self._attach_srv_a.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        self._attach_srv_d = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        self._attach_srv_d.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        rospy.loginfo( '\033[94m' + " >>> Vacuum Gripper init done." + '\033[0m')

    
    def activate_vacuum_gripper(self):
        
        rospy.set_param("vacuum_gripper_plugin_in_usage", True)

        rospy.loginfo("Attach request received")
        req = AttachRequest()
        req.model_name_1 = self._vacuum_gripper_model_name
        req.link_name_1 = self._vacuum_gripper_link_name
        req.model_name_2 = self._object_model_name
        req.link_name_2 = self._object_link_name
        self._attach_srv_a.call(req)

        rospy.set_param("vacuum_gripper_plugin_in_usage", False)

    
    
    def deactivate_vacuum_gripper(self):
      
      rospy.set_param("vacuum_gripper_plugin_in_usage", True)

      rospy.loginfo("Detach request received")
      req = AttachRequest()
      req.model_name_1 = self._vacuum_gripper_model_name
      req.link_name_1 = self._vacuum_gripper_link_name
      req.model_name_2 = self._object_model_name
      req.link_name_2 = self._object_link_name
      self._attach_srv_d.call(req)

      rospy.set_param("vacuum_gripper_plugin_in_usage", False)

    
    
    def callback_service_on_request(self, req):

        self._flag_plugin_in_use = rospy.get_param("vacuum_gripper_plugin_in_usage")

        rospy.loginfo( '\033[94m' + " >>> Vacuum Gripper Activate: {}".format(req.activate_vacuum_gripper) + '\033[0m')
        rospy.loginfo( '\033[94m' + " >>> Vacuum Gripper Flag Pickable: {}".format(self._flag_pickable) + '\033[0m')
        rospy.loginfo( '\033[94m' + " >>> Vacuum Gripper Plugin in Use: {}".format(self._flag_plugin_in_use) + '\033[0m')

        
        if( (req.activate_vacuum_gripper is True) and (self._flag_pickable is True) and (self._flag_plugin_in_use is False) ):
            self.activate_vacuum_gripper()
            return vacuumGripperResponse(True)
        else:
            # self._flag_pickable = False
            self.deactivate_vacuum_gripper()
            return vacuumGripperResponse(False)
        
        

    def callback_topic_subscription(self, rx_msg):
        # rospy.logwarn( '\033[94m' + "{}".format(rx_msg) + '\033[0m')
        
        self._count += 1

        number_models = len(rx_msg.models)

        if ( (self._count > 1) and (number_models == 0) ):
            return

        elif ( (self._count > 1) and (number_models == 0) ):
            flag_attachable_object_found = False
            self._flag_pickable = False
            self._count = 0

        else:
            for i in range(0, number_models):
                name_model = rx_msg.models[i].type
                
                lst_name_model = name_model.split(self._attachable_object_delimiter)
                
                if(lst_name_model[0] == self._attachable_object_prefix):
                    rospy.loginfo( '\033[94m' + ">>> [ur5_1] Vacuum Gripper: Pickable object found {}. Pickable: {}".format(name_model, self._flag_pickable) + '\033[0m')
                    
                    self._object_model_name = name_model

                    flag_attachable_object_found = True
                    self._flag_pickable = True
                    break
            
        # if(flag_attachable_object_found is False):
        #     rospy.logwarn("making flag pickable False")
        #     self._flag_pickable = False
            
        
        

    # Destructor
    def __del__(self):
        rospy.loginfo( '\033[94m' + " >>> Vacuum Gripper Del." + '\033[0m')





def main():
    rospy.init_node('node_service_server_vacuum_gripper_ur5_1')
    
    ur5_vacuum_gripper = VacuumGripper()
    
    s = rospy.Service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper, ur5_vacuum_gripper.callback_service_on_request)
    rospy.loginfo( '\033[94m' + " >>> Vacuum Gripper Activation Service Ready." + '\033[0m')
    
    rospy.Subscriber(ur5_vacuum_gripper._logical_camera_topic_name, LogicalCameraImage, ur5_vacuum_gripper.callback_topic_subscription)
    
    rospy.spin()


if __name__ == "__main__":
    main()