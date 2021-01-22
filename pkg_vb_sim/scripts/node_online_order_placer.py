#!/usr/bin/env python

# ROS Node - MQTT Online Order Placer

import rospy
import paho.mqtt.client as mqtt
import time
import datetime
import json


class OnlineOrderPlacer:

    def __init__(self):

        rospy.init_node('node_online_order_placer')

        
        param_online_order_config = rospy.get_param('online_order_config')
        self._mqtt_server_url = param_online_order_config['mqtt_server_url']
        self._mqtt_server_port = param_online_order_config['mqtt_server_port']
        self._mqtt_qos = param_online_order_config['mqtt_qos']
        self._mqtt_unique_id = param_online_order_config['mqtt_unique_id']
        
        rospy.loginfo(param_online_order_config)

        param_online_order_intervals = rospy.get_param('online_order_intervals')
        self._interval_package_1 = param_online_order_intervals['package_1']
        self._interval_package_2 = param_online_order_intervals['package_2']
        self._interval_package_3 = param_online_order_intervals['package_3']
        self._interval_package_4 = param_online_order_intervals['package_4']
        self._interval_package_5 = param_online_order_intervals['package_5']
        self._interval_package_6 = param_online_order_intervals['package_6']
        self._interval_package_7 = param_online_order_intervals['package_7']
        self._interval_package_8 = param_online_order_intervals['package_8']
        self._interval_package_9 = param_online_order_intervals['package_9']

        rospy.loginfo(param_online_order_intervals)
        # self._server_url = "broker.mqttdashboard.com"
        # self._server_port = 1883
        # self._qos = 0
        # self._pub_topic = "/eyrc/vb/eyantraiitb/orders"
        # self._interval_list = [20, 25, 30, 60, 65, 70, 100, 105, 110]

        self._server_url = self._mqtt_server_url
        self._server_port = self._mqtt_server_port
        self._qos = self._mqtt_qos
        self._pub_topic = "/eyrc/vb/" + self._mqtt_unique_id + "/orders"
        self._interval_list = [
            self._interval_package_1,
            self._interval_package_2,
            self._interval_package_3,
            self._interval_package_4,
            self._interval_package_5,
            self._interval_package_6,
            self._interval_package_7,
            self._interval_package_8,
            self._interval_package_9] 
        

        self._flag_order_placed_1 = False
        self._flag_order_placed_2 = False
        self._flag_order_placed_3 = False

        self._flag_order_placed_4 = False
        self._flag_order_placed_5 = False
        self._flag_order_placed_6 = False

        self._flag_order_placed_7 = False
        self._flag_order_placed_8 = False
        self._flag_order_placed_9 = False

        self._mqtt_client = mqtt.Client()
        self._mqtt_client.on_publish = self.mqtt_on_publish
        self._mqtt_client.connect(self._server_url, self._server_port)
        self._mqtt_client.loop_start()

    def mqtt_on_publish(self, client, userdata, mid):
        # print("mid: " + str(mid))
        rospy.loginfo("Order Placed.")

    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    
    def place_online_order(self, arg_order_id, arg_item, arg_quantity, arg_city, arg_lat, arg_lon):
        
        dict_payload = {
            "order_id" : arg_order_id,
            "order_time" : self.get_time_str(),
            "item": arg_item,
            "qty": arg_quantity,
            "city": arg_city,
            "lat": arg_lat,
            "lon": arg_lon
        }

        str_payload = json.dumps(dict_payload)

        (rc, mid) = self._mqtt_client.publish(self._pub_topic, str_payload, qos = self._qos)

    

 

# Main Function
def main():

    online_order = OnlineOrderPlacer()

    while not rospy.is_shutdown():

        sim_time_now = rospy.get_rostime()
        # rospy.logwarn("Current Sim Time: %i", sim_time_now.secs)

        
        if( (sim_time_now.secs == online_order._interval_list[0]) and (online_order._flag_order_placed_1 is False) ):
            online_order.place_online_order("3001", "Clothes", "1", "Mumbai", "19.0760 N", "72.8777 E")
            online_order._flag_order_placed_1 = True
        
        # 28.7041 N, 77.1025 E
        elif( (sim_time_now.secs == online_order._interval_list[1]) and (online_order._flag_order_placed_2 is False) ):
            online_order.place_online_order("1001", "Medicine", "1", "Delhi", "28.7041 N", "77.1025 E")
            online_order._flag_order_placed_2 = True

        # 13.0827 N, 80.2707 E
        elif( (sim_time_now.secs == online_order._interval_list[2]) and (online_order._flag_order_placed_3 is False) ):
            online_order.place_online_order("2001", "Food", "1", "Chennai", "13.0827 N", "80.2707 E")
            online_order._flag_order_placed_3 = True

        # 22.5726 N, 88.3639 E
        elif( (sim_time_now.secs == online_order._interval_list[3]) and (online_order._flag_order_placed_4 is False) ):
            online_order.place_online_order("2002", "Food", "1", "Kolkata", "22.5726 N", "88.3639 E")
            online_order._flag_order_placed_4 = True
        
        # 18.5204 N, 73.8567 E
        elif( (sim_time_now.secs == online_order._interval_list[4]) and (online_order._flag_order_placed_5 is False) ):
            online_order.place_online_order("3002", "Clothes", "1", "Pune", "18.5204 N", "73.8567 E")
            online_order._flag_order_placed_5 = True

        # 17.3850 N, 78.4867 E
        elif( (sim_time_now.secs == online_order._interval_list[5]) and (online_order._flag_order_placed_6 is False) ):
            online_order.place_online_order("1002", "Medicine", "1", "Hyderabad", "17.3850 N", "78.4867 E")
            online_order._flag_order_placed_6 = True

        # 12.9716 N, 77.5946 E
        elif( (sim_time_now.secs == online_order._interval_list[6]) and (online_order._flag_order_placed_7 is False) ):
            online_order.place_online_order("3003", "Clothes", "1", "Bangalore", "12.9716 N", "77.5946 E")
            online_order._flag_order_placed_7 = True
        
        # 31.6340 N, 74.8723 E
        elif( (sim_time_now.secs == online_order._interval_list[7]) and (online_order._flag_order_placed_8 is False) ):
            online_order.place_online_order("1003", "Medicine", "1", "Amritsar", "31.6340 N", "74.8723 E")
            online_order._flag_order_placed_8 = True

        # 23.8315 N, 91.2868 E
        elif( (sim_time_now.secs == online_order._interval_list[8]) and (online_order._flag_order_placed_9 is False) ):
            online_order.place_online_order("2003", "Food", "1", "Agartala", "23.8315 N", "91.2868 E")
            online_order._flag_order_placed_9 = True

        else:
            pass

            
    rospy.spin()


if __name__ == '__main__':
    main()
