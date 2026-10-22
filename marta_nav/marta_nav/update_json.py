#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import marta_nav.restest.restthru as restthru
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from nav_msgs.msg import Odometry
import numpy as np
import math

restthru.http_init()

def quaternion_to_euler(q):
    # Normalizando o quaternion
    norm = (q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)**0.5
    w, x, y, z = q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm

    # Cálculo dos ângulos de Euler
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return roll, pitch, yaw

class WM_update(Node):
    def __init__(self):
        super().__init__("WM_update_node")
        self.declare_parameter(
            name="server_address",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Server IP adress",
            ),
        )
        self.declare_parameter(
            name="File_name",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="WM filme name",
            ),
        )
        self.host = 'http://10.1.3.106:8000'#(self.get_parameter("server_address").get_parameter_value().string_value)
        self.file = (self.get_parameter("File_name").get_parameter_value().string_value)
        self.timer = self.create_timer(0.1, self.update)
        self.url = self.host + "/agents"
        self.pose = []
        self.pose.append(0)
        self.pose.append(0)
        self.pose.append(0)
        self.localization_sub = self.create_subscription(Odometry, 'P3DX/odom', self.update_loc, 10)
        print('Inicializou')
    
    def update_loc(self, data):
        #self.get_logger().info("OIe")
        self.pose[0]= data.pose.pose.position.x + 19.16
        self.pose[1] = data.pose.pose.position.y - 0.2
        roll,pitch,yaw = quaternion_to_euler([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.pose[2] = yaw - math.pi

    def update(self):
        url = self.url + "/69/pose"
        print(url)
        #enviar = {} 
        #enviar["pose"] = self.pose
        status = restthru.http_put(url, self.pose)
        if status[0] == 200:
            self.get_logger().info("Server update")
        else:
            self.get_logger().info(f"Error: {status}")
        print(url)
        #dados, status = restthru.http_get(url)
        #print(dados)
        #exit()
        #dados1, status2 = restthru.http_get()
        #agents = dados.get('agents', [])
        #data = {}
        if status == 200:
            with open(self.file, "r") as file:
                data = json.load(file)
                #old_agents = dados.get('agents', [])
                #self.get_logger().info(dados)
            for update in data:
                agent_found = False
                for agent in data["agents"]:
                    if agent["id"] == update["id"]:
                        agent["current_room"] = update["current_room"]
                        agent_found = True
                        break
                if not agent_found:
                    data["agents"].append(update)
            with open(self.file, 'w') as file:
                json.dump(data, file, indent=4)
        else:
            self.get_logger().error("Erro de conexão")
def main():
    rclpy.init()
    node = WM_update()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
