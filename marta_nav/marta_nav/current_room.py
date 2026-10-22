import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import numpy as np
#from aruco_msgs.msg import MarkerArray
from math import pi
from transformations import rotation_matrix, euler_from_matrix, identity_matrix, translation_matrix, concatenate_matrices
import marta_nav.restest.restthru
import json
from nav_msgs.msg import Odometry


x = [3.17, 3.85, 1.5, 0.0]
y = [10.5, 2.25, 0.0, -3.26, 2.25, 10.5]

ipjsonzao = 'http://10.1.3.103:8000/agents/68/current_room'
host = 'http://10.1.3.130:4950'

current = '/agents/68/current_room'
pose_path = '/agents/68/pose'
path = '/agents/'

salas = {
    "kitchen": [[19.16,-0.17], [20.6, -0.17], [20.6, -2.64], [19.16, -2.64]],
    "living_room": [
                [19.16,-2.64],[23.84,-2.64],[23.84,-4.77],[19.16,-4.77]],
    "bedroom": [
                [
                    19.16,
                    -8.37
                ],
                [
                    19.16,
                    -4.77
                ],
                [
                    23.84,
                    -4.77
                ],
                [
                    23.84,
                    -8.37
                ]
            ]
}


class RoomClassifier(Node):
    def __init__(self):
        super().__init__('room_classifier')
        self.subscriber_ = self.create_subscription(Odometry, 'P3DX/odom', self.callback_odometry, 10)
        self.timer = self.create_timer(1.0, self.update)
        self.sala = "living room"
        marta_nav.restest.restthru.http_init()
        self.nova = True

    def callback_odometry(self, data):
        sala_encontrada = self.determine_room(data.pose.pose.position.x + 19.35, data.pose.pose.position.y - 0.2)
        #print(sala_encontrada)
        if sala_encontrada != self.sala:
            self.nova = True
            self.sala = sala_encontrada

    
    def update(self):
        a = {}
        a['current_room'] = self.sala
        marta_nav.restest.restthru.http_put(ipjsonzao, self.sala)
        if self.nova:
            
            self.nova = False
            try:
                    with open("/home/maria/demo/ros2_ws/src/marta_nav/files_demo/agents.json", "r+") as arquivo:
                        # Escreva os dados no arquivo usando o método json.dump()
                        dados = json.load(arquivo)
                        agentes = dados.get('agents', [])
                        print(self.sala)
                        for agente in agentes:
                            if agente.get('id') == 68:
                                agente['current_room'] = self.sala
                                
                                arquivo.seek(0)
                                arquivo.truncate()
                                json.dump(dados, arquivo)
            except:
                self.get_logger().warning("Erro ao escrever")
    def determine_room(self, x, y):
        for room, vertices in salas.items():
            if self.is_inside_polygon(x, y, vertices):
                return room

        return "Outside"

    def is_inside_polygon(self, x, y, vertices):
        n = len(vertices)
        inside = False

        p1x, p1y = vertices[0]
        for i in range(n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
            p1x, p1y = p2x, p2y

        return inside

def main(args=None):
    rclpy.init()
    node = RoomClassifier()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()


