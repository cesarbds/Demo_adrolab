import numpy as np
import math
import rclpy
from rclpy.node import Node
import marta_nav.restest.restthru
from geometry_msgs.msg import Twist


class control_p3dx(Node):    
    def __init__(self):
        super().__init__('speed_controller')
        self.angular = 0
        self.linear  = 0
        self.updated = False
        self.raio = 0.195/2.0
        self.b = 0.195
        self.client_id = id
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.callback_cmd_vel, 1)
        self.subscription

    def callback_cmd_vel(self, data):
        self.linear = data.linear.x
        self.angular = data.angular.z#s180/math.pi
        self.updated = True
        
    
    def send_speed(self):
        if self.updated:
            Vl = self.linear - self.angular*self.b
            Vr = self.linear + self.angular*self.b
            Wr = Vr *1000
            Wl = Vl *1000 
            self.updated = False

            return Wr, Wl

def main(args=None):
    rclpy.init(args=args)
    speed_control = control_p3dx()
    rate = speed_control.create_rate(20) # 20hz
    host = 'http://143.106.11.151:4950'
    rodas = '/motion/vel2'
    marta_nav.restest.restthru.http_init()
    vel2 = {}
    limits = {}
    limits['accel'] = 150
    limits['maxvel'] = 600
    status = marta_nav.restest.restthru.http_put('http://143.106.11.151:4950/motion/limits', limits)
    while rclpy.ok():
        if(speed_control.updated):
            Wr, Wl = speed_control.send_speed()
            vel2['right']= Wr
            vel2['left']= Wl

            status = marta_nav.restest.restthru.http_put('http://143.106.11.151:4950/motion/vel2', vel2)

        rclpy.spin_once(speed_control)

    speed_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
