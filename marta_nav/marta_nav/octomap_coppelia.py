try:
    import marta_nav.sim as sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import marta_nav.readpoints as rp
import time
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from visualization_msgs.msg import MarkerArray
import marta_nav.restest.restthru as restthru


emptyBuff = bytearray()
#restthru.http_init()



#criar um objeto que irá carregar a informação do robô e a informação da nuvem de pontos
robox = 0.0
roboy = 0.0
roboth = 0.0

#Misturar leitura do sensor com pose do robô 


class threedselfping(Node):
    def __init__(self, id, gs): #Inicialização do selfa, a partir da posição inicial do robôs
        super().__init__('Octomap_to_coppelia')
        # Robot's real position on the grid
        self.url = 'http://10.1.3.101:8000/maps'
        self.points = np.array([])
        self.gridSize = gs
        self.clientId = id
        self.points_inertial = np.array([])
        self.robot_pos = [0, 0, 0]
        self.robot_yaw = 0  # in rad
        self.updated_sensor = False
        self.updated_pose = False
        self.positions = []
        self.positionsy = []
        self.positionsz = []
        self.new_occ = True
        self.new_free = True
        self.octree = []
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.processed_points = set()
        self.subscriber_occupied = self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.callback_occupied, 10)
        self.subscriber_free = self.create_subscription(MarkerArray, '/free_cells_vis_array', self.callback_free, 10)
        #self.subscriber_ = self.create_subscription(Image, '/depth', self.callback_depth, 10)
        #self.pose_robo = self.create_subscription(Odometry, "/pose", self.callback_pose, 10)
        self.occupancy_map = self.create_3d_occupancy_map(size=(400, 400, 10))
        self.timer = self.create_timer(0.1, self.update)

    #Function to create a 3D occupancy map
    def create_3d_occupancy_map(self, size=(150, 100, 10)):
        occupancy_map = np.zeros(size, dtype=int)
        print("Init")
        return occupancy_map
    
    def ray_tracing_add(self, measure):
        add = []
        #processed_points = set()
        x_index = int((measure[0] + (self.occupancy_map.shape[0]*self.gridSize)/2)//self.gridSize)
        y_index = int((measure[1] + (self.occupancy_map.shape[1]*self.gridSize)/2)//self.gridSize)
        z_index = int(measure[2]//self.gridSize)

        if 0 <= x_index < self.occupancy_map.shape[0] and \
        0 <= y_index < self.occupancy_map.shape[1] and \
        1 <= z_index < self.occupancy_map.shape[2]:

            if self.occupancy_map[x_index, y_index, z_index] != 1:
                add += [measure[0], measure[1], measure[2]]
                self.occupancy_map[x_index, y_index, z_index] = 1

        return add
    
    def ray_tracing_free(self, measure):
        remove = []
        #processed_points = set()
        x_index = int((measure[0] + (self.occupancy_map.shape[0]*self.gridSize)/2)//self.gridSize)
        y_index = int((measure[1] + (self.occupancy_map.shape[1]*self.gridSize)/2)//self.gridSize)
        z_index = int(measure[2]//self.gridSize)
        
        if 0 <= x_index < self.occupancy_map.shape[0] and \
        0 <= y_index < self.occupancy_map.shape[1] and \
        1 <= z_index < self.occupancy_map.shape[2]:

            if self.occupancy_map[x_index, y_index, z_index] != 0:
                remove += [measure[0], measure[1], measure[2]]
                self.occupancy_map[x_index, y_index, z_index] = 0

        return remove
    # def ray_tracing(self, measure):
    #     remove = []
    #     add = []

    #     for i in range(int(self.robot_pos[0]//self.gridSize), int(measure[0]//self.gridSize)):
    #         x_index = int(measure[0]//self.gridSize)
    #         y_index = int(measure[1]//self.gridSize)
    #         z_index = int(measure[2]//self.gridSize)

    #         if 0 <= i < self.occupancy_map.shape[2] and \
    #         0 <= y_index < self.occupancy_map.shape[1] and \
    #         0 <= z_index < self.occupancy_map.shape[0] and \
    #         self.occupancy_map[x_index, y_index, z_index] != 0:
    #             remove += [i, measure[1], measure[2]]
    #             self.occupancy_map[i, y_index, z_index] = 0

    #     if 0 <= x_index < self.occupancy_map.shape[0] and \
    #     0 <= y_index < self.occupancy_map.shape[1] and \
    #     0 <= z_index < self.occupancy_map.shape[2] and \
    #     self.occupancy_map[x_index, y_index, z_index] != 1:
    #         self.occupancy_map[x_index, y_index, z_index] = 1
    #         add += [measure[0], measure[1], measure[2]]

    #     return remove, add
    
    def callback_occupied(self, msg):
        positions_add = []
        print("Recebeu")
        self.processed_points = set()
        
        for marker in msg.markers:
            #positions += [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            # for node in marker.points:
            #     self.processed_points.add(tuple([node.x, node.y, node.z]))
            #print((marker.points))
            for node in marker.points:
                add = self.ray_tracing_add([node.x, node.y, node.z])
                positions_add += add
                
        #exit()
        
        
        if len(positions_add) > 0:
            url = self.url + '/add'
            print(positions_add)
            status = restthru.http_put(url, positions_add)
            #self.addVoxels(positions_add)
        #self.removeVoxels(positions_remove)
            #print(len(marker.points))
        #self.getVoxel()
        #exit()
            
        
    
    def callback_free(self, msg):
        positions_remove = []
        # if self.new_free:
        #     self.new_free = False
        #print(len(msg.markers))
        for marker in msg.markers:
            #positions += [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            for node in marker.points:
                remove = self.ray_tracing_free([node.x, node.y, node.z])
                positions_remove += remove
                #print(len(marker.points))
        if len(positions_remove) > 0:
            url = self.url + '/remove'
            print(positions_remove)
            status = restthru.http_put(url, positions_remove)
            #self.removeVoxels(positions_remove)
            
            
        
    def update(self):
        self.new_occ = True
        self.new_free = True

    def addVoxel(self, pos,rgb=[],ref='absolute'):
        sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'addVoxel',rgb,pos,[ref],emptyBuff,sim.simx_opmode_oneshot)
    
    def getVoxel(self):
        self.octree = sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'getVoxel', [], [], [] ,emptyBuff,sim.simx_opmode_oneshot_wait)
        #print(self.octree)
        #exit()

    def addVoxels(self, pos,rgb=[],ref='absolute'):
        sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'addVoxels',rgb,pos,[ref],emptyBuff,sim.simx_opmode_oneshot_wait)

    def removeVoxels(self, listCoord):
        sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'removeVoxels',[],listCoord,[],emptyBuff,sim.simx_opmode_oneshot_wait)


def setOctreeParams(minHeight,maxHeight,gridSize, clientID):
    sim.simxCallScriptFunction(clientID,'voxelFunctions',sim.sim_scripttype_childscript,'setOctreeParams',[],[minHeight,maxHeight,gridSize],[],emptyBuff,sim.simx_opmode_oneshot)




def main():    
    print ('Program started')
    
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

    #rospy.spin()
    if clientID!=-1:
        print ('Connected to remote API server')
        print("clientID",clientID)
        #some variables
        # Instead of using lists, use sets to store unique voxel positions

        colors = []
        minHeight = 0
        minX = 0
        minY = 0
        maxX = 2
        maxY = 2
        maxHeight = 2
        gridSize = 0.2 # also update manually in the octree object windows "voxel size"
                        # in the future it will be done automatically
        #set the max and min height for the calculation of the gradient
        setOctreeParams(minHeight,maxHeight,gridSize, clientID)
        rclpy.init()
        node = threedselfping(clientID, gridSize)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


if __name__ == '__main__':
    main()