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

import numpy as np
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


emptyBuff = bytearray()



#criar um objeto que irá carregar a informação do robô e a informação da nuvem de pontos
robox = 0.0
roboy = 0.0
roboth = 0.0

#Misturar leitura do sensor com pose do robô 


class threedselfping(Node):
    def __init__(self, id, gs): #Inicialização do selfa, a partir da posição inicial do robôs
        super().__init__('Thread_3d_reconstruction')
        # Robot's real position on the grid
        self.points = np.array([])
        self.gridSize = gs
        self.clientId = id
        self.points_inertial = np.array([])
        self.robot_pos = [0, 0, 0]
        self.robot_yaw = 0  # in rad
        self.updated_sensor = False
        self.updated_pose = False
        self.positionsx = []
        self.positionsy = []
        self.positionsz = []
        self.new = True

        self.o3d_pcd = o3d.geometry.PointCloud()
        self.subscriber_ = self.create_subscription(PointCloud2, '/cloud', self.callback_pointcloud, 10)
        #self.subscriber_ = self.create_subscription(Image, '/depth', self.callback_depth, 10)
        self.pose_robo = self.create_subscription(Odometry, "/pose", self.callback_pose, 10)
        self.timer = self.create_timer(0.1, self.update)

    def callback_depth(self, msg):
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        # Get depth value at position (50, 50)
        depth_image_float32 = depth_image.astype(np.float32)
        measure = depth_image_float32[50, 50]*depth_image_float32[50, 50]*0.0000387265 + depth_image_float32[50, 50]*0.0000495879 - 0.004827465
        # Get depth value at position (50, 50)
       

        # Multiply the depth value by itself
        #squared_depth_value = depth_value * depth_value
        print(f"Measure value at (50, 50): {measure} cm")
        self.count = 0
        self.sum = 0



    def callback_pointcloud(self, data):
        
        if self.updated_sensor == False:
            pointstempx = []
            pointstempy = []
            pointstempz = []
            cloud_points = []
            for p in rp.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
                print(f"pppppppppppppppppppppppppppppppp{p}")
                pointstempx.append(p[0]*1)
                pointstempy.append(p[1]*1)
                pointstempz.append(p[2]*1)
                #exit()
            #exit()
            self.points_inertial = np.zeros((len(pointstempx), 3))
            self.points = np.zeros((len(pointstempx), 3))
            self.points[:,0] = pointstempx
            self.points[:,1] = pointstempy
            self.points[:,2] = pointstempz

                    #self.points_inertial = np.zeros((self.points.shape[0], 3))
            r = R.from_rotvec(np.pi/2 * np.array([0, -1, 0]))
            r2 = R.from_rotvec(np.pi/2 * np.array([-1, 0, 0]))
            print(f"Rotation: {r2.as_matrix()}")
            for i in range(0, 10000):
                #print(np.transpose(self.points_inertial[i,:]))
                self.points[i,:] = np.dot(r.as_matrix(), np.transpose(self.points[i,:]))
                self.points[i,:] = np.dot(r2.as_matrix(), np.transpose(self.points[i,:]))
            #exit()
            self.points_inertial[:,0] = math.cos(self.robot_yaw)*self.points[:,0] - math.sin(self.robot_yaw)*self.points[:,1] + self.robot_pos[0]
            self.points_inertial[:,1] = math.cos(self.robot_yaw)*self.points[:,1] + math.sin(self.robot_yaw)*self.points[:,0] + self.robot_pos[1]
            self.points_inertial[:,2] = self.points[:,2] + self.robot_pos[2]
            

            #print(len(pointstempx))
                            #print (" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
            #print(len(cloud_points))

            self.updated_sensor = True
        
        #p = pcl.PointCloud(np.array(points, dtype=np.float32))

    def callback_pose(self, data):
        self.robot_pos[0] = data.pose.pose.position.x
        self.robot_pos[1] = data.pose.pose.position.y
        self.robot_pos[2] = data.pose.pose.orientation.z

    def lidar_to_inertial_frame(self):
        
        #self.points_inertial = np.zeros((self.points.shape[0], 3))
        self.points_inertial[:,0] = math.cos(self.robot_pos[2])*self.points[:,0] - math.sin(self.robot_pos[2])*self.points[:,1] + self.robot_pos[0]
        self.points_inertial[:,1] = math.cos(self.robot_pos[2])*self.points[:,1] + math.sin(self.robot_pos[2])*self.points[:,0] + self.robot_pos[1]
        self.points_inertial[:,2] = self.points[:,2] + self.robot_pos[2]

    def update(self):
        if self.updated_sensor == True:
        #self.points_inertial = np.zeros((self.points.shape[0], 3))
        #self.lidar_to_inertial_frame()
        # Inside the loop where you add positions
            for j in range(self.points_inertial.shape[0]-1):

                y = float(self.points_inertial[j, 0]) #// gridSize
                x = float(self.points_inertial[j, 1]) #// gridSize
                z = float(self.points_inertial[j, 2]) #// gridSize
                    # Check if the position is already in the sets
                # if z not in positionsz:                        
                #     if y not in positionsx:
                #         if x not in positionsy:
                #             positionsz.append(x)
                #             positionsx.append(z)
                #             positionsy.append(y)
                
                
            # Perform division on the lists
            #positions = [x / 2 for x in positionsx], [y / 2 for y in positionsy], [z / 2 for z in positionsz]     
            x_inicial = self.robot_pos[0]//self.gridSize
            y_inicial = self.robot_pos[1]//self.gridSize         
            for j in range(0, self.points_inertial.shape[0]):
                #print(j)
                self.new = True
                for i in range(len(self.positionsx)):
                    if self.positionsx[i] == self.points_inertial[j,0]//self.gridSize:
                        if self.positionsy[i] == self.points_inertial[j,1]//self.gridSize:
                            if self.positionsz[i] == self.points_inertial[j,2]//self.gridSize:
                                self.new = False
                                break
                if self.new:
                    if (math.sqrt(math.pow(self.points_inertial[j, 0] - self.robot_pos[0],2) + math.pow(self.points_inertial[j, 1] - self.robot_pos[1],2) + math.pow((self.points_inertial[j, 2]-self.robot_pos[2]),2))) <= 1.5:
                        self.addVoxel([self.points_inertial[j, 0], self.points_inertial[j, 1],self.points_inertial[j, 2]+1.0])
                        self.positionsx.append(self.points_inertial[j,0]//self.gridSize)
                        self.positionsy.append(self.points_inertial[j,1]//self.gridSize)
                        self.positionsz.append(self.points_inertial[j,2]//self.gridSize)
            #positions = [positionsx/2, positionsy/2, positionsz/2]
            #print
            self.points=np.zeros([])
            self.points_inertial=np.zeros([])
            self.updated_sensor = False
    def addVoxel(self, pos,rgb=[],ref='absolute'):
        sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'addVoxel',rgb,pos,[ref],emptyBuff,sim.simx_opmode_oneshot)

    def addVoxels(self, pos,rgb=[],ref='absolute'):
        sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'addVoxels',rgb,pos,[ref],emptyBuff,sim.simx_opmode_oneshot)

    def removeVoxels(self, listCoord):
        sim.simxCallScriptFunction(self.clientId,'voxelFunctions',sim.sim_scripttype_childscript,'removeVoxels',[],listCoord,[],emptyBuff,sim.simx_opmode_oneshot)


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
        gridSize = 0.01 # also update manually in the octree object windows "voxel size"
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