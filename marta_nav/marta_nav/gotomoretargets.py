#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import json
import math

import threading
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import marta_nav.restest.restthru as restthru


restthru.http_init()

file_path = '/home/maria/demo/demo_cnpem/update/target.json'

def send_data(data, str):
    url = f"http://10.1.3.103:8000/taskactions{str}"
    print("oi")
    status = restthru.http_put(url, data)
    if status[0] == 200:
        print("Dados enviados com sucesso")
    else:
        print(f"Erro ao enviar dados: {status}")

class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.info('Navigating with ' + str(len(poses)) + ' goals.' + '...')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        
        msg = {
            'target': 'nothing',
            'status': 'success'
        }
        try:
            with open(file_path, 'w') as file:
                json.dump(msg, file)
        except IOError as e:
            print(f"Erro ao escrever no arquivo {file}: {e}")
        send_data("success", "/status")
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        #self._waitForNodeToActivate('amcl')
        #self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def main(args=None):
    rclpy.init()
    nav = BasicNavigator()
    i = 0
    # ...
    # init_pose = Pose()
    # init_pose.position.x = 0.83
    # init_pose.position.y = 3.72  
    # init_pose.position.z = 0.0
    # init_pose.orientation.x = 0.0
    # init_pose.orientation.y = 0.0
    # init_pose.orientation.z = 0.0
    # init_pose.orientation.w = 1.0
    #nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()
    status = 'wait'
    

    rate = nav.create_rate(1)
    new_path = False
    while rclpy.ok():
        print(file_path)
        try:            
            with open(file_path, "r") as archive:
                data = json.load(archive)
                status = data['status']
                target = data['target']
        except:
            print("Erro na leitura")   
        # print(target)

        if status == 'new':
            print(target.lower())
            new = False
            #if target.lower() == 'google_assistant' or target.lower() == 'tv':
            #    pose_target = PoseStamped()
            #    pose_target.header.frame_id = "odom"
            #    pose_target.pose.position.x = 0.83
            #    pose_target.pose.position.y = -1.75
            #    pose_target.pose.position.z = 0.0
            #    q = quaternion_from_euler(0.0, 0.0, math.pi)
            #    pose_target.pose.orientation.x = q[0]
            #    pose_target.pose.orientation.y = q[1]
            #    pose_target.pose.orientation.z = q[2]
            #    pose_target.pose.orientation.w = q[3]
            #    status = 'busy'
            #    nav.goToPose(pose_target)
            #    new = True
            if target.lower() == 'kinova': # or target.lower() == 'dispenser':
                data, status_get = restthru.http_get(f'http://10.1.3.103:8000/agents/kinova')
                data = json.loads(data)
                data1 = {}
                data1['id'] = data.get('id', {})
                data, status_get = restthru.http_get(f'http://10.1.3.103:8000/agents/{data1['id']}/pose')
                data = json.loads(data)
                data1['x'] = data.get('x',{})
                data1['y'] = data.get('y',{}) #+0.2

                data1['th'] = math.radians( data.get('th',[]))
                pose_target = PoseStamped()
                pose_target.header.frame_id = "odom"
                pose_target.pose.position.x =  data1['x'] #- 19.35
                pose_target.pose.position.y =  data1['y'] #- 0.2
                pose_target.pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, data1['th'])
                pose_target.pose.orientation.x = q[0]
                pose_target.pose.orientation.y = q[1]
                pose_target.pose.orientation.z = q[2]
                pose_target.pose.orientation.w = q[3]
                status = 'busy'
                nav.goToPose(pose_target)
                new = True
               
           
            if target.lower() == 'ervin':
                # print('oi')
                data, status_get = restthru.http_get(f'http://10.1.3.103:8000/objects/water_dispenser/in_front_pose')
                data1 = {}
                print(status_get)
                data = json.loads(data)
                data = data.get('in_front_pose', [])
                data1['x'] = data.get('x',{})
                data1['y'] = data.get('y',{}) #+0.2

                data1['th'] = math.radians( data.get('th',[]))
                pose_target = PoseStamped()
                pose_target.header.frame_id = "odom"
                pose_target.pose.position.x =  data1['x'] #- 19.35
                pose_target.pose.position.y =  data1['y'] #- 0.2
                pose_target.pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, data1['th'])
                pose_target.pose.orientation.x = q[0]
                pose_target.pose.orientation.y = q[1]
                pose_target.pose.orientation.z = q[2]
                pose_target.pose.orientation.w = q[3]
                status = 'busy'
                nav.goToPose(pose_target)
                new = True
                
            print(target.lower() == 'eric')
            if target.lower() == 'eric':
                # print('oi')
                data, status_get = restthru.http_get(f'http://10.1.3.103:8000/objects/sofa/in_front_pose')
                data1 = {}
                print(status_get)
                data = json.loads(data)
                data = data.get('in_front_pose', [])
                data1['x'] = data.get('x',{})
                data1['y'] = data.get('y',{}) #+0.2

                data1['th'] = math.radians( data.get('th',[]))
                pose_target = PoseStamped()
                pose_target.header.frame_id = "odom"
                pose_target.pose.position.x =  data1['x'] #- 19.35
                pose_target.pose.position.y =  data1['y'] #- 0.2
                pose_target.pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, data1['th'])
                pose_target.pose.orientation.x = q[0]
                pose_target.pose.orientation.y = q[1]
                pose_target.pose.orientation.z = q[2]
                pose_target.pose.orientation.w = q[3]
                status = 'busy'
                nav.goToPose(pose_target)
                new = True
            if target.lower() == 'living_room' or target.lower() == 'kitchen' or target.lower() == 'felipe':
                if target.lower() == 'felipe':
                    target = 'kitchen'
                data, status_get = restthru.http_get(f'http://10.1.3.103:8000/rooms/{target}/room_center')
                data = json.loads(data)
                data = data.get('room_center', [])

                room_center_x = data[0] #- 19.16
                room_center_y = data[1] #+  0.2
                room_center_th = 0,0
                pose_target = PoseStamped()
                pose_target.header.frame_id = "odom"
                pose_target.pose.position.x = room_center_x  
                pose_target.pose.position.y = room_center_y
                pose_target.pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, 0.0)
                pose_target.pose.orientation.x = q[0]
                pose_target.pose.orientation.y = q[1]
                pose_target.pose.orientation.z = q[2]
                pose_target.pose.orientation.w = q[3]
                nav.goToPose(pose_target)
                status = 'busy'
                new = True
            if new == False:
                data, status_get = restthru.http_get(f'http://10.1.3.103:8000/objects/{target}/in_front_pose')
                data1 = {}
                data = json.loads(data)
                data = data.get('in_front_pose', [])
                data1['x'] = data.get('x',{})
                data1['y'] = data.get('y',{}) 

                data1['th'] = math.radians( data.get('th',[]))
                pose_target = PoseStamped()
                pose_target.header.frame_id = "odom"
                pose_target.pose.position.x =  data1['x'] #- 19.35
                pose_target.pose.position.y =  data1['y'] #- 0.2
                pose_target.pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, data1['th'])
                pose_target.pose.orientation.x = q[0]
                pose_target.pose.orientation.y = q[1]
                pose_target.pose.orientation.z = q[2]
                pose_target.pose.orientation.w = q[3]
                status = 'busy'
                nav.goToPose(pose_target)
                
            while not nav.isNavComplete():
                feedback = nav.getFeedback()

                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        nav.cancelTask()

                    # Some navigation request change to demo preemption
                    #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    #    goal_pose.pose.position.x = -3.0
                    #     nav.goToPose(goal_pose)

            # ...

            result = nav.getResult()
            if result == TaskResult.SUCCEEDED:
                # Escrever a lista atualizada de volta no arquivo JSON
                print('Goal succeeded!')
                
            elif result == TaskResult.CANCELED:
                feedback = nav.getFeedback()
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                feedback = nav.getFeedback()
                print('Goal failed!')
            rclpy.spin_once(nav)

if __name__ == '__main__':
    main()
