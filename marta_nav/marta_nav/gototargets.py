#! /usr/bin/env python3
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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration



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



file_path = "/home/cesar/Documents/Doutorado/catkin_ws2/src/restest/scripts/ervin/agents.json"

salas = {
    "meeting": [0.45, 0.83],
    "corredor": [1.06, 1.28],
    "a1": [0.4, 1.8],
    "demo": [1.58, 0.98],
    "lab": [1.61, 1.46],
    "cafe": [2.12, 0.7]
}

transition = {
    "a1": {"corredor": [0.45, 0.83]},
    "corredor": {"demo": [2.45, 5]},
    "demo": {"cafe": [2, 3]},
}

class take_target_and_move():

    def __init__(self):
        rclpy.init("sendo goal")
        #restest.restthru.http_init()
        pose = "/motion/pose"
        limits = {}
        limits['accel'] = 150
        limits['maxvel'] = 150
        #status = restest.restthru.http_put('http://10.1.3.130:4950/motion/limits', limits)
        self.pose_seq = list()
        self.goal_cnt = 0
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Esperando pelo navstack")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return  
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.json_targetlist()

    def json_targetlist(self):
        #pub = rospy.Publisher('/locais', PoseArray, queue_size=10)
        rate = rospy.Rate(10) #10hz
        initial_pose = {}
        new_goal = False
        while not rospy.is_shutdown():
            with open(file_path, "r+") as archive:
                    # Escreva os data no archive usando o método json.dump()
                        data = json.load(archive)
                        agents = data.get('agents', [])
                        for agent in agents:
                            if agent.get('id') == 111:
                                if agent['new_path']:
                                    print(agent['new_path'])
                                    target = agent['Target_List']
                                    current = agent['current_room']
                                    #position = agent['pose']
                                    new_goal = True
                                    rospy.loginfo("New Path received")
            point_seq = list()
            pose_seq =  list()
            if new_goal:
                new_goal = False
                #update json to not resend the same path
                for agent in agents:
                    if agent.get('id') == 111:
                        agent['new_path'] = False
                # Write the updated data back to the same file
                with open(file_path, "w") as archive:
                    json.dump(data, archive, indent=4)  # Use 'indent' for pretty-printing if desired
                ###################################
                for i in range(len(target) - 1):
                    point_seq.append([salas[target[i]][0], salas[target[i]][1],1])
                    point_seq.append([transition[target[i]][target[i+1]][0], transition[target[i]][target[i+1]][1],1])
                point_seq.append([salas[target[i]][0], salas[target[i]][1],1])
                quat_seq = Quaternion(*(quaternion_from_euler(0, 0, 0, axes='sxyz')))
                for point in point_seq:
                    self.pose_seq.append(Pose(Point(*point),quat_seq))
                self.movebase_client()
            rospy.sleep(0.1)


def main(args=None):
    
    nav = BasicNavigator()
    i = 0
    # ...
    init_pose = Pose()
    init_pose.position.x = 0.8
    init_pose.position.y = 1.11  
    init_pose.position.z = 0.0
    init_pose.orientation.x = 0.0
    init_pose.orientation.y = 0.0
    init_pose.orientation.z = 0.0
    init_pose.orientation.w = 1.0
    #nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

    # ...
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.04
    goal_pose.pose.position.y = 4.01  
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    #path = nav.getPath(init_pose, goal_pose)
    #smoothed_path = nav.smoothPath(path)

    # ...

    nav.goToPose(goal_pose)
    while not nav.isNavComplete():
        i = i + 1
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
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

if __name__ == '__main__':
    main()
