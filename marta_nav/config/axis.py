#!/usr/bin/env python3
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#

import threading
import urllib.request, urllib.error, urllib.parse
import requests, requests.auth
import datetime
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Bool


class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.logger = self.axis.node.get_logger()
        self.daemon = True
        self.timeoutSeconds = 2.5
        self.rate = self.axis.node.create_rate(2)

    def run(self):
        while(True):
            self.stream()

    def stream(self):
        while(True):
            self.formURL()
            self.authenticate()
            if self.openURL():
                self.publishFramesContinuously()
            self.rate.sleep(2) # if stream stays intact we shouldn't get to this

    def formURL(self):
        self.url = 'http://%s/axis-cgi/mjpg/video.cgi?resolution=2CIFEXP' % self.axis.hostname
        self.url += "?fps=%d&resolution=%dx%d" % (self.axis.fps, self.axis.width,
                                                            self.axis.height)

        # support for Axis F34 multicamera switch
        if (self.axis.camera != 0):
            self.url += "&camera=%s" % str(self.axis.camera)

        self.logger.debug('opening ' + str(self.axis))

    def authenticate(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            top_level_url = "http://" + self.axis.hostname
            password_mgr.add_password(None, top_level_url, self.axis.username,
                                                            self.axis.password)
            if self.axis.use_encrypted_password:
                handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib.request.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib.request.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib.request.install_opener(opener)

    def openURL(self):
        '''Open connection to Axis camera using http'''
        try:
            self.fp = urllib.request.urlopen(self.url, timeout=self.timeoutSeconds)
            return(True)
        except urllib.error.URLError as e:
            self.logger.warn('Error opening URL %s' % (self.url) +
                            'Possible timeout.  Looping until camera appears')
            return(False)

    def publishFramesContinuously(self):
        '''Continuous loop to publish images'''
        while(True):
            try:
                self.findBoundary()
                self.getImage()
                self.publishMsg()
                self.publishCameraInfoMsg()

            except:
                self.logger.warn('Timed out while trying to get message.')
                break

    def findBoundary(self):
        '''The string "--myboundary" is used to denote the start of an image in
        Axis cameras'''
        while(True):
            boundary = self.fp.readline()
            if boundary == b'--myboundary\r\n':
                break

    def getImage(self):
        '''Get the image header and image itself'''
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = {}
        while(True):
            line = self.fp.readline()
            if line == b'\r\n':
                break
            line = line.strip()
            parts = line.split(b": ", 1)
            try:
                self.header[parts[0]] = parts[1]
            except:
                self.logger.warn('Problem encountered with image header.  Setting '
                                                    'content_length to zero')
                self.header[b'Content-Length'] = 0 # set content_length to zero if
                                            # there is a problem reading header
        self.content_length = int(self.header[b'Content-Length'])

    def getImageData(self):
        '''Get the binary image data itself (ie. without header)'''
        if self.content_length>0:
            self.img = self.fp.read(self.content_length)
            self.fp.readline() # Read terminating \r\n and do nothing with it

    def publishMsg(self):
        '''Publish jpeg image as a ROS message'''
        self.msg = CompressedImage()
        self.msg.header.stamp = self.axis.node.get_clock().now().to_msg()
        self.msg.header.frame_id = self.axis.frame_id
        self.msg.format = "jpeg"
        self.msg.data = self.img
        self.axis.pub.publish(self.msg)

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
        cimsg = self.axis.cinfo.getCameraInfo()
        cimsg.header.stamp = self.msg.header.stamp
        cimsg.header.frame_id = self.axis.frame_id
        cimsg.width = self.axis.width
        cimsg.height = self.axis.height
        self.axis.caminfo_pub.publish(cimsg)

class Axis:
    def __init__(self, node, hostname, username, password, width, height, fps, frame_id,
                 camera_info_url, use_encrypted_password, camera, ir, defog, wiper):
        self.node = node
        self.hostname = hostname
        self.username = username
        self.password = password
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        self.use_encrypted_password = use_encrypted_password
        self.camera = camera

        self.http_headers = {
            'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36',
            'From': f'http://{self.hostname}'
        }
        if self.use_encrypted_password:
            self.http_auth = requests.auth.HTTPDigestAuth(self.username, self.password)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(self.username, self.password)
        self.http_timeout = (3, 5)

        # generate a valid camera name based on the hostname
        self.cname = "axis_lab"#camera_info_manager.genCameraName(self.hostname)
        self.cinfo = CameraInfo.CameraInfoManager(cname = self.cname,
                                                   url = self.camera_info_url)
        self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.st = None
        self.pub = self.node.create_publisher("image_raw/compressed", CompressedImage, self, queue_size=1)
        self.caminfo_pub = self.node.create_publisher("camera_info", CameraInfo, self, queue_size=1)

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        '''Lazy-start the image-publisher.'''
        if self.st is None:
            self.st = StreamThread(self)
            self.st.start()

def main():
    rclpy.init()
    node = rclpy.create_node("axis_driver")

    arg_defaults = {
        'hostname': '10.1.3.143:8080',       # default IP address
        'username': 'root',               # default login name
        'password': 'root',
        'width': 640,
        'height': 480,
        'fps': 0,                         # frames per second (0 = camera default)
        'frame_id': 'axis_camera',
        'camera_info_url': 'package://axis_camera/calibaxis.yaml',
        'use_encrypted_password' : False,
        'camera' : 0,
        'ir': False,
        'defog': False,
        'wiper': False }
    args = updateArgs(node, arg_defaults)
    Axis(node, **args)
    rclpy.spin()

def get_parameter(node, param_name, default_value=None):
    param_value = node.get_parameter(param_name)
    if param_value:
        return param_value.value
    else:
        return default_value

def updateArgs(node, arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver.'''
    args = {}
    for name, val in arg_defaults.items():
        full_name = get_parameter(node, name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = node.get_parameter(full_name) 
        
    # resolve frame_id with tf_prefix (unless already absolute)
    if args['frame_id'][0] != '/':        # not absolute?
        tf_prefix = get_parameter(node,'tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:           # prefix defined?
            prefix_val = node.get_parameter(tf_prefix)
            if prefix_val[0] != '/':          # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return(args)

if __name__ == "__main__":
    main()