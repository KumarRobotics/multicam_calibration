#!/usr/bin/env python
#
# 2020 Bernd Pfrommer
#

#
#
#

from __future__ import print_function

import yaml
import argparse
import rospy
import re
import threading
import copy

from sensor_msgs.msg import CameraInfo

def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)

def write_yaml(ydict, fname):
    with open(fname, 'w') as f:
        try:
            yaml.dump(ydict, f)
        except yaml.YAMLError as y:
            print("Error:", y)

def make_topic(name, c):
    if 'rostopic_camerainfo' in c:
        return c['rostopic_camerainfo']
    if 'rostopic' not in c:
        print('ERROR: no rostopic found for camera: ', name)
        raise Excepton('ERROR: no rostopic found for camera: ' + name)
    tp = c['rostopic']
    return re.sub(r'(?is)/image_raw.+', '/camera_info', tp)


class CameraInfoWaiter():
    def __init__(self, name, topic, max_time):
        self.name = name
        self.topic = topic
        self.max_time = max_time
        self.msg = None
    
    def callback(self, msg):
        rospy.loginfo('got caminfo msg on topic ' + self.topic)
        #print(str(msg))
        self.msg = msg
        self.subscriber.unregister()
        self.event.set()

    def get_camerainfo(self):
        self.subscriber = rospy.Subscriber(self.topic, CameraInfo,
                                           self.callback)
        self.event = threading.Event()
        rospy.loginfo('waiting for max %fs for camerainfo event' %
                      self.max_time)
        self.event.wait(self.max_time)
        self.subscriber.unregister()
        return self.msg  # will return none if nothing comes in


def adjustIntrinsics(c, ci):
    c['intrinsics'][0] = ci.K[0]
    c['intrinsics'][1] = ci.K[4]
    c['intrinsics'][2] = ci.K[2]
    c['intrinsics'][3] = ci.K[5]
    c['resolution'][0] = ci.width
    c['resolution'][1] = ci.height

    dist_model = c['distortion_model']
    if (dist_model == 'equidistant' or dist_model == 'equidist' or
        dist_model == 'fisheye'):
        c['distortion_coeffs'][0:4] = ci.D[0:4]
    elif dist_model == 'radtan' or dist_model == 'plumb_bob':
        c['distortion_coeffs'][:] = ci.D[:]
    else:
        print('ERROR: invalid dist model: ', dist_model)
        raise Exception('ERROR: invalid dist model: ' + dist_model)
    return c
    
def main():
    parser = argparse.ArgumentParser(description='Subscribe to camerainfo messages to adjust camera intrinsics')
    parser.add_argument("-t", "--template", required=True, help="template input file in Kalibr format")
    parser.add_argument("-o", "--output", required=True, help="output file with adjusted intrinsics")
    args = parser.parse_args()
    
    rospy.init_node('caminfo_waiter')

    cinfo = read_yaml(args.template)
    cinfo_adj = {}
    for cam in sorted(cinfo.keys()):
        c = cinfo[cam]
        print('camera: ', cam)
        topic = make_topic(cam, c)
        print('checking topic: ', topic)
        caminfoWaiter = CameraInfoWaiter(cam, topic, 3.0)
        ci = caminfoWaiter.get_camerainfo()
        if ci is not None:
            cinfo_adj[cam] = adjustIntrinsics(copy.deepcopy(c), ci)
        else:
            print("WARNING WARNING WARNING: intrinsics not adjusted!!!")
            cinfo_adj[cam] = copy.deepcopy(c)

    write_yaml(cinfo_adj, args.output)

if __name__=="__main__":
    main()    
