#!/usr/bin/env python
#
# 2020 Bernd Pfrommer
#

# little script to grab the intrinsics from camera_info
#
#

from __future__ import print_function

import yaml
import argparse
import rospy
import re
import threading
import copy
import sys

from sensor_msgs.msg import CameraInfo
import std_srvs.srv

args = None

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
    if '/image_raw' in tp:
        return re.sub(r'/image_raw', '/camera_info', tp)
    elif '/image_rect_raw' in tp:
        return re.sub(r'/image_rect_raw', '/camera_info', tp)
    else:
        raise Exception('ERROR: cannot guess camerainfo topic for ' + tp)


class CameraInfoWaiter():
    def __init__(self, name, topic, max_time):
        self.name = name
        self.topic = topic
        self.max_time = max_time
        self.msg = None
    
    def callback(self, msg):
        #rospy.loginfo('got caminfo msg on topic ' + self.topic)
        #print(str(msg))
        self.msg = msg
        self.subscriber.unregister()
        self.event.set()

    def get_camerainfo(self):
        self.subscriber = rospy.Subscriber(self.topic, CameraInfo,
                                           self.callback)
        self.event = threading.Event()
        rospy.loginfo('%s waiting for %.2fs for topic %s' %
                      (self.name, self.max_time, self.topic))
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

def do_intrinsics_update():
    global args
    cinfo = read_yaml(args.template)
    cinfo_adj = {}
    status = True
    for cam in sorted(cinfo.keys()):
        c = cinfo[cam]
        topic = make_topic(cam, c)
        caminfoWaiter = CameraInfoWaiter(cam, topic, 3.0)
        ci = caminfoWaiter.get_camerainfo()
        if ci is not None:
            cinfo_adj[cam] = adjustIntrinsics(copy.deepcopy(c), ci)
        else:
            rospy.logerr("intrinsics not adjusted!!!")
            cinfo_adj[cam] = copy.deepcopy(c)
            status = False
    print("writing file to: ", args.output)
    write_yaml(cinfo_adj, args.output)
    return status
    
def service_callback(req):
    print("--------- updating intrinsics --------")
    status = do_intrinsics_update()
    return status, "intrinsics " + ("updated" if status else "NOT updated")


def main():
    parser = argparse.ArgumentParser(description='Subscribe to camerainfo messages to adjust camera intrinsics')
    global args
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('caminfo_waiter')
    args.template = rospy.get_param('~template')
    args.output = rospy.get_param('~output')
    args.offer_service = rospy.get_param('~offer_service')
    
    if args.offer_service:
        s = rospy.Service('update_intrinsics', std_srvs.srv.Trigger, service_callback)
        rospy.spin()
    else:
        return do_intrinsics_update()

if __name__=="__main__":
    status = main()
    if status:
        sys.exit(0)
    else:
        sys.exit(1)
