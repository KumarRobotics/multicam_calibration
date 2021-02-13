#!/usr/bin/python2
#
# 2018 Bernd Pfrommer
#
# Use this script as template for calibration management
#

import rospy
import roslib
import argparse
import std_srvs.srv
from multicam_calibration.srv import *

FIX_INTRINSICS = 0
FIX_EXTRINSICS = 1
SET_ACTIVE     = 2

#
# put all the calibration actions you want to do into this function
#

def do_calibration(req):
    print "--------- executing calibration sequence! --------"
    try:
        # first do intrinsics of cam0
        set_p(FIX_INTRINSICS, "cam0", False)
        set_p(FIX_EXTRINSICS, "cam0", True)
        set_p(SET_ACTIVE,     "cam0", True)
        set_p(FIX_INTRINSICS, "cam1", True)
        set_p(FIX_EXTRINSICS, "cam1", True)
        set_p(SET_ACTIVE,     "cam1", False)
        run_cal()
        # then do intrinsics of cam1
        set_p(FIX_INTRINSICS, "cam0", True)
        set_p(FIX_EXTRINSICS, "cam0", True)
        set_p(SET_ACTIVE,     "cam0", False)
        set_p(FIX_INTRINSICS, "cam1", False)
        set_p(FIX_EXTRINSICS, "cam1", True)
        set_p(SET_ACTIVE,     "cam1", True)
        run_cal()
        # now extrinsics between the two
        set_p(FIX_INTRINSICS, "cam0", True)
        set_p(FIX_EXTRINSICS, "cam0", True)
        set_p(SET_ACTIVE,     "cam0", True)
        set_p(FIX_INTRINSICS, "cam1", True)
        set_p(FIX_EXTRINSICS, "cam1", False)
        set_p(SET_ACTIVE,     "cam1", True)
        run_cal()
    except (RuntimeError, Exception),e:
        print "exception: " + str(e)
        return False, str(e)
    return True, "calibration program finished!"

#
# ----------- no need to touch anything below
#

set_calib_param = None
run_calib = None

def set_p(param, cam, val):
    print "setting %d for cam %s to %d" % (param, cam, val)
    resp = set_calib_param(param, cam, val)
    if not resp.success:
        raise RuntimeError('cannot set param: %s' % resp.msg)

def run_cal():
    print "running calibration!"
    resp = run_calib()
    if not resp.success:
        raise RuntimeError('calib failed: %s' % resp.message)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--node", required=True, help="name of calibration node")
    args = parser.parse_args()

    rospy.init_node('example_calib_manager')
    print "waiting for calibration service..."
    rospy.wait_for_service(args.node + '/calibration')
    print "waiting for calibration parameter service..."
    rospy.wait_for_service(args.node + '/set_parameter')
    run_calib = rospy.ServiceProxy(args.node + '/calibration', std_srvs.srv.Trigger)
    set_calib_param = rospy.ServiceProxy(args.node + '/set_parameter', ParameterCmd)
    print "found calibration services!"

    s = rospy.Service('calib_manager', std_srvs.srv.Trigger, do_calibration)
    print "calibration manager running ..."
    rospy.spin()
