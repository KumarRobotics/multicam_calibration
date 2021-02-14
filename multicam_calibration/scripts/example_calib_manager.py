#!/usr/bin/env python3
#
# 2021 Bernd Pfrommer
#
# Use this script as template for calibration management
#

import rclpy
from rclpy.node import Node
import argparse
import std_srvs.srv
from multicam_calibration_srvs.srv import ParameterCmd
from std_srvs.srv import Trigger
import copy

FIX_INTRINSICS = 0
FIX_EXTRINSICS = 1
SET_ACTIVE     = 2

class ExampleCalibrationManager(Node):
#
# put all the calibration actions you want to do into this function
#
    set_calib_param = None
    run_calib = None
    

    def do_calibration(self, req, resp):
        print("--------- executing calibration sequence! --------")
        resp = Trigger.Response()
        resp.success = True
        resp.message = "calibration program finished!"
        try:
            # first do intrinsics of cam0
            self.set_p(FIX_INTRINSICS, "cam0", False)
            self.set_p(FIX_EXTRINSICS, "cam0", True)
            self.set_p(SET_ACTIVE,     "cam0", True)
            self.set_p(FIX_INTRINSICS, "cam1", True)
            self.set_p(FIX_EXTRINSICS, "cam1", True)
            self.set_p(SET_ACTIVE,     "cam1", False)
            self.run_cal()
            # then do intrinsics of cam1
            self.set_p(FIX_INTRINSICS, "cam0", True)
            self.set_p(FIX_EXTRINSICS, "cam0", True)
            self.set_p(SET_ACTIVE,     "cam0", False)
            self.set_p(FIX_INTRINSICS, "cam1", False)
            self.set_p(FIX_EXTRINSICS, "cam1", True)
            self.set_p(SET_ACTIVE,     "cam1", True)
            self.run_cal()
            # now extrinsics between the two
            self.set_p(FIX_INTRINSICS, "cam0", True)
            self.set_p(FIX_EXTRINSICS, "cam0", True)
            self.set_p(SET_ACTIVE,     "cam0", True)
            self.set_p(FIX_INTRINSICS, "cam1", True)
            self.set_p(FIX_EXTRINSICS, "cam1", False)
            self.set_p(SET_ACTIVE,     "cam1", True)
            self.run_cal()
        except (RuntimeError, Exception) as e:
            print("exception: " + str(e))
            resp.success = False
            resp.message = str(e)
        return resp

#
# ----------- no need to touch anything below
#
    def set_p(self, param, cam, val):
        print("setting %d for cam %s to %d" % (param, cam, val))
        self.req_set_param.param = param
        self.req_set_param.camera = cam
        self.req_set_param.value = val
        #self.cli_param.call_async(self.req_run))
        self.futures.append((self.cli_param, copy.deepcopy(self.req_set_param)))

    def run_cal(self):
        print("running calibration!")
        #self.futures.append(self.cli_run.call_async(self.req_run))
        self.futures.append((self.cli_run, copy.deepcopy(self.req_run)))
        #self.futures.append(self.req_run)

    def __init__(self):
        super().__init__('calibration_manager')
        self.futures = []
        self.req_set_param = ParameterCmd.Request()
        self.req_run = Trigger.Request()
        self.srv = self.create_service(Trigger,
                                       'run_calibration_manager',
                                       self.do_calibration)
        self.cli_param = self.create_client(ParameterCmd, 'set_parameter')
        self.cli_run  = self.create_client(Trigger, 'calibration')
        while not self.cli_param.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('param service not available, waiting again...')
        print('found calibration service!')
        while not self.cli_run.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('run service not available, waiting again...')
        print('found parameter set service!')
        print('---------------------\n now call run_calibration_manager')

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            while len(self.futures) != 0:
                print("executing: ", self.futures[0][1])
                my_future = self.futures[0][0].call_async(self.futures[0][1])
                while not my_future.done():
                    rclpy.spin_once(self)
                self.futures.pop(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--node", required=True, help="name of calibration node")
    args = parser.parse_args()

    rclpy.init(args=None)
    cm = ExampleCalibrationManager()
    cm.spin()

    cm.destroy_node()
    rclpy.shutdown()
