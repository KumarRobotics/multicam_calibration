#!/usr/bin/python2
#
# 2018 Bernd Pfrommer
#
# Script to generate test corners (world, image coordinates), given a camera
# calibration file in kalibr format.
#

from tf.transformations import *
import math
import yaml
import argparse
import cv2
import numpy as np

#
# 
#
def make_grid():
    """ make_grid()
    generates grid in world coordinates"""
    n     = 20   # number of grid points
    nhalf = n/2
    L     = 2.0  # length in meters
    d     = L/n   # grid spacing
    xp = []
    for iy in range(0, n):
        for ix in range(0, n):
            w  = np.array([(ix - nhalf) * d, (iy - nhalf) * d, 0])
            xp.append(w)
    return np.array(xp)

def make_frame(of, camid, fnum, Rbase, worldPoints, K, dist_model,
               dist, res, theta, phi, d):
    """generates a grid of world and points from a given vantage point"""
    # cook up additional rotation R
    Ra = euler_matrix(theta, phi, 0, 'rxyz')
    R  = np.matmul(Ra, Rbase)
    # go back to angle/axis representation
    angle, axis, pt = rotation_from_matrix(R)
    # rotation vector
    rvec   = angle * axis
    # translation vector
    tvec   = np.array([0.0, 0.0, d])
    # project rotated/translated world points
    if dist_model == "radtan" or dist_model == "plumb_bob":
        im, jac = cv2.projectPoints(worldPoints, rvec, tvec, K, dist)
        imgPoints = np.array(im)[:,0,:]
    elif dist_model == "equidistant":
        imgPoints = project_points_fisheye(worldPoints, rvec, tvec, K, dist)
    for i in range(0, worldPoints.shape[0]):
        uv = imgPoints[i,:]
        w  = worldPoints[i,:]
        if uv[0] > 0 and uv[0] < res[0] and uv[1] > 0 and uv[1] < res[1]:
            of.write('%d, intrinsics, %d, %f, %f, %f, %f\n' %
                     (fnum, camid, w[0], w[1], uv[0], uv[1]))

def project_points_fisheye(wp, rvec, tvec, K, dist):
    """ project_points_fisheye(wp, rvec, tvec, K, dist)
    replacement for missing opencv function
    """
    angle = np.linalg.norm(rvec)
    nvec  = np.array([0,0,1]) if abs(angle) < 1e-8 else rvec/angle
    R = rotation_matrix(angle, nvec)[0:3, 0:3]
    im = []
    for w in wp:
        X = np.matmul(R, w) + tvec
        x = X/X[2]
        r = math.sqrt(x[0] * x[0] + x[1] * x[1])
        theta   = math.atan(r)
        theta2  = theta * theta
        theta_d = theta
        theta_pow = theta * theta2
        for k in dist:
            theta_d += k * theta_pow
            theta_pow *= theta2
        fac = (theta_d/r) if (r > 1e-8) else 1.0
        xp = np.array([x[0] * fac, x[1] * fac, 1.0])
        uv = np.matmul(K, xp)
        im.append([uv[0], uv[1]])
    return np.array(im)

def make_points(ofname, intr, dist_model, dist_coeffs, res):
    K     = np.array([[intr[0], 0.0,     intr[2]],
                      [0.0,     intr[1], intr[3]],
                      [0.0,     0.0,     1.0]])
    dist  = np.array(dist_coeffs)
    #
    # does the base transform: rotates the camera
    # so it faces the point grid
    Rbase = np.array([[1.0,  0.0,  0.0, 0.0],
                      [0.0, -1.0,  0.0, 0.0],
                      [0.0,  0.0, -1.0, 0.0],
                      [0.0,  0.0,  0.0, 1.0]])
    wp    = make_grid() # world points
    fnum  = 0
    with open(ofname, 'w') as of:
        for d in np.arange(1.0, 10.0, 1): # distance to the point grid
            for theta in np.arange(-0.1, 0.1, 0.02): # euler angle
                for phi in np.arange(-0.1, 0.1, 0.02): # euler angle
                    make_frame(of, 0, fnum, Rbase, wp, K, dist_model, dist,
                               res, theta, phi, d)
                    fnum += 1
    print "wrote %d frames to %s" % (fnum, ofname)


def get_param(cam, name):
    if not cam[name]:
        raise ValueError('cannot find field \'%s\' for camera!' % name)
    return cam[name]

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='make test corners.')
    parser.add_argument("-o", "--output", default="corners.csv", help="name of output corner file")
    parser.add_argument("-c", "--camera", default="cam0", help="name of camera to use")
    parser.add_argument("-f", "--calib", required=True, help="name of calibration file")

    args   = parser.parse_args()
    cname  = args.camera
    with open(args.calib, 'r') as stream:
        try:
            params = yaml.load(stream)
            if not params[cname]:
                raise ValueError('cannot find camera % in calib file!' % cname)
            pcam = params[cname]
            make_points(args.output,
                        get_param(pcam,'intrinsics'),
                        get_param(pcam,'distortion_model'),
                        get_param(pcam,'distortion_coeffs'),
                        get_param(pcam,'resolution'))
        except yaml.YAMLError as exc:
            print(exc)
        except ValueError as exc:
            print(exc)
