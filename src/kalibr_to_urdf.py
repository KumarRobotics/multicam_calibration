#!/usr/bin/env python
#
# 2020 Bernd Pfrommer
#
# grabs extrinsic calibration from kalibr file and creates urdf snippets

import tf
import argparse
import yaml
import numpy as np

from tf.transformations import *

def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)

def matrix_to_tf(T):
    # returns 'sxyz' euler angles: roll(x), pitch(y), yaw(z)
    eul = euler_from_matrix(T)
    return {'origin': tuple(T[0:3, 3]), 'rpy': eul}

def make_transforms(args, kalibr):
    # frame_ids has one extra element at beginning
    frame_ids = [args.parent_frame] + args.frame_ids
    chain = []
    for cam_idx, cam in enumerate(args.cameras):
        k = kalibr[cam]
        tfn = {}
        tfn['parent'] = frame_ids[cam_idx]
        tfn['child'] = frame_ids[cam_idx + 1]
        if 'T_cn_cnm1' in k:
            T = np.linalg.inv(np.asarray(k['T_cn_cnm1']))
            tfn['T'] = matrix_to_tf(T)
        else:
            tfn['T'] = matrix_to_tf(np.asarray(
                [[1.0, 0, 0, 0], [0, 1, 0, 0],
                 [0,   0, 1, 0], [0, 0, 0, 1]]))
        chain.append(tfn)
    return chain

def write_joint(fh, parent, child, rpy, orig):
    fh.write('<link name="' + child + '"/>\n')
    fh.write('<joint name="' + parent + '_to_' + child + '" type="fixed">\n')
    fh.write('  <parent link="' + parent + '"/>\n')
    fh.write('  <child link="' + child + '"/>\n')
    fh.write('  <origin xyz="%.4f %.4f %.4f" rpy="%.4f %.4f %.4f"/>\n' % (orig + rpy))
    fh.write('  <axis xyz="0 0 1"/>\n')
    fh.write('</joint>\n')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='convert kalibr extrinsic transforms to urdf link file')
    parser.add_argument('--cameras', '-c', nargs='+', action='store', required=True,
                        help='list of cameras to publish: -c cam0 cam1')
    parser.add_argument('--frame_ids',  nargs='+', action='store', required=True,
                        help='corresponding frames: -f id_1 id_2')
    parser.add_argument('--parent_frame', '-p', action='store', required=True,
                        help='parent frame id')
    parser.add_argument('--input_file', '-f', action='store', required=True,
                        help='kalibr file with transform chain')
    parser.add_argument('--output_file', '-o', action='store', required=True,
                        help='name of urdf output file')

    args = parser.parse_args()

    kalibr = read_yaml(args.input_file)
    tflist = make_transforms(args, kalibr)
    with open(args.output_file, 'w') as fh:
        for t in tflist:
            write_joint(fh, t['parent'], t['child'],
                        t['T']['rpy'], t['T']['origin'])
        fh.close()
