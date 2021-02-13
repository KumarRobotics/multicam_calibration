# multicam_calibration - extrinsic and intrinsic calbration of cameras

This repo contains a ROS2 package for intrinsic and extrinsic
calibration of multi-camera systems using a grid of apriltags (checkerboard is not supported).

Supports fisheye (``equidistant``) and standard ``radtan`` distortion models.

Tested on Ubuntu 20.04 with ROS2 Foxy.

## Installation
You will need libceres:
```
	sudo apt install libceres-dev
```

Download the package:
```	
mkdir -p my_ros_ws/src
cd my_ros_ws/src
git clone --branch ros2	https://github.com/KumarRobotics/multicam_calibration.git
```
You will also need the [ROS2 version of the apriltag ROS2 wrapper](https://github.com/versatran01/apriltag/tree/ros2). Follow the above link for instructions, but it should be something like:
```
git clone --branch ros2 https://github.com/berndpfrommer/apriltag.git
cd ..
wstool init src src/apriltag/apriltag_umich/apriltag_umich.rosinstall 
```
Build (after sourcing your ROS2 environment):
```
cd my_rows_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Overlay workspace:
```
source install/setup.bash
```
## How to use:

First produce the best starting guess you can come up with,
and edit it. There is an example in ``calib/example/example_camera-initial.yaml``. 
```
	cam0:
	  camera_model: pinhole
	  intrinsics: [605.054, 604.66, 641.791, 508.728]
	  distortion_model: equidistant
	  distortion_coeffs: [-0.0146915, 0.000370396, -0.00425216, 0.0015107]
	  resolution: [1280, 1024]
	  rostopic: /rig/left/image_mono
	cam1:
	  T_cn_cnm1:
	  - [ 0.99999965648, -0.00013331925,  0.00081808159, -0.19946344647]
	  - [ 0.00013388601,  0.99999975107, -0.00069277676, -0.00005674605]
	  - [-0.00081798903,  0.00069288605,  0.99999942540,  0.00010022941]
	  - [ 0.00000000000,  0.00000000000,  0.00000000000,  1.00000000000]
	  camera_model: pinhole
	  intrinsics: [605.097, 604.321, 698.772, 573.558]
	  distortion_model: equidistant
	  distortion_coeffs: [-0.0146155, -0.00291494, -0.000681981, 0.000221855]
	  resolution: [1280, 1024]
	  rostopic: /rig/right/image_mono
```

Adjust the ``rostopic`` to match your camera sources, match the resolution, distortion model etc.
The file format is identical to the [Kalibr file format](https://github.com/ethz-asl/kalibr).
Put it the edited file into a directory called e.g. ``stereo_cam`` or whatever the name of your device is.

Edit the launch file ``calibration.launch.py`` in the ``launch`` directory to your liking:
```python
parameters=[{'calib_dir': calib_dir,
             'target_type': 'aprilgrid',
             'tag_rows': 5,
             'tag_cols': 7,
             'tag_size': 0.04,
             'tag_spacing': 0.25,
             'black_border': 2,
             'tag_family': '36h11',
             'corners_in_file': '',
             'use_approximate_sync': False,
             'device_name': device_name}],
```
Adjust device_name to match the file name of your initial calibration file, e.g. ``stereo_cam``.
Match the rows, columns and tag size of your aprilgrid.
Pay attention to the size of the black border, and set ``use_approximate_sync`` to true if your
camera does not produce synchronized images (synced means: message time stamps must be identical across cameras).
Once done editing, install the launch file run above colcon command again), and launch it. 
```bash
ros2 launch multicam_calibration calibration.launch.py
```
You should see some output like this:
```
[calibration_node-1] parsing initial camera calibration file: /mypath/calib/stereo_cam/stereo_cam-initial.yaml
[calibration_node-1] [INFO] [1613243221.909816377] [multicam_calibration]: Found 2 cameras!
[calibration_node-1] [INFO] [1613243221.909937970] [multicam_calibration]: not using approximate sync
[calibration_node-1] [INFO] [1613243221.910176548] [multicam_calibration]: writing extracted corners to file /mypath/calib/stereo_cam/corners.csv
[calibration_node-1] [INFO] [1613243221.924387353] [multicam_calibration]: calibration_node started up!
[calibration_node-1] [INFO] [1613243225.043543976] [multicam_calibration]: frames:   10, total # tags found:  115 67
[calibration_node-1] [INFO] [1613243228.906891399] [multicam_calibration]: frames:   20, total # tags found:  233 131
[calibration_node-1] [INFO] [1613243257.529047985] [multicam_calibration]: frames:   30, total # tags found:  350 198
[calibration_node-1] [INFO] [1613243260.514001182] [multicam_calibration]: frames:   40, total # tags found:  467 266

```
Visualize the camera debug images:
```bash
ros2 run rqt_gui rqt_gui
```
Look for the ``debug_image`` topics. You should see something like this:
![Example Calibration Session](images/example_gui.jpg)

Then play your calibration bag (or do live calibration)

	rosbag play falcam_rig_2018-01-09-14-28-56.bag

You should see the tags detected (see above output). When you think you have
enough frames collected (5000 tags is usually plenty enough), you can start the calibration:

	ros2 service call /calibration std_srvs/srv/Trigger
	
This should give you output like this:
```
	Num params: 2476
	Num residuals: 201928
	iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
	0  4.478809e+03    0.00e+00    5.32e+06   0.00e+00   0.00e+00  1.00e+04        0    2.45e-01    3.10e-01
	1  1.291247e+03    3.19e+03    2.03e+05   1.46e+00   1.55e+00  3.00e+04        1    5.11e-01    8.21e-01
	2  1.288842e+03    2.40e+00    6.22e+03   2.38e-01   1.04e+00  9.00e+04        1    4.56e-01    1.28e+00
	3  1.288794e+03    4.79e-02    3.19e+02   3.57e-02   1.02e+00  2.70e+05        1    4.37e-01    1.71e+00
	4  1.288792e+03    2.27e-03    3.73e+01   7.64e-03   1.01e+00  8.10e+05        1    4.38e-01    2.15e+00
	5  1.288792e+03    2.61e-05    5.09e+00   7.20e-04   1.01e+00  2.43e+06        1    4.38e-01    2.59e+00
	6  1.288792e+03    6.92e-08    5.35e-01   3.46e-05   1.03e+00  7.29e+06        1    4.37e-01    3.03e+00

	Solver Summary (v 1.12.0-eigen-(3.2.92)-lapack-suitesparse-(4.4.6)-cxsparse-(3.1.4)-openmp)

	Original                  Reduced
	Parameter blocks                          410                      410
	Parameters                               2476                     2476
	Residual blocks                           409                      409
	Residual                               201928                   201928

	Minimizer                        TRUST_REGION

	Sparse linear algebra library    SUITE_SPARSE
	Trust region strategy     LEVENBERG_MARQUARDT
	
	Given                     Used
	Linear solver          SPARSE_NORMAL_CHOLESKY   SPARSE_NORMAL_CHOLESKY
	Threads                                     4                        4
	Linear solver threads                       1                        1
	Linear solver ordering              AUTOMATIC                      410
	
	Cost:
	Initial                          4.478809e+03
	Final                            1.288792e+03
	Change                           3.190017e+03
	
	Minimizer iterations                        7
	Successful steps                            7
	Unsuccessful steps                          0
	
	Time (in seconds):
	Preprocessor                           0.0653
	
	Residual evaluation                  0.0680
	Jacobian evaluation                  1.4113
	Linear solver                        1.5961
	Minimizer                              3.2011
	
	Postprocessor                          0.0000
	Total                                  3.2663
	
	Termination:                      CONVERGENCE (Function tolerance reached. |cost_change|/cost: 1.930077e-13 <= 1.000000e-12)
	
	[ INFO] [1515674589.074056064]: writing calibration to /home/pfrommer/Documents/foo/src/multicam_calibration/calib/example/example_camera-2018-01-11-07-43-09.yaml
	cam0:
	camera_model: pinhole
	intrinsics: [604.355, 604.153, 642.488, 508.135]
	distortion_model: equidistant
	distortion_coeffs: [-0.014811, -0.00110814, -0.00137418, 0.000474477]
	resolution: [1280, 1024]
	rostopic: /rig/left/image_mono
	cam1:
	T_cn_cnm1:
	- [ 0.99999720028,  0.00030730438,  0.00234627487, -0.19936845450]
	- [-0.00030303357,  0.99999829718, -0.00182038902,  0.00004464487]
	- [-0.00234683029,  0.00181967292,  0.99999559058,  0.00029671670]
	- [ 0.00000000000,  0.00000000000,  0.00000000000,  1.00000000000]
	camera_model: pinhole
	intrinsics: [604.364, 603.62, 698.645, 573.02]
	distortion_model: equidistant
	distortion_coeffs: [-0.0125438, -0.00503567, 0.00031359, 0.000546495]
	resolution: [1280, 1024]
	rostopic: /rig/right/image_mono
	[ INFO] [1515674589.251025662]: ----------------- reprojection errors: ---------------
	[ INFO] [1515674589.251045482]: total error:     0.283519 px
	[ INFO] [1515674589.251053450]: avg error cam 0: 0.28266 px
	[ INFO] [1515674589.251059520]: avg error cam 1: 0.284286 px
	[ INFO] [1515674589.251070091]: max error: 8.84058 px at frame: 110 for cam: 1
	[ INFO] [1515674589.251410620]: -------------- simple homography test ---------
	[ INFO] [1515674589.331235450]: camera: 0 points: 47700 reproj err: 0.440283
	[ INFO] [1515674589.331257726]: camera: 1 points: 53252 reproj err: 0.761365
```

In the calibration directory you can now find the output of the calibration:

	ls -1
	stereo_camera-2018-01-11-08-24-22.yaml
	stereo_camera-initial.yaml
	stereo_camera-latest.yaml


## Managed calibrations

Sometimes a calibration consists of a sequence of steps, for example: first the
intrinsics of each sensor, then the extrinsics of the sensors with
respect to each other. This is particularly useful when image data
between sensors is not synchronized.

To help with this, you can write a little python program that does
that. In fact, you just have to modify the section below in
``src/example_calib_manager.py``, and voila, when you trigger your
calibration manager, it will in turn run multiple calibrations via
service calls into the calibration node, each time retaining the
previous calibration's output as initial value. Here is an example
section, adjust as needed:

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

## Undistortion

For convenience this repo contains a node to undistort fisheye (equidistant) camera images.
Run it like this, after adjusting the parameters in the launch file:
```
ros2 launch multicam_calibration undistort.launch.py
```

## Unit tests

For unit testing of the calibration code, refer to [this page](multicam_calibratin/test/README.md).
NOTE: the tests have been ported but never run under ROS2. Expect things to be broken.


