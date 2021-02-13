# multicam_calibration tests

## How to run

There are some pre-generated ``corners.csv`` and corresponding
Kalibr-format yaml files in the ``test`` subdirectories. To run
for example a test of the equidistant model:

    roslaunch multicam_calibration test_calib.launch type:=equidistant

Then look at the screen output and the calibration output files
generated into the corresponding ``test`` subdirectory. Does the
screen output match the `correct_output.txt`?

## How to generate grid points

Use the ``make_test_corners.py`` script to generate test points, specifying
in the calibration file you want to use:

    rosrun multicam_calibration make_test_corners.py -o ./corners.csv -f ./equidistant-initial.yaml

You may have to fiddle with some of the parameters in the script that determine
the size of the target in world coordinates, the (varying!) distance between camera and
target etc.
