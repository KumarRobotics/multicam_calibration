# -----------------------------------------------------------------------------
# Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
# example launch file for calibration test
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg


def generate_launch_description():
    """Launch test component."""
    container = ComposableNodeContainer(
            name='calibration_test_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='multicam_calibration',
                    plugin='multicam_calibration::CalibrationTestComponent',
                    name=LaunchConfig('node_name'),
                    parameters=[{'fov_scale': 1.0,
                                 'balance': 1.0,
                                 'ignore_wrong_distortion': True}],
                    remappings=[('/image', '/cam_0/synced/image_raw'),
                                ('/camera_info_raw', '/cam_0/synced/camera_info')
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )
    name_arg = LaunchArg('node_name', default_value=['calibration_test'],
                         description='calibration_test')
    return launch.LaunchDescription([name_arg, container])
