#!/usr/bin/env python3

import launch
import os

from launch_ros.actions import Node, PushROSNamespace
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # launch.logging.get_logger().setLevel(launch.logging.logging.DEBUG)

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_uav_ardupilot_api'

    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ args from ENV

    uav_name = os.getenv('UAV_NAME', 'uav1')
    ARDUPILOT_IP = os.getenv('ARDUPILOT_IP', '')
    ARDUPILOT_SERIAL = os.getenv('ARDUPILOT_SERIAL', '/dev/ttyACM0')
    # ArduPilot's default SERIALx_BAUD is 115 (115200). On USB CDC (/dev/ttyACM*) the baudrate is
    # effectively ignored by the device, but mavros still requires a sane value here; using PX4's
    # 2000000 high-speed default causes a serial receive EOF / mavros crash on ArduPilot FCUs.
    ARDUPILOT_BAUD = os.getenv('ARDUPILOT_BAUD', '115200')

    if ARDUPILOT_IP:
      fcu_url = f'udp://:14550@{ARDUPILOT_IP}:14550'
    else:
      fcu_url = f'{ARDUPILOT_SERIAL}:{ARDUPILOT_BAUD}'

    gcs_url = 'tcp-l://'
    respawn_mavros = os.getenv('respawn_mavros', 'false') == 'true'

    # #} end of args from ENV

    tgt_system = 1
    namespace = uav_name

    apm_launch_arguments = {
        'fcu_url': fcu_url,
        'gcs_url': gcs_url,
        'tgt_system': str(tgt_system),
        'tgt_component': str(1),
        'log_output': 'screen',
        'fcu_protocol': 'v2.0',
        'respawn_mavros': str(respawn_mavros),
        'namespace': 'mavros',
        'pluginlists_yaml': this_pkg_path + '/config/mavros_apm_pluginlists.yaml',
        'config_yaml': this_pkg_path + '/config/mavros_apm_config.yaml',
    }

    print(apm_launch_arguments.items())

    launch_xml_include_with_namespace = GroupAction(
        actions=[
            # push_ros_namespace first to set namespace of included nodes for following actions
            PushROSNamespace(namespace),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mrs_uav_ardupilot_api'),
                        'launch/mavros.launch')
                    ),
                    launch_arguments=apm_launch_arguments.items()
            ),
        ],
    )

    ld.add_action(launch_xml_include_with_namespace)

    ld.add_action(
        # Nodes under test
        Node(
            package='tf2_ros',
            namespace='',
            executable='static_transform_publisher',
            name='fcu_to_garmin',
            arguments=['0.0', '0.0', '-0.05', '0', '1.57', '0', uav_name + '/fcu', 'garmin'],
        )
    )

    return ld
