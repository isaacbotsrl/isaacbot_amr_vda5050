# Copyright 2026 IsaacBot
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

"""VDA 5050 connector for Ardennes: MQTT bridge, controller and isaacbot adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    isaacbot_amr_namespace = arg('namespace').strip('/')
    # The connector's nodes live beside the isaacbot's, not among them: <namespace>/vda5050.
    connector_namespace = f'{isaacbot_amr_namespace}/vda5050' if isaacbot_amr_namespace else 'vda5050'

    manufacturer = arg('manufacturer')
    serial_number = arg('serial_number')
    use_sim_time = _as_bool(arg('use_sim_time'))
    params_file = arg('params_file')

    identity = {
        'manufacturer_name': manufacturer,
        'serial_number': serial_number,
        'use_sim_time': use_sim_time,
    }

    mqtt_bridge = Node(
        package='isaacbot_amr_vda5050_connector',
        executable='mqtt_bridge.py',
        name='mqtt_bridge',
        namespace=connector_namespace,
        output='screen',
        parameters=[
            params_file,
            {
                **identity,
                'mqtt_address': arg('broker_host'),
                'mqtt_port': int(arg('broker_port')),
                'mqtt_username': arg('broker_username'),
                'mqtt_password': arg('broker_password'),
            },
        ],
    )

    controller = Node(
        package='isaacbot_amr_vda5050_connector',
        executable='vda5050_controller.py',
        name='controller',
        namespace=connector_namespace,
        output='screen',
        parameters=[params_file, {**identity, 'robot_name': serial_number}],
    )

    # Delivery (isaacbot_amr_delivery_portal): the shared ENU frame and the cargo box. Each goes in
    # only when set, so connector.yaml's defaults hold otherwise.
    delivery = {}
    for name, parameter, kind in (
            ('datum_lat', 'isaacbot.geo.datum_lat', float),
            ('datum_lon', 'isaacbot.geo.datum_lon', float),
            ('enu_map_id', 'isaacbot.geo.enu_map_id', str),
            ('parked_heading_deg', 'isaacbot.geo.parked_heading_deg', float),
            ('cargo_lock_output', 'isaacbot.cargo.lock_output', int)):
        value = arg(name).strip()
        if value:
            delivery[parameter] = kind(value)

    adapter = Node(
        package='isaacbot_amr_vda5050_adapter',
        executable='adapter_node',
        name='adapter',
        namespace=connector_namespace,
        output='screen',
        parameters=[
            params_file,
            {
                **identity,
                'robot_name': serial_number,
                'isaacbot.namespace': isaacbot_amr_namespace,
                'isaacbot.map_frame': arg('map_frame'),
                **delivery,
            },
        ],
    )

    return [mqtt_bridge, controller, adapter]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('isaacbot_amr_vda5050_bringup'), 'config', 'connector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=os.environ.get('ISAACBOT_AMR_SYSTEM_NAMESPACE', 'isaacbot'),
            description='Isaacbot namespace; the connector runs under <namespace>/vda5050.'),
        DeclareLaunchArgument(
            'broker_host', default_value='127.0.0.1', description='MQTT broker host.'),
        DeclareLaunchArgument(
            'broker_port', default_value='1883', description='MQTT broker port.'),
        DeclareLaunchArgument(
            'broker_username', default_value='',
            description='MQTT user; non-empty also enables TLS (CA from '
                        'VDA5050_CONNECTOR_TLS_CA_CERT).'),
        DeclareLaunchArgument(
            'broker_password', default_value='', description='MQTT password.'),
        DeclareLaunchArgument(
            'manufacturer', default_value='IsaacBot',
            description='VDA 5050 manufacturer (MQTT topic level).'),
        DeclareLaunchArgument(
            'serial_number', default_value='ardennes',
            description='VDA 5050 serialNumber (MQTT topic level): A-Z a-z 0-9 _ . : -'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Nav 2 global frame the order coordinates are in; prefixed with '
                        '<namespace>/.'),
        DeclareLaunchArgument(
            'datum_lat', default_value='',
            description='Delivery: latitude of the shared ENU frame origin (the pizzeria). '
                        'Empty with datum_lon: no ENU frame (map-frame orders only).'),
        DeclareLaunchArgument(
            'datum_lon', default_value='', description='Delivery: longitude of the ENU origin.'),
        DeclareLaunchArgument(
            'enu_map_id', default_value='',
            description="Delivery: the ENU frame's VDA 5050 mapId (as the portal's ENU_MAP_ID)."),
        DeclareLaunchArgument(
            'parked_heading_deg', default_value='',
            description='Delivery: compass bearing (0 = North, clockwise) the isaacbot faces '
                        'parked at the shop; seeds the GPS alignment of each trip.'),
        DeclareLaunchArgument(
            'cargo_lock_output', default_value='',
            description='Delivery: aux output 1..6 driving the cargo box lock; 0 = none.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use /clock (Gazebo).'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Connector parameters (config/connector.yaml).'),
        OpaqueFunction(function=_launch_setup),
    ])
