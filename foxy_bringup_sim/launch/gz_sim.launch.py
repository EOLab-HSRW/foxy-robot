# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Launch Gazebo Sim without an intermediate shell.


Based on ros_gz_sim/launch/gz_sim.launch.py.in from the ros_gz Humble branch.

Local changes:
* Execute Gazebo with ``shell=False`` so ROS 2 Launch directly owns the
  Gazebo launcher process.
* Construct the command as an argv list instead of a shell command string.
* Parse ``gz_args`` / ``ign_args`` with ``shlex.split``.
* Use Gazebo Sim major version 8 as the local default instead of the
  upstream CMake template placeholder ``@GZ_SIM_VER@``.

"""

import os

from os import environ
import shlex
import shutil

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import (
    InvalidPackage,
    PACKAGE_MANIFEST_FILENAME,
    parse_package,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from ros2pkg.api import get_package_names


class GazeboRosPaths:
    """Collect Gazebo model, media and plugin paths exported by ROS packages."""

    @staticmethod
    def get_paths():
        gazebo_model_paths = []
        gazebo_plugin_paths = []
        gazebo_media_paths = []

        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)

            package_file_path = os.path.join(
                package_share_path,
                PACKAGE_MANIFEST_FILENAME,
            )

            if not os.path.isfile(package_file_path):
                continue


            try:
                package = parse_package(package_file_path)
            except InvalidPackage:
                continue

            for export in package.exports:
                if export.tagname != 'gazebo_ros':
                    continue

                if 'gazebo_model_path' in export.attributes:
                    xml_path = export.attributes['gazebo_model_path']
                    gazebo_model_paths.append(
                        xml_path.replace('${prefix}', package_share_path)
                    )

                if 'plugin_path' in export.attributes:
                    xml_path = export.attributes['plugin_path']
                    gazebo_plugin_paths.append(
                        xml_path.replace('${prefix}', package_share_path)
                    )

                if 'gazebo_media_path' in export.attributes:
                    xml_path = export.attributes['gazebo_media_path']
                    gazebo_media_paths.append(
                        xml_path.replace('${prefix}', package_share_path)
                    )

        model_and_media_paths = os.pathsep.join(
            gazebo_model_paths + gazebo_media_paths
        )
        plugin_paths = os.pathsep.join(gazebo_plugin_paths)
        return model_and_media_paths, plugin_paths



def get_executable_path(command):
    """Resolve a Gazebo CLI script and fail clearly when it is unavailable."""
    path = shutil.which(command)
    if path is None:
        raise RuntimeError(
            f"Unable to find executable '{command}' in PATH"
        )

    if path.lower().endswith('.bat'):
        return os.path.splitext(path)[0]

    return path


def _split_cli_args(argument_string):
    """Convert a launch argument string into an argv list."""
    try:
        return shlex.split(argument_string)
    except ValueError as exc:
        raise RuntimeError(
            f'Invalid Gazebo command-line arguments: {argument_string!r}'
        ) from exc


def launch_gz(context, *args, **kwargs):
    """Create the Gazebo process after evaluating launch configurations."""
    del args, kwargs

    model_paths, plugin_paths = GazeboRosPaths.get_paths()

    env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            environ.get('LD_LIBRARY_PATH', ''),
            plugin_paths,
        ]),
        # Deprecated, retained for pre-Garden compatibility.
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            environ.get('LD_LIBRARY_PATH', ''),
            plugin_paths,
        ]),
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
            environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_paths,
        ]),
        # Deprecated, retained for pre-Garden compatibility.
        'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join([
            environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
            model_paths,
        ]),
    }


    gz_args = LaunchConfiguration('gz_args').perform(context)
    gz_version = LaunchConfiguration('gz_version').perform(context)
    ign_args = LaunchConfiguration('ign_args').perform(context)
    ign_version = LaunchConfiguration('ign_version').perform(context)
    debugger = LaunchConfiguration('debugger').perform(context)
    on_exit_shutdown = LaunchConfiguration(
        'on_exit_shutdown'
    ).perform(context)
    debug_env = LaunchConfiguration('debug_env').perform(context)

    if not gz_args and ign_args:
        print('ign_args is deprecated; migrate to gz_args.')
        cli_args = _split_cli_args(ign_args)
    else:
        cli_args = _split_cli_args(gz_args)

    if ign_version or (not ign_version and int(gz_version) < 7):

        command = [
            'ruby',
            get_executable_path('ign'),
            'gazebo',
        ]
        if ign_version:
            gz_version = ign_version
    else:
        command = [
            'ruby',
            get_executable_path('gz'),
            'sim',
        ]

    debug_prefix = (
        'x-terminal-emulator -e gdb -ex run --args'
        if debugger != 'false'
        else None
    )

    on_exit = (
        Shutdown(reason='Gazebo Sim exited')
        if on_exit_shutdown != 'false'
        else None

    )

    command.extend(cli_args)
    command.extend(['--force-version', gz_version])

    actions = [
        ExecuteProcess(
            cmd=command,
            output='screen',
            additional_env=env,

            # Critical fix: launch the Gazebo CLI directly rather than through
            # /bin/sh. ROS 2 Launch can now signal the process it actually owns.
            shell=False,

            prefix=debug_prefix,
            on_exit=on_exit,
        )
    ]

    if debug_env == 'true':
        actions.insert(
            0,
            LogInfo(
                msg=f'Launching Gazebo with environment variables: {env}'
            ),
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value='',
            description='Arguments passed to Gazebo Sim',
        ),
        DeclareLaunchArgument(
            'gz_version',
            default_value='8',

            description='Gazebo Sim major version',

        ),
        DeclareLaunchArgument(
            'ign_args',
            default_value='',
            description='Deprecated: arguments passed to Gazebo Sim',
        ),
        DeclareLaunchArgument(
            'ign_version',
            default_value='',
            description='Deprecated: Gazebo Sim major version',
        ),
        DeclareLaunchArgument(
            'debugger',

            default_value='false',
            description='Run Gazebo in GDB',
        ),

        DeclareLaunchArgument(
            'debug_env',
            default_value='false',
            description='Print Gazebo environment variables',
        ),
        DeclareLaunchArgument(
            'on_exit_shutdown',

            default_value='false',
            description='Shut down the launch system when Gazebo exits',
        ),
        OpaqueFunction(function=launch_gz),
    ])
