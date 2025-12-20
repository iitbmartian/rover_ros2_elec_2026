from launch import LaunchDescription
from launch_ros.actions import Node
import os, subprocess
from ament_index_python import get_package_share_directory

package = 'arm_can'

resource_path = os.path.join(get_package_share_directory(package), 'scripts', 'can_init.sh')
subprocess.run(['bash', resource_path], check=True)

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=package,
                executable='arm_can',
                name='arm_can'
            ),
        ]
    )
