import os
import random

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_block(i):
    model_path = os.path.join(get_package_share_directory('808x-final-project'), 'models')
    x_pos = random.choice([0.0, 2.2, -2.0])
    y_pos = -1.8

    return Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'book{}'.format(i),
              '-file', model_path + '/book.sdf',
              '-x', str(x_pos),
              '-y', str(y_pos),
              '-z', '0.0',
              '-Y', '1.5708'],
              output='screen'
    )

def generate_launch_description():

    blocks = []
    blocks.append(generate_block(1))

    return LaunchDescription(blocks)