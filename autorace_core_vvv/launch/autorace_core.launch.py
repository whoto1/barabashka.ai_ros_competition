import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package='autorace_core_vvv',
         executable='detect_line',
         namespace='detect',
         name='lane',
      ),
      Node(
         package='autorace_core_vvv',
         executable='pid',
         namespace='drive',
         name='pid',
      )
   ])
