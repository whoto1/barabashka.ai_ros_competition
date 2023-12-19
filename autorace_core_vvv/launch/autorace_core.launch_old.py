import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #config = os.path.join(
     # get_package_share_directory('autorace_core_vvv'),
      #'calibration',
      #'calibration.yaml'
      #)
   
   #pid_config = os.path.join(
    #  get_package_share_directory('autorace_core_vvv'),
    #  'calibration',
    #  'pid_config.yaml'
     # )


   return LaunchDescription([
      Node(
         package='autorace_core_vvv',
         executable='detect_line',
         namespace='detect',
         name='lane',
         #parameters=[config]
      ),
      Node(
         package='autorace_core_vvv',
         executable='pid',
         namespace='drive',
         name='pid',
         #parameters=[pid_config]
      )
   ])
