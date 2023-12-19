# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import os
import cv2
import time
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String, Empty, Bool, UInt16
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class PublisherSubscriber(Node):

	def __init__(self):
		super().__init__('robot_app')
		self.publisher_ = self.create_publisher(String, '/commands', 1)
		self.obstacles_publisher = self.create_publisher(Empty, '/obstacles', 1)
		self.tunnel_publisher = self.create_publisher(Empty, '/tunnel', 1)
		self.crossroad_publisher = self.create_publisher(Empty, '/crossroad', 1)
		self.parking_publisher = self.create_publisher(UInt16, '/parking', 1)
		self.pedestrian_publisher = self.create_publisher(Bool, '/pedestrian', 1)
		self.subscription = self.create_subscription(Image, '/color/image', self.callback, 1)
		self.br = CvBridge()
		self.msg = String()
		self.left_or_right = UInt16()
		self.empty = Empty()
		self.is_human = Bool()
		self.startTime = 0.0
		self.subscription # prevent unused variable warn
		path = os.path.join(
			get_package_share_directory('autorace_core_vvv'),
			'calibration',
			'best.pt'
			)
		self.model = YOLO(path)
		self.names = self.model.names
		self.commands = {
			"green_light" :  self.green_light,
			"red_light" : self.skip,
			"yellow_light" : self.skip,
			"crossroad" : self.crossroad,
			"turn_left" : self.skip,
			"turn_right" : self.skip,
			"obstacles" : self.obstacles,
			"car" : self.skip,
			"parking" : self.obstacles_end,
			"right_parking" : self.skip,
			"left_parking" : self.skip,
			"pedestrian" : self.pedestrian,
			"human" : self.skip,
			"tunnel" : self.tunnel
		}
		self.wigths = {
			"red_light" : 1000,
			"yellow_light" : 1000,
			"green_light" : 0,
			"crossroad" : 200,
			"turn_left" : 100,
			"turn_right" : 80,
			"obstacles" : 190,
			"car" : 0,
			"parking" : 94,
			"right_parking" : 0,
			"left_parking" : 0,
			"pedestrian" : 200,
			"human" : 0,
			"tunnel" : 140
		}

	def callback(self, msg):
		dsensorImage = msg
		current_frame = self.br.imgmsg_to_cv2(dsensorImage, "bgr8")
		#YOLO
		results = self.model.predict(current_frame, show = True)
		for c in results[0]:
			value = c.boxes.cls.item()
			self.get_logger().info(f"this is yolo: {self.names[int(value)]}")
			box_coordinates = c.boxes.xyxy[0].cpu().numpy()
			width = box_coordinates[2] - box_coordinates[0]
			#self.get_logger().info(f"Here width {self.names[int(value)]}: {width}")
			centre = (box_coordinates[2] + box_coordinates[0])/2
			if width > self.wigths[self.names[int(value)]]: self.commands[self.names[int(value)]](centre)

			# self.get_logger().info(f"Upper-left coordinates: ({box_coordinates[0]}, {box_coordinates[1]})")
			# self.get_logger().info(f"Lower-right coordinates: ({box_coordinates[2]}, {box_coordinates[3]})")	

	def green_light(self, *args):
		self.msg.data = "green_light"
		self.publisher_.publish(self.msg)
		self.commands["green_light"] = self.skip

	def crossroad(self, *args):
		self.commands["turn_left"] = self.turn_left
		self.commands["turn_right"] = self.turn_right
		self.commands["crossroad"] = self.skip

	def turn_left(self, *args):		
		self.msg.data = "turn_left"
		self.publisher_.publish(self.msg)
		self.commands["turn_left"] = self.skip
		self.commands["turn_right"] = self.skip
		self.crossroad_publisher.publish(self.empty)

	def turn_right(self, *args):			
		self.msg.data = "turn_right"
		self.publisher_.publish(self.msg)
		self.commands["turn_right"] = self.skip
		self.commands["turn_left"] = self.skip
		self.crossroad_publisher.publish(self.empty)

	def obstacles(self, *args):
		self.obstacles_publisher.publish(self.empty)
		self.commands["obstacles"] = self.skip

	def obstacles_end(self, *args):
		self.obstacles_publisher.publish(self.empty)
		self.commands["parking"] = self.parking
		self.wigths["parking"] = 200

	def parking(self, *args):
		self.msg.data = "parking"
		self.publisher_.publish(self.msg)
		self.commands["left_parking"] = self.left_parking
		self.commands["right_parking"] = self.right_parking
		self.commands["parking"] = self.skip

	def left_parking(self, *args):
		if self.startTime == 0.0:
			self.startTime = time.time()
		if time.time() - self.startTime > 4.0:
			self.startTime = 0.0
			self.get_logger().info('in function left_parking')
			self.left_or_right.data = 1
			self.parking_publisher.publish(self.left_or_right)
			self.commands["left_parking"] = self.skip
			self.commands["right_parking"] = self.skip

	def right_parking(self, *args):
		if self.startTime == 0.0:
			self.startTime = time.time()
		if time.time() - self.startTime > 4.0:
			self.startTime = 0.0
			self.get_logger().info('in function right_parking')
			self.left_or_right.data = 2
			self.parking_publisher.publish(self.left_or_right)
			self.commands["left_parking"] = self.skip
			self.commands["right_parking"] = self.skip

	def pedestrian(self, *args):
		self.commands["human"] = self.human
		self.commands["pedestrian"] = self.skip

	def human(self, centre, *args):
		if centre < 600 and centre > 240:
			self.is_human.data = True
		else:
			self.is_human.data = False
			self.commands["human"] = self.skip
		self.pedestrian_publisher.publish(self.is_human)
	
	def tunnel(self, *args):
		self.tunnel_publisher.publish(self.empty)
		self.commands["tunnel"] = self.skip
		self.commands["green_light"] = self.the_end

	def the_end(self, *args):
		self.get_logger().info("I in the_end")
		self.tunnel_publisher.publish(self.empty)
		self.commands = {
			"green_light" :  self.skip,
			"red_light" : self.skip,
			"yellow_light" : self.skip,
			"crossroad" : self.crossroad,
			"turn_left" : self.skip,
			"turn_right" : self.skip,
			"obstacles" : self.obstacles,
			"car" : self.skip,
			"parking" : self.obstacles_end,
			"right_parking" : self.skip,
			"left_parking" : self.skip,
			"pedestrian" : self.pedestrian,
			"human" : self.skip,
			"tunnel" : self.tunnel
		}
		self.wigths = {
			"red_light" : 1000,
			"yellow_light" : 1000,
			"green_light" : 0,
			"crossroad" : 200,
			"turn_left" : 100,
			"turn_right" : 80,
			"obstacles" : 190,
			"car" : 0,
			"parking" : 85,
			"right_parking" : 0,
			"left_parking" : 0,
			"pedestrian" : 200,
			"human" : 0,
			"tunnel" : 140
		}

	def skip(self, *args):
		pass

def main(args=None):
    rclpy.init(args=args)
    robot_app = PublisherSubscriber()
    rclpy.spin(robot_app)
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
