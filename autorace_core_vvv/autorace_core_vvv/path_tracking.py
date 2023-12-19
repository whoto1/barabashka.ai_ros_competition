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

import time
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float64, String, UInt8
from sensor_msgs.msg import Image

class DetectLane(Node):

	def __init__(self):
		super().__init__('pid')
		self.declare_parameters(
            namespace='',
            parameters=[
            ('white/hue_l', 0),
            ('white/hue_h', 179),
            ('white/saturation_l', 0),
            ('white/saturation_h', 70),
            ('white/lightness_l', 105),
            ('white/lightness_h', 255),
            ('yellow/hue_l', 10),
            ('yellow/hue_h', 127),
            ('yellow/saturation_l', 70),
            ('yellow/saturation_h', 255),
            ('yellow/lightness_l', 95),
            ('yellow/lightness_h', 255),
        ])

		self.publisher_ = self.create_publisher(Float64, '/detect/lane', 10)
		self.subscription = self.create_subscription(Image, '/color/image_projected_compensated', self.cbFindLane, 1)
		self.subscription2 = self.create_subscription(String, '/commands', self.commands, 1)
		self.br = CvBridge()
		self.is_calibration_mode = True
		self.flags = {
			"green_light" : True,
			"turn_left" : False,
			"turn_right" : False,
			"parking" : False,
		}
		self.startTime = 0.0
		self.reliability_white_line = 100
		self.reliability_yellow_line = 100
		self.subscription # prevent unused variable warn
		self.subscription2

	def commands(self, msg):
		self.flags[msg.data] = True

        
	def cbFindLane(self, image_msg):
		self.hue_white_l = self.get_parameter("white/hue_l").get_parameter_value().integer_value
		self.hue_white_h = self.get_parameter("white/hue_h").get_parameter_value().integer_value
		self.saturation_white_l = self.get_parameter("white/saturation_l").get_parameter_value().integer_value
		self.saturation_white_h = self.get_parameter("white/saturation_h").get_parameter_value().integer_value
		self.lightness_white_l = self.get_parameter("white/lightness_l").get_parameter_value().integer_value
		self.lightness_white_h = self.get_parameter("white/lightness_h").get_parameter_value().integer_value
		self.hue_yellow_l = self.get_parameter("yellow/hue_l").get_parameter_value().integer_value
		self.hue_yellow_h = self.get_parameter("yellow/hue_h").get_parameter_value().integer_value
		self.saturation_yellow_l = self.get_parameter("yellow/saturation_l").get_parameter_value().integer_value
		self.saturation_yellow_h = self.get_parameter("yellow/saturation_h").get_parameter_value().integer_value
		self.lightness_yellow_l = self.get_parameter("yellow/lightness_l").get_parameter_value().integer_value
		self.lightness_yellow_h = self.get_parameter("yellow/lightness_h").get_parameter_value().integer_value
		
		cv_image = self.br.imgmsg_to_cv2(image_msg, "bgr8")
		self.window_width = cv_image.shape[1]
		self.window_height = cv_image.shape[0]
		# find White and Yellow Lanes
		white_fraction, cv_white_lane = self.maskWhiteLane(cv_image)
		yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_image)

		try:
			if yellow_fraction > 3000:
				self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, cv_yellow_lane)
				self.mov_avg_left = np.append(self.mov_avg_left,np.array([self.left_fit]), axis=0)

			if white_fraction > 3000:
				self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, cv_white_lane)
				self.mov_avg_right = np.append(self.mov_avg_right,np.array([self.right_fit]), axis=0)
		except:
			if yellow_fraction > 3000:
				self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
				self.mov_avg_left = np.array([self.left_fit])

			if white_fraction > 3000:
				self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
				self.mov_avg_right = np.array([self.right_fit])

		MOV_AVG_LENGTH = 5

		self.left_fit = np.array([np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
						np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
						np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
		self.right_fit = np.array([np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
						np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
						np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

		if self.mov_avg_left.shape[0] > self.window_width:
			self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

		if self.mov_avg_right.shape[0] > self.window_width:
			self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

		self.make_lane(cv_image, white_fraction, yellow_fraction)

	def maskWhiteLane(self, image):
        # Convert BGR to HSV
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		Hue_l = self.hue_white_l
		Hue_h = self.hue_white_h
		Saturation_l = self.saturation_white_l
		Saturation_h = self.saturation_white_h
		Lightness_l = self.lightness_white_l
		Lightness_h = self.lightness_white_h

		# define range of white color in HSV
		lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
		upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

		# Threshold the HSV image to get only white colors
		mask = cv2.inRange(hsv, lower_white, upper_white)
		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(image, image, mask = mask)
		
		fraction_num = np.count_nonzero(mask)

		if self.is_calibration_mode == False:
			if fraction_num > 35000:
				if self.lightness_white_l < 250:
					self.lightness_white_l += 5
			elif fraction_num < 5000:
				if self.lightness_white_l > 50:
					self.lightness_white_l -= 5

		how_much_short = 0

		for i in range(0, self.window_height):
			if np.count_nonzero(mask[i,::]) > 0:
				how_much_short += 1

		how_much_short = self.window_height - how_much_short

		# if how_much_short > 100:
		# 	if self.reliability_white_line >= 5:
		# 		self.reliability_white_line -= 5
		# elif how_much_short <= 100:
		# 	if self.reliability_white_line <= 99:
		# 		self.reliability_white_line += 5

		msg_white_line_reliability = UInt8()
		msg_white_line_reliability.data = self.reliability_white_line
		#self.pub_white_line_reliability.publish(msg_white_line_reliability)

		return fraction_num, mask
	
	def maskYellowLane(self, image):
        # Convert BGR to HSV
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		Hue_l = self.hue_yellow_l
		Hue_h = self.hue_yellow_h
		Saturation_l = self.saturation_yellow_l
		Saturation_h = self.saturation_yellow_h
		Lightness_l = self.lightness_yellow_l
		Lightness_h = self.lightness_yellow_h

		# define range of yellow color in HSV
		lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
		upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

		# Threshold the HSV image to get only yellow colors
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(image, image, mask = mask)

		fraction_num = np.count_nonzero(mask)

		if self.is_calibration_mode == False:
			if fraction_num > 35000:
				if self.lightness_yellow_l < 250:
					self.lightness_yellow_l += 20
			elif fraction_num < 5000:
				if self.lightness_yellow_l > 90:
					self.lightness_yellow_l -= 20

		how_much_short = 0

		for i in range(0, self.window_height):
			if np.count_nonzero(mask[i,::]) > 0:
				how_much_short += 1

		how_much_short = self.window_height - how_much_short

		# if how_much_short > 100:
		# 	if self.reliability_yellow_line >= 5:
		# 		self.reliability_yellow_line -= 5
		# elif how_much_short <= 100:
		# 	if self.reliability_yellow_line <= 99:
		# 		self.reliability_yellow_line += 5

		msg_yellow_line_reliability = UInt8()
		msg_yellow_line_reliability.data = self.reliability_yellow_line
		#self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)                       !!!
		return fraction_num, mask
	
	def fit_from_lines(self, lane_fit, image):
		nonzero = image.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])
		margin = 100
		lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
		nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

		# Again, extract line pixel positions
		x = nonzerox[lane_inds]
		y = nonzeroy[lane_inds]

		# Fit a second order polynomial to each
		lane_fit = np.polyfit(y, x, 2)

		# Generate x and y values for plotting
		ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
		lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
			
		return lane_fitx, lane_fit

	def sliding_windown(self, img_w, left_or_right):
		histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

		# Create an output image to draw on and visualize the result
		out_img = np.dstack((img_w, img_w, img_w)) * 255

		# Find the peak of the left and right halves of the histogram
		# These will be the starting point for the left and right lines
		midpoint = int(histogram.shape[0] / 2)

		if left_or_right == 'left':
			lane_base = np.argmax(histogram[:midpoint])
		elif left_or_right == 'right':
			lane_base = np.argmax(histogram[midpoint:]) + midpoint

		# Choose the number of sliding windows
		nwindows = 20

		# Set height of windows
		window_height = int(img_w.shape[0] / nwindows)

		# Identify the x and y positions of all nonzero pixels in the image
		nonzero = img_w.nonzero()
		nonzeroy = np.array(nonzero[0])
		nonzerox = np.array(nonzero[1])

		# Current positions to be updated for each window
		x_current = lane_base

		# Set the width of the windows +/- margin
		margin = 50

		# Set minimum number of pixels found to recenter window
		minpix = 50

		# Create empty lists to receive lane pixel indices
		lane_inds = []

		# Step through the windows one by one
		for window in range(nwindows):
			# Identify window boundaries in x and y
			win_y_low = img_w.shape[0] - (window + 1) * window_height
			win_y_high = img_w.shape[0] - window * window_height
			win_x_low = x_current - margin
			win_x_high = x_current + margin

			# Draw the windows on the visualization image
			cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

			# Identify the nonzero pixels in x and y within the window
			good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
				nonzerox < win_x_high)).nonzero()[0]

			# Append these indices to the lists
			lane_inds.append(good_lane_inds)

			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_lane_inds) > minpix:
				x_current = int(np.mean(nonzerox[good_lane_inds]))
		
		# Concatenate the arrays of indices
		lane_inds = np.concatenate(lane_inds)

		# Extract line pixel positions
		x = nonzerox[lane_inds]
		y = nonzeroy[lane_inds]

		# Fit a second order polynomial to each
		try:
			lane_fit = np.polyfit(y, x, 2)
			self.lane_fit_bef = lane_fit
		except:
			lane_fit = self.lane_fit_bef

		# Generate x and y values for plotting
		ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
		lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

		return lane_fitx, lane_fit

	def make_lane(self, cv_image, white_fraction, yellow_fraction):
		# Create an image to draw the lines on
		warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

		color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
		color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

		ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

		if yellow_fraction > 3000:
			pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
			cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=35)

		if white_fraction > 3000:
			pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
			cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=35)

		if self.flags["turn_left"] == True or self.flags["turn_right"] == True:
			if self.startTime == 0.0:
				self.startTime = time.time()
			if time.time() - self.startTime > 25.0:
				self.flags["turn_left"] = False
				self.flags["turn_right"] = False
				self.startTime = 0.0

		if self.flags["parking"] == True:
			if self.startTime == 0.0:
				self.get_logger().info('Start in path_tracking')
				self.startTime = time.time()
			if time.time() - self.startTime > 34.0:
				self.get_logger().info('Finish in path_tracking')
				self.flags["parking"] = False
				self.startTime = 0.0

		if self.flags["turn_left"] == False and self.flags["turn_right"] == False and self.flags["parking"] == False:  
			if white_fraction > 3000 and yellow_fraction > 3000:
				centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
				pts = np.hstack((pts_left, pts_right))
				pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
				cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)

				# Draw the lane onto the warped blank image
				cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

			elif white_fraction > 3000 and yellow_fraction <= 3000:
				centerx = np.subtract(self.right_fitx, 320)
				pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
				cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)

			elif white_fraction <= 3000 and yellow_fraction > 3000:
				centerx = np.add(self.left_fitx, 320)
				pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
				cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)
			else:
				centerx = np.add(np.zeros_like(self.left_fitx, dtype=float), 390)
				pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
				cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)

		elif self.flags["turn_left"] == True or self.flags["parking"] == True:
			centerx = np.add(self.left_fitx, 260)
			pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
			cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)

		elif self.flags["turn_right"] == True:
			centerx = np.subtract(self.right_fitx, 250)
			pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
			cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)
		else:
			centerx = np.add(np.zeros_like(self.left_fitx), 390)
			pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
			cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=15)

		# Combine the result with the original image
		#final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
		final = cv2.addWeighted(cv_image, 1, color_warp_lines, 1, 0)
		cv2.imshow('camera', final)
		cv2.waitKey(1)
		if self.flags["green_light"] == True:
			# publishes lane center
			msg_desired_center = Float64()
			msg_desired_center.data = centerx.item(350)
			self.publisher_.publish(msg_desired_center)
		

def main(args=None):
    rclpy.init(args=args)
    robot_app = DetectLane()
    rclpy.spin(robot_app)
	
    robot_app.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
