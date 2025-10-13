from system_interfaces.srv import CameraData # camera variables
from system_interfaces.msg import CameraMsg # camera variables
from system_interfaces.srv import CameraCheck
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import sys
import rclpy
from rclpy.node import Node

import cv2
import math
import time
import numpy as np

from ultralytics import YOLO

coordsList = []
bboxList = []
numBalls = 0 # number of balls (not necessarily on screen)
ballsLeftScreen = 0

identified = 0

x1 = 0
x2 = 0
y1 = 0
y2 = 0

model = None
cap = None
kalman = None

class Camera(Node):

	# initialization function
	def __init__(self):
		global model, cap, kalman

		# KALMAN SETUP -- NOT BEING USED

		# Initialize Kalman filter
		kalman = cv2.KalmanFilter(8, 4)  # 8 state variables, 4 measurement variables
		# measurement variables are the four corners of the bounding box
		kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
											[0, 1, 0, 0, 0, 0, 0, 0],
											[0, 0, 1, 0, 0, 0, 0, 0],
											[0, 0, 0, 1, 0, 0, 0, 0]], np.float32)
		# linear relationship between the position and velocity
		kalman.transitionMatrix = np.array([[1, 0, 0, 0, 1, 0, 0, 0],
											[0, 1, 0, 0, 0, 1, 0, 0],
											[0, 0, 1, 0, 0, 0, 1, 0],
											[0, 0, 0, 1, 0, 0, 0, 1],
											[0, 0, 0, 0, 1, 0, 0, 0],
											[0, 0, 0, 0, 0, 1, 0, 0],
											[0, 0, 0, 0, 0, 0, 1, 0],
											[0, 0, 0, 0, 0, 0, 0, 1]], np.float32)
		kalman.processNoiseCov = np.eye(8, dtype=np.float32) * 1e-5
		kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 1e-1
		kalman.errorCovPost = np.eye(8, dtype=np.float32) * 1

		# Load the YOLO11 model
		model = YOLO("/home/on-par/onpar_ws/onpar_v3_ncnn_model", task='detect')

		# Open the webcam
		cap = cv2.VideoCapture(0)
	
		super().__init__('camera')
		
		# callback groups
		camera_cb_group = MutuallyExclusiveCallbackGroup()
		camera_timer_cb_group = MutuallyExclusiveCallbackGroup()
		
		# services
		self.position_srv = self.create_service(CameraCheck, 'ball_check', self.checkPosition)
		self.camera_srv = self.create_service(CameraData, 'ball_reset', self.checkCamera	)
		self.publisher_ = self.create_publisher(CameraMsg, 'camera', 10, callback_group=camera_cb_group) # publisher that says how many golf balls are on screen
		
		self.get_logger().info('Camera is online!')
		
		# timers
		self.timer = self.create_timer(0.1, self.camera_timer_callback, callback_group=camera_timer_cb_group) # timer to run ml algorithm repeatedly

		# array that stores all the balls' positions for a given session
		self.ballData = []

	# function that sends the position of all the golf balls for the session
	def camera_publisher(self):
 		# pass as an array
		#self.get_logger().info('sending camera data')						
		ballArray = []
		for tup in self.ballData:
			ballArray.extend(tup)
		
		msg = CameraMsg()
		msg.ball_data = ballArray
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing the camera data!')
   
	# function that is really just used to reset the camera data now
	def checkCamera(self, request, response):
		global coordsList, bboxList, numBalls, ballsLeftScreen
		
		if (request.reset == 1): # if the balls have been collected
			#coordsList.clear()

			# only clear balls that are less than or equal to 6 inches from the center, that would have been collected			
			checkedCoords = 0
			for coords in coordsList:
				if abs(coords[1]) <= 6:
					del coordsList[checkedCoords]
				checkedCoords = checkedCoords + 1			

			bboxList.clear()
			numBalls = 0
			ballsLeftScreen = 0
			self.get_logger().info('Nearby balls have been cleared!')
			
		elif (request.reset == 2): # if a full collection has occurred, clear the coords list entirely but not the ball data
			coordsList.clear()
			bboxList.clear()
			numBalls = 0
			ballsLeftScreen = 0
			self.get_logger().info('All balls have been cleared!')
			
		elif (request.reset == 3): # if this is a new session, reset ALL variables
			self.ballData.clear() # clear the map entirely
			coordsList.clear()
			bboxList.clear()
			numBalls = 0
			ballsLeftScreen = 0
			self.get_logger().info('All data has been cleared for new session!')

		elif (request.reset == 4): # if the balls have been collected

			# only clear balls that are missing		
			checkedCoords = 0
			for coords in coordsList:
				if abs(coords[5]) == 1:
					del coordsList[checkedCoords]
				checkedCoords = checkedCoords + 1			

			bboxList.clear()
			numBalls = 0
			ballsLeftScreen = 0
			self.get_logger().info('Missing balls have been cleared!')


		else:	# not really relevant anymore
			# pass as an array
			#self.get_logger().info('sending camera data')						
			ballArray = []
			for tup in self.ballData:
				ballArray.extend(tup)
			
			response.ball_data = ballArray
		return response

	# function used Kalman prediction, obsolete
	def predict_bounding_box(x1_measured, x2_measured, y1_measured, y2_measured):
		global kalman
		"""Predicts bounding box width and height using Kalman filter."""
		measurement = np.array([[np.float32(x1_measured)], [np.float32(x2_measured)], [np.float32(y1_measured)],[np.float32(y2_measured)]])
			
		# Prediction step
		kalman.predict()

		# Update step
		kalman.correct(measurement)
		
		predicted = kalman.statePost
		#print(kalman.statePost)
		return int(predicted[0]), int(predicted[1]), int(predicted[2]), int(predicted[3]), int(predicted[4]), int(predicted[5]), int(predicted[6]), int(predicted[7])

	# function that finds the position of the golf balls currently in front of the camera
	# returns the distances away for the stimpmeter test
	# returns if a ball is centered for the collection process, and if there is, how far away it is
	def checkPosition(self, request, response):
		global cap, model
		
		distances = []
		# variable to store if a ball is in the center of the screen
		isCentered = 0

		# Read a frame from the video
		success, frame = cap.read()
		if success:
			successTime = time.time_ns() / 1_000_000_000 # take current time in seconds
			# Run YOLO11 tracking on the frame, persisting tracks between frames
			results = model.predict(frame, verbose=False)

			# take the results
			results_coords = results[0]
						
			# take the bounding box with each ID
			for box in results_coords.boxes:
				# set new value of 
				x1, y1, x2, y2 = box.xyxy.tolist()[0]
				
				# calculate golf ball pixels
				x_length = x2 - x1 # size in pixels
				y_length = y2 - y1

				length = max(x_length, y_length) # take the max of x height and y height in case one is cut off

				yCoord = 969.216 * length ** -0.9355 # convert pixels to distance in inches

				x_center = (x2 + x1)/2 # find center of the bounding box
				x_distance = x_center - 320 # find the pixel distance of the center of the ball to the center of the camera
				# defined such that negative is to the left of the camera
				xCoord = x_distance / (927.202 * yCoord**-1.069) # convert pixels to inches
				
				isCentered = 0
				# if the x coordinate is close enough to the center
				if (abs(xCoord) <= 3):
					isCentered = yCoord # prepare to return the y coordinate of a centered golf ball
				distance = int(math.sqrt(xCoord**2 + yCoord**2)) # find the distance away for the stimpmeter
				distances.append(distance)	

		# return all results
		response.ball_distance = distances
		response.ball_position = int(isCentered)
		return response
		
	# function to keep track of all the golf balls that have been marked by the system
	def camera_timer_callback(self):	
		global cap, model, kalman, numBalls, coordsList, bboxList, identified, numBalls, ballsLeftScreen
		global numBalls, x1, x2, y1, y2

		# Read a frame from the video
		success, frame = cap.read()
		if success:

			# keep track of when the processing begins
			successTime = time.time_ns() / 1_000_000_000 # take current time in seconds

			# Run YOLO11 tracking on the frame, persisting tracks between frames
			results = model.track(frame, persist=True, verbose=False, tracker="/home/on-par/onpar_ws/onpar_botsort.yaml")

			# take the results
			results_coords = results[0]

			# update the number of balls spotted if necessary
			if len(results_coords.boxes) > numBalls: numBalls = len(results_coords.boxes)
			
			# take the bounding box with each ID
			for box in results_coords.boxes:
				if box.id != None:
						identifier = box.id.tolist()[0]
				else:
					identifier = -1
				identifier = int(identifier)
				
				for bbox in bboxList: # for all currently known golf balls
					if bbox[0] == identifier: # if the ID matches
						# get old value of the bounding box
						x1 = bbox[1]
						x2 = bbox[2]
						y1 = bbox[3]
						y2 = bbox[4]

				# set post state based on previous value
				#kalman.statePost = np.array([[x1], [x2], [y1], [y2], [0], [0], [0], [0]], dtype=np.float32) # this should be updated BEFORE the coordinates changes

				# set new value of the bounding box coordinates
				x1, y1, x2, y2 = box.xyxy.tolist()[0]

				# KALMAN PREDICTION
				#x1_predicted, x2_predicted, y1_predicted, y2_predicted, x1_velocity, x2_velocity, y1_velocity, y2_velocity = predict_bounding_box(x1, x2, y1, y2)
				#print(f"Measured: ({x1}, {x2}, {y1}, {y2}), Predicted: ({x1_predicted}, {x2_predicted}, {y1_predicted}, {y2_predicted})")

				# predict if the ball will go off screen
				#if (x2_predicted > 640 or x1_predicted < 0):
				#    print("The golf ball is expected to go offscreen!")

				# DISTANCE + VELOCITY CALCULATIONS

				# calculate golf ball pixels
				x_length = x2 - x1 # size in pixels
				y_length = y2 - y1

				length = max(x_length, y_length) # take the max of x height and y height in case one is cut off
				yCoord = 969.216 * length ** -0.9355 # convert pixels to distance in inches

				x_center = (x2 + x1)/2 # find center of the bounding box
				x_distance = x_center - 320 # find the pixel distance of the center of the ball to the center of the camera
				# defined such that negative is to the left of the camera
				xCoord = x_distance / (927.202 * yCoord**-1.069) # convert pixels to inches

				# TRACKING THE GOLF BALLS

				# check if this coordinate already exists
				coordsChecked = 0
				identified = 0
				expectation = 1

				for coords in coordsList: # for all currently known golf balls
					if coords[0] == identifier: # if the ID matches
						identified = 1 # there exists a coordinate for this ID already

						if coords[0] != -1 and coords[3] != 0: # make sure the ID exists and is moving

							# check if the ID has moved significantly
							distValue = math.sqrt((coords[1] - xCoord)**2 + (coords[2] - yCoord)**2)
							if distValue < 0.05 and distValue != 0 and coords[2] < 48:
								if coords[3] != 0: self.get_logger().info("Golf ball has stopped at %d, %d!"%(xCoord,yCoord))								
								expectation = 0
								self.ballData.append((coords[1], coords[2], 0))
								self.camera_publisher()
								
							del coordsList[coordsChecked] # delete the previous one                    
							coordsList.append((identifier, xCoord, yCoord, expectation, successTime, 0)) # add a new one
							bboxList.append((identifier, x1, x2, y1, y2))

					coordsChecked = coordsChecked + 1 # keep track of which golf ball is currently being tested

				# save this coordinate as a new golf ball set with the ID if it does not have a matching ID
				if identified == 0 and identifier != -1 and len(coordsList) < numBalls:
					self.get_logger().info('New golf ball located at %d,%d!'%(xCoord, yCoord))
					coordsList.append((identifier, xCoord, yCoord, expectation, successTime, 0))
					bboxList.append((identifier, x1, x2, y1, y2))

			# tested even if there are no golf balls on screen
			coordsChecked = 0				
			# check if a ball went missing
			for coords in coordsList: # for all coordinates
				if coords[3] != 0: # if it is still expected to move
					currentTime = time.time_ns() / 1_000_000_000 # find the current time in seconds
					if currentTime > coords[4] + 2: # if two seconds have passed since the last successful recognition
						
						expectation = 0
						identifier = coords[0]
						xCoord = coords[1]
						yCoord = coords[2]
						del coordsList[coordsChecked] # delete the previous one                    
						coordsList.append((identifier, xCoord, yCoord, expectation, currentTime, 1)) # add a new one

						self.get_logger().info('Oh no! The ball seems to have gone missing at %d, %d...' % (coords[1], coords[2]))
						self.ballData.append((coords[1], coords[2], 1)) # publish the last known coordinates, along with a 1 signifying to the other programs the ball is missing
							
						self.camera_publisher() # tell the listener to check the camera

				coordsChecked = coordsChecked + 1

def main():
	rclpy.init()
	camera = Camera()
	rclpy.spin(camera)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

