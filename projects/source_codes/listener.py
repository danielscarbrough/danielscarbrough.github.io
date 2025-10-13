# ros libraries
import sys
import rclpy

import time
import math

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from system_interfaces.msg import BBeam
from system_interfaces.msg import CameraMsg
from system_interfaces.msg import Display
from system_interfaces.msg import Incline

from system_interfaces.srv import MoveDistance
from system_interfaces.srv import CameraData
from system_interfaces.srv import MoveFlywheels
from system_interfaces.srv import RotateDistance
from system_interfaces.srv import CameraCheck
from system_interfaces.srv import ListenerSrv

from Adafruit_PCA9685 import PCA9685

class ListenerAsync(Node):

	# initialization function
	def __init__(self):
	
		super().__init__('listener_async')
		self.get_logger().info('Listener is online!')

		# callback groups
		motor_cb_group = MutuallyExclusiveCallbackGroup()
		flywheels_cb_group = MutuallyExclusiveCallbackGroup()
		position_cb_group = MutuallyExclusiveCallbackGroup()
		bbeam_cb_group = MutuallyExclusiveCallbackGroup()
		camera_cb_group = MutuallyExclusiveCallbackGroup()		
		display_cb_group = MutuallyExclusiveCallbackGroup()		
		listener_cb_group = MutuallyExclusiveCallbackGroup()		
		incline_cb_group = MutuallyExclusiveCallbackGroup()		
		
		# subscriptions
		self.subscription = self.create_subscription(BBeam, 'bbeam', self.bbeam_callback, 10, callback_group=bbeam_cb_group) # subscriber listening for bbeam data		
		self.cam_subscription = self.create_subscription(CameraMsg, 'camera', self.camera_callback, 10, callback_group=camera_cb_group) # subscriber listening for camera data		
		self.disp_subscription = self.create_subscription(Display, 'display_data', self.display_callback, 10, callback_group=display_cb_group) # subscriber listening for display data
		self.incline_subscription = self.create_subscription(Incline, 'incline', self.incline_callback, 10, callback_group=incline_cb_group) # subscriber listening for inclinometer data
		
		# clients
		self.motorCli = self.create_client(MoveDistance, 'move_distance',callback_group=motor_cb_group) # client to move drive wheel motors
		self.rotateCli = self.create_client(RotateDistance, 'rotate_distance',callback_group=motor_cb_group) # client to move drive wheel motors
		self.flywheelsCli = self.create_client(MoveFlywheels, 'move_flywheels',callback_group=flywheels_cb_group) # client to move the flywheels
		self.positionCli = self.create_client(CameraCheck, 'ball_check', callback_group=position_cb_group) # client to request position from the camera
		self.cameraCli = self.create_client(CameraData, 'ball_reset', callback_group=camera_cb_group) # client to request position from the camera
		
		# service
		self.srv = self.create_service(ListenerSrv, 'listener', self.updateDisplay, callback_group=listener_cb_group)

		# requests
		self.motorReq = MoveDistance.Request() # create the drive motors
		self.rotateReq = RotateDistance.Request() # create the drive motors
		self.flywheelsReq = MoveFlywheels.Request() # create the flywheels
		self.positionReq = CameraCheck.Request()
		
		# variables
		self.bbeam = 0
		self.speed = 0
		self.longBalls = 0
		self.shortBalls = 0
		self.goodBalls = 0
		self.numPutts = 0
		self.numBallsGiven = 0
		self.on = False
		self.ballDataArray = []
		self.ballsMissing = 0
		self.dist = 0
		self.ballsSeen = 0
		self.rotateDir = 1
		
		# function to control the PWM driver for the LEDs
		self.pwm = PCA9685(busnum=1)

	# function that is called by the display, gives the most up to date information collected
	def updateDisplay(self, request, response):
		if request.done == 0: # if the session is still going
			response.good_putts = self.goodBalls
			response.short_putts = self.shortBalls			
			response.long_putts = self.longBalls			
			response.ball_data = self.ballDataArray				
		else: # if the session is complete
			self.get_logger().info('No longer listening!')		
			# set all values to 0
			self.on = False

		return response
	
	# turn on the LEDs based on the information given by the inclinometer
	def incline_callback(self, msg):
		if (self.on):
			for n in range(0, 6):
				self.pwm.set_pwm(n, 0, 0)
			for leds in msg.new_inclines:
				self.get_logger().info('Turning on LED %d!'% leds)					
				self.pwm.set_pwm(leds, 0, 4095)		

	# a subscriber that keeps track of how many golf balls and putts the user has
	def display_callback(self, msg):
		
		# reset values on startup
		self.on = True
		self.ballsSeen = 0
		self.bbeam = 0
		self.speed = 0
		self.longBalls = 0
		self.shortBalls = 0
		self.goodBalls = 0
		self.numPutts = 0
		self.numBallsGiven = 0
		self.ballDataArray = []			
		self.ballsMissing = 0

		# set the data received by the display
		self.get_logger().info('The golfer is %d feet away, has %d golf balls and is putting %d for this session'% (msg.distance, msg.num_balls, msg.num_putts))
		self.numPutts = msg.num_putts
		self.numBallsGiven = msg.num_balls
		self.dist = msg.distance
		
		# request the motors to drive the given distance
		self.send_motor_request(self.dist, 0)

	# variable to check how many balls have been recorded, if any are missing
	def count_balls(self):
		
		totalBalls = self.shortBalls + self.goodBalls + self.longBalls # total putts
		self.ballsSeen = self.ballsSeen + 1	# balls seen for this "round"
		self.get_logger().info('I have seen %d balls so far!'% self.ballsSeen)	
		
		# turn on as many lights as there are balls
		for n in range(0, self.ballsSeen):
			self.pwm.set_pwm(n, 0, 4095)	
			self.get_logger().info('Turning on LED %d!'% n)	
			
		# check if all the balls have been putt or if the putter has no more putts
		if (self.ballsSeen == self.numBallsGiven or totalBalls == self.numPutts):
			
			# if there are missing balls, collect them
			if (self.ballsMissing != 0):
				self.get_logger().info('Looking for %d missing golf balls'% (self.ballsMissing))
				
				degreesRotated = 0
				
				# find and collect all golf balls
				for balls in range(0,self.ballsMissing):
				
					# while the camera function returns 0 -- rotate
					position = 0				
					# check before moving
					future = self.positionCli.call_async(self.positionReq)
					time.sleep(0.8) # might need to increase this delay
					response = future.result()
					position = response.ball_position

					while (position == 0): # check the camera if there's a ball in the center of the screen
						future = self.positionCli.call_async(self.positionReq)
						time.sleep(0.8)
						response = future.result()
						position = response.ball_position
						
						# rotate the robot some amount
						ROTATION = 5 * self.rotateDir # rotate in the direction of the nearest ball
						self.rotateReq.distance = ROTATION
						self.rotateCli.call_async(self.rotateReq)
						time.sleep(abs(ROTATION)/10)
						degreesRotated = degreesRotated + ROTATION	
						
					self.get_logger().info('Located ball %d inches away! '% (position))					
					# if there is, drive forward the distance recorded and then some
					# move the motors forward until the ball does, turn on the flywheels, and then move a little further (come back as well)
					self.send_flywheels_request(0, 1) # request the flywheels to recover the golf balls
					# are the flywheels spinning for long enough?
					self.send_motor_request(-(1+int(position/12)), 1)
					# (record the distance and map it? that would require calculating the angle and changing the coordinates to match... bleh) 
					
				# rotate back to initial position once all balls have been collected
				self.send_rotate_request(-degreesRotated)
				time.sleep(abs(degreesRotated)/10)
			
			# spit out all golf balls, regardless of whether more was collected
			self.send_flywheels_request(1, self.ballsSeen) # request the flywheels to launch out balls
			
			# wait for all the golf balls to be spit out
			time.sleep(self.ballsSeen * 5)
			self.ballsSeen = 0	
			
			# reset the LEDs since there are no more golf balls in the system
			self.get_logger().info('Reseting golf ball LEDs...')	
			for n in range(0, self.numBallsGiven):
				self.pwm.set_pwm(n, 0, 0)	

			# tell the camera all the balls in sight have been cleared
			self.cameraReq = CameraData.Request()
			self.cameraReq.reset = 2
			self.cameraCli.call_async(self.cameraReq)
			
		if (totalBalls == self.numPutts): # if the golfer has done all their putts
			self.get_logger().info('The putter has finished their session!')									
			# call the motor to return to the user
			self.send_motor_request(-self.dist, 0)
			
	# function that updates based on the break-beam's information		
	def bbeam_callback(self, msg):
		if (self.on): # if the session is still going
		
			self.bbeam = msg.bbeam
			self.speed = msg.speed
			self.get_logger().info('Bbeam number %d was broken at %d ft/sec!'% (bbeam, speed))
			
			if bbeam == 1: # if the first break beam broken, do nothing
				pass
			if bbeam == 2: # if the second break beam broken, turn on the flywheels to spin backwards
			
				self.send_flywheels_request(0, 1) # request the flywheels to spin backwards 
				self.send_motor_request(-1, 1) # move the motors a bit forward just in case the ball doesn't go in all the way		
				
				if (self.speed > 4): # if the ball is moving faster than 4 feet per second, it would skip the hole
					self.longBalls = self.longBalls + 1
				else:
					self.goodBalls = self.goodBalls + 1

				# wait for these to finish before counting balls
				self.count_balls()
	
	# function that processes the camera information
	def camera_callback(self, msg):
		if (self.on): # while the session is still going
		
			# update the ball coordinates
			self.ballDataArray = msg.ball_data
			ballData = [i for i in zip(*[iter(self.ballDataArray)]*3)] # take the list and make it into a list of tuples

			# check the breakbeams to see how short the ball is and do collection process
			match self.bbeam:
			
				case 0: # very short
					self.get_logger().info('The ball was VERY short')            						
					self.shortBalls = self.shortBalls + 1
					
					balls = ballData[-1] # only check the last golf ball recorded

					if abs(balls[0]) <= 6 and balls[2] == 0: # if one of the x coordinates is less than or equal to 6 inches from the center (where the ball is able to be collected)
						self.get_logger().info('The ball is in front of the hole!')									
						self.send_flywheels_request(0, 1) # request the flywheels to spin backwards
						self.send_motor_request(-(int(balls[1]/12) + 1), 1) # move the motors the y distance and then some
							
						# tell the camera the nearby balls have been cleared
						self.cameraReq = CameraData.Request() # create the updater for the display	
						self.cameraReq.reset = 1
						self.cameraCli.call_async(self.cameraReq)
							
					else: # if the balls are not nearby to be collected
						if balls[2] == 1: # if the ball went missing from the screen
							# tell the camera to clear the missing balls from the data
							self.cameraReq = CameraData.Request() # create the updater for the display	
							self.cameraReq.reset = 4
							self.cameraCli.call_async(self.cameraReq)

						self.get_logger().info('The ball is not in front of the hole, will collect later')
						self.ballsMissing = self.ballsMissing + 1
						# check the direction the ball is in, set the direction to look later based on that
						if balls[0] > 0:
							self.rotateDir = 1
						else:
							self.rotateDir = -1
					
					# wait for these to finish before counting balls					
					self.count_balls()					
					
					return
				case 1: # slightly short
					self.get_logger().info('The ball was only a bit short')            						
					self.shortBalls = self.shortBalls + 1			
					
					# move the motors forward until the ball does, turn on the flywheels, and then move a little further
					self.send_flywheels_request(0, 1) # request the flywheels to spin backwards 
					self.send_motor_request(-2, 1)

					# wait for these to finish before counting balls
					self.count_balls()
					return

	# requests to move the motor and flywheels
	def send_motor_request(self, distance, comeback): # define the request to move a set distance

		# constants
		DIAMETER = 3.125 # in inches
		RPM = 160

		self.motorReq.distance = distance
		self.motorReq.comeback = comeback
		self.motorCli.call_async(self.motorReq)
		
		time.sleep(float(abs(distance) / ((math.pi*DIAMETER / 12) * RPM / 60)) + 2) # wait the time it takes for the motors to run, plus a little more		

	def send_rotate_request(self, distance): # define the request to move a set distance
		self.rotateReq.distance = distance
		return self.rotateCli.call_async(self.rotateReq)

	def send_flywheels_request(self, direction, balls): # define the request to move a set direction
		self.flywheelsReq.direction = direction
		self.flywheelsReq.balls = balls
		return self.flywheelsCli.call_async(self.flywheelsReq)

def main():
	rclpy.init()

	listener = ListenerAsync()
	executor = MultiThreadedExecutor()
	executor.add_node(listener)
	
	executor.spin()

	display.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
