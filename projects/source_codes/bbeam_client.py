# ros libraries
import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

from system_interfaces.msg import BBeam

# gpio libraries
import RPi.GPIO as GPIO
import time

# bbeam gpio pins
firstBeam = 10
secondBeam = 9

# setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(firstBeam,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(secondBeam,GPIO.IN, pull_up_down=GPIO.PUD_UP)

class BBeamAsync(Node):

	# initialization function
	def __init__(self):
		
		super().__init__('bbeam_async')
		
		# create callback groups that may or may not do anything
		bbeam_cb_group = MutuallyExclusiveCallbackGroup()
		bbeam_one_timer_cb_group = MutuallyExclusiveCallbackGroup()
		bbeam_two_timer_cb_group = MutuallyExclusiveCallbackGroup()
		
		# timers
		self.timer = self.create_timer(0.5, self.bbeam_one_timer_callback, callback_group=bbeam_one_timer_cb_group) # timer to run break-beam code repeatedly
		self.timer = self.create_timer(0.5, self.bbeam_two_timer_callback, callback_group=bbeam_two_timer_cb_group) # timer to run break-beam code repeatedly
		
		self.publisher_ = self.create_publisher(BBeam, 'bbeam', 10, callback_group=bbeam_cb_group) # publisher that says how many golf balls and number of putts in the session

	# function for the first break beam
	def bbeam_one_timer_callback(self):
		self.get_logger().info('waiting for the first break beam to sense a golfball...')

		# FIRST BREAK BEAM
		while (GPIO.input(firstBeam)): # wait to sense a golf ball when the signal is high
			# ask the camera if a golf ball has stopped yet, if it has, break out of the while and define it as short				
			time.sleep(0.1)			
		self.get_logger().info('I SENSE A GOLF BALL!!')			
		startTime = time.time_ns() # start the timer
			
		while (not GPIO.input(firstBeam)): # wait until the golf ball has passed
			pass 
					
		endTime = time.time_ns() # end the timer

		totalTime = (endTime-startTime) / 1_000_000_000
				
		self.get_logger().info('I waited for %d seconds!' % (int(totalTime)))
		ballSpeed = int(round(0.14/totalTime, 2))
		self.get_logger().info('For a golf ball of size 1.68 inches, the golf ball was traveling at %d ft/sec!' % ballSpeed)
		
		msg = BBeam() # set up message
		# fill message
		msg.bbeam = 1
		msg.speed = ballSpeed
		# publish message
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing the first bbeam status!')

	# function for the second break beam -- identical to the first but with slightly different variables
	def bbeam_two_timer_callback(self):
		self.get_logger().info('waiting for the second break beam to sense a golfball...')

		# FIRST BREAK BEAM
		while (GPIO.input(secondBeam)): # wait to sense a golf ball when the signal is high
			# ask the camera if a golf ball has stopped yet, if it has, break out of the while and define it as short				
			time.sleep(0.1)			
		self.get_logger().info('I SENSE A GOLF BALL!!')			
		startTime = time.time_ns() # start the timer
			
		while (not GPIO.input(secondBeam)): # wait until the golf ball has passed
			pass 
					
		endTime = time.time_ns() # end the timer

		totalTime = (endTime-startTime) / 1_000_000_000
				
		self.get_logger().info('I waited for %d seconds!' % (int(totalTime)))
		ballSpeed = int(round(0.14/totalTime, 2))
		self.get_logger().info('For a golf ball of size 1.68 inches, the golf ball was traveling at %d ft/sec!' % ballSpeed)
		
		msg = BBeam() # set up message
		# fill message
		msg.bbeam = 2
		msg.speed = ballSpeed
		# publish message
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing the second bbeam status!')

def main():
	rclpy.init()
	bbeam = BBeamAsync() # create a client	
	executor = MultiThreadedExecutor()
	executor.add_node(bbeam)
	
	executor.spin()

	bbeam.destroy_node() # stop when finished
	rclpy.shutdown()


if __name__ == '__main__':
	main()
