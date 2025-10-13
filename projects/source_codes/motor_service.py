import rclpy
from rclpy.node import Node

from system_interfaces.srv import MoveDistance
from system_interfaces.srv import RotateDistance
from rclpy.executors import MultiThreadedExecutor

import time
import RPi.GPIO as GPIO

import math

# constants for the motor and wheels
DIAMETER = 3.125 # in inches
RPM = 160

# defining pins for the dc drive motors
IN1 = 24
IN2 = 23
IN3 = 16
IN4 = 12

# CURRENTLY NOT USING ENABLE TO CHANGE SPEED
#EN1 = 
#EN2 = 

# define HIGH/LOW constants
HIGH = 1
LOW = 0

# gpio setup
GPIO.setmode(GPIO.BCM)

GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
#GPIO.setup(EN1,GPIO.OUT)
#GPIO.setup(EN2,GPIO.OUT)
#enable1 = GPIO.PWM(EN1,490)
#enable2 = GPIO.PWM(EN2,490)

# LOW/HIGH = CCW
# HIGH/LOW = CW
# HIGH/HIGH OR LOW/LOW = OFF
 
class Motors(Node):

	# initialization function	
	def __init__(self):
		super().__init__('motors')
		self.srv = self.create_service(MoveDistance, 'move_distance', self.moveMotors)
		self.srv = self.create_service(RotateDistance, 'rotate_distance', self.rotateMotors)
		self.get_logger().info('Motors are online!')

	# function for the robot to rotate in place, used for ball collection and the stimpmeter test
	def rotateMotors(self, request, response):
	
		degrees = int(request.distance) # get the amount wanted to turn
		self.get_logger().info('Rotating %d degrees!' % degrees)
		
		# turn on the motors briefly in opposite directions
		if (degrees > 0): # one direction
			GPIO.output(IN1, HIGH)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, HIGH)
			GPIO.output(IN4, LOW)
		else: # reverse direction
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, HIGH)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, HIGH)

		time.sleep(abs(degrees)/10) # maybe change later, need to check how much the robot actually turns

		# turn the motors off
		GPIO.output(IN1, LOW)
		GPIO.output(IN2, LOW)
		GPIO.output(IN3, LOW)
		GPIO.output(IN4, LOW)
		return response

	# function for the dc motors to drive the motor
	def moveMotors(self, request, response):
		self.get_logger().info('Running the motors to move %d feet!' % (int(request.distance)))
		
		if (request.distance > 0): # if the robot needs to move forwards (forwards defined as AWAY from the user)

			# calculate the time needed to move the set distance			
			runtime = float(request.distance / ((math.pi*DIAMETER / 12) * RPM / 60))
			self.get_logger().info('to drive %d feet forward, the drive motors are required to run for %f seconds' % (request.distance, runtime))

			# move the distance by running for a set amount of time
			GPIO.output(IN1, HIGH)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, HIGH)

			time.sleep(runtime)

			# turn motors off
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, LOW)
		
		else: # move the robot backwards
			# calculate the time needed to move the set distance			
			runtime = abs(float(request.distance / ((math.pi*DIAMETER / 12) * RPM / 60)))
			self.get_logger().info('to drive %d feet backwards, the drive motors are required to run for %f seconds' % (abs(request.distance), runtime))

			# move the distance by running for a set amount of time
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, HIGH)
			GPIO.output(IN3, HIGH)
			GPIO.output(IN4, LOW)

			time.sleep(runtime)
			
			# turn motors off
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, LOW)
			
			if (request.comeback != 0): # comeback if its just picking up a ball or something
			    time.sleep(3) # wait some amount of time
			    self.get_logger().info('to drive %d feet forward, the drive motors are required to run for %f seconds' % (abs(request.distance), runtime))

			# move the distance by running for a set amount of time
			GPIO.output(IN1, HIGH)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, HIGH)

			time.sleep(runtime)

			# turn motors off
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, LOW)

		self.get_logger().info('motors finished!')
		return response

def main():
	rclpy.init()
	motors = Motors()
	executor = MultiThreadedExecutor()
	executor.add_node(motors)
	
	executor.spin()

	motors.destroy_node()
	rclpy.shutdown()
	GPIO.cleanup()

if __name__ == '__main__':
	main()
