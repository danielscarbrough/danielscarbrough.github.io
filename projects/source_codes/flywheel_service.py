# import libraries
import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node

from system_interfaces.srv import MoveFlywheels

import time

# defining pins
SERVO = 19 # servo

# stepper motor for turnstile
DIR2 = 21
STEP2 = 20

# dc motors for flywheels
IN1 = 14
IN2 = 15
IN3 = 5
IN4 = 6
EN1 = 13
EN2 = 18

# define HIGH/LOW constants
HIGH = 1
LOW = 0

# define the pwm duty cycles
MIN_DUTY = 1.5
MAX_DUTY = 22

# gpio setup
GPIO.setmode(GPIO.BCM)

# servo setup
GPIO.setup(SERVO,GPIO.OUT)
servo = GPIO.PWM(SERVO, 50)
servo.start(0)

# turnstile setup
GPIO.setup(DIR2,GPIO.OUT)
GPIO.setup(STEP2,GPIO.OUT)
GPIO.output(DIR2, HIGH) # initial direction

# flywheels setup
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
GPIO.setup(EN1,GPIO.OUT)
GPIO.setup(EN2,GPIO.OUT)
enable1 = GPIO.PWM(EN1,490)
enable2 = GPIO.PWM(EN2,490)

# LOW/HIGH = CCW
# HIGH/LOW = CW
# HIGH/HIGH OR LOW/LOW = OFF

# start with all flywheels off
GPIO.output(IN1, LOW)
GPIO.output(IN2, LOW)
GPIO.output(IN3, LOW)
GPIO.output(IN4, LOW)

# start flywheel pwm
enable1.start(0)
enable2.start(0)

class Flywheels(Node):

	# initialization function
	def __init__(self):
		super().__init__('flywheels')
		self.srv = self.create_service(MoveFlywheels, 'move_flywheels', self.runFlywheels)
		self.get_logger().info('Flywheels are online!')

	# function that moves the flywheels and turnstile
	def runFlywheels(self, request, response):
		
		# change the flywheels to run the appropriate speed
		enable1.ChangeDutyCycle(100)
		enable2.ChangeDutyCycle(100)
		
		if (request.direction == 0): # determines direction of the flywheels, this direction is for ball retreival
			self.get_logger().info('Running the flywheels backwards to retrieve the golf ball!') # run the flywheels backwards
			
			# set the flywheels to backwards
			GPIO.output(IN1, HIGH)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, HIGH)

			for i in range(0, request.balls): # expect this many golf balls
				time.sleep(5) # wait some given time
				# TURNSTILE
				self.get_logger().info('Moving turnstile for the next golf ball!')# move the turnstile 
				GPIO.output(DIR2, HIGH)
				for i in range(0, 66): # 200 steps for a full revolution, 50 steps for 90 degrees
					GPIO.output(STEP2, HIGH)
					time.sleep(0.01)
					GPIO.output(STEP2, LOW)
					time.sleep(0.01)

			# stop running the flywheels
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, LOW)
			
			self.get_logger().info('Flywheels finished!')	
			return response
		else: # direction for shooting the balls back to the user
			self.get_logger().info('Running the flywheels forward to launch out %d golf ball(s)!' % request.balls) # run the flywheels forward

			# set flywheels to forward
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, HIGH)
			GPIO.output(IN3, HIGH)
			GPIO.output(IN4, LOW)

			# TURNSTILE -- DO IT ONCE TO GET BACK TO THE LATEST BALL
			GPIO.output(DIR2, LOW) # change the direction
			self.get_logger().info('Moving turnstile for the next golf ball!')# move the turnstile 
			for i in range(0, 50): # 200 steps for a full revolution, 50 steps for 90 degrees
				GPIO.output(STEP2, HIGH)
				time.sleep(0.01)
				GPIO.output(STEP2, LOW)
				time.sleep(0.01)

			for i in range(0,request.balls): # run for the number of golf balls
				# SERVO
				self.get_logger().info('Running the servo to kick the golf ball!') # kick out the golf ball
				servo.ChangeDutyCycle(int((90 - 0) * (MAX_DUTY - MIN_DUTY) / 180 + MIN_DUTY))
				time.sleep(0.4)
				servo.ChangeDutyCycle(int((0 - 0) * (MAX_DUTY - MIN_DUTY) / 180 + MIN_DUTY))
				time.sleep(2)
				
				# don't turn the turnstile if there aren't any other balls to kick out
				if (request.balls) != 1:
					# TURNSTILE -- TURN IN THE OPPOSITE DIRECTION AS COLLECTION TO GET LATEST BALL
					GPIO.output(DIR2, LOW)
					self.get_logger().info('Moving turnstile for the next golf ball!')# move the turnstile 
					for i in range(0, 50): # 200 steps for a full revolution, 50 steps for 90 degrees
						GPIO.output(STEP2, HIGH)
						time.sleep(0.01)
						GPIO.output(STEP2, LOW)
						time.sleep(0.01)

				time.sleep(3) # do we really need to wait this long?
				
			# turn off the flywheels
			GPIO.output(IN1, LOW)
			GPIO.output(IN2, LOW)
			GPIO.output(IN3, LOW)
			GPIO.output(IN4, LOW)
			
			self.get_logger().info('Flywheels finished!')				
			return response

def main():
	rclpy.init()
	flywheels = Flywheels()
	rclpy.spin(flywheels)
	rclpy.shutdown()
	GPIO.cleanup()

if __name__ == '__main__':
	main()
