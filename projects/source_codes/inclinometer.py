# ros libraries
import sys
import rclpy

import time
import math

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from system_interfaces.msg import Incline

import Adafruit_ADXL345

class InclineAsync(Node):

	# initialization function
	def __init__(self):
	
		super().__init__('incline_async')
		
		self.publisher_ = self.create_publisher(Incline, 'incline', 10) # publisher that says how many golf balls and number of putts in the session
		self.timer = self.create_timer(0.5, self.check_incline) # timer to run break-beam code repeatedly
		
		self.accel = Adafruit_ADXL345.ADXL345(address=0x53, busnum=1)
		
		self.get_logger().info('Inclinometer is online!')	

		self.prevLeds = [] # list that stores which LEDs were on before, so the data is not sent if it is not new

	def check_incline(self):

		leds = []

		# get readings from accelerometer
		x,y,z = self.accel.read()

		# Convert to angles using trigonometry
		roll = math.atan2(y, z) * 180 / math.pi;
		pitch = math.atan2(-x, math.sqrt(y * y + z * z)) * 180 / math.pi;

		if (roll < -1.0):
			# Turn on LED 3 for roll less than -1 degrees
			leds.append(3)    
		if (roll > 1.0):
			# Turn on LED 5 for roll greater than 1 degrees
			leds.append(5)

		# Pitch LED control is LEFT AND RIGHT
		if (pitch < -1.0):
			# Turn on LED 6 for pitch less than -1 degrees
			leds.append(6)      
		if (pitch > 1.0):
			# Turn on LED 4 for pitch greater than 1 degrees
			leds.append(4)

		if (roll > -1.0 and roll < 1.0 and pitch > -1.0 and pitch < 1.0):
			# Turn on LED 7 for neutral
			leds.append(7)


		if (leds != self.prevLeds): # don't send an update if there's nothing new
			msg = Incline() # set up message
			msg.new_inclines = leds
			self.publisher_.publish(msg)       

		self.prevLeds = leds # set the current leds on to be the previous ones for the next iteration

def main():
	rclpy.init()

	inclinometer = InclineAsync()
	executor = MultiThreadedExecutor()
	executor.add_node(inclinometer)
	
	executor.spin()

	display.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
