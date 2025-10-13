# ros libraries
import sys
import rclpy

from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


from system_interfaces.msg import BBeam
from system_interfaces.msg import CameraMsg
from system_interfaces.msg import Display

from system_interfaces.srv import MoveDistance
from system_interfaces.srv import CameraData
from system_interfaces.srv import MoveFlywheels
from system_interfaces.srv import RotateDistance # wheel motors request
from system_interfaces.srv import CameraCheck
from system_interfaces.srv import ListenerSrv

# import necessary packages
from tkinter import *
from tkinter import ttk
from tkinter.font import Font
import tkinter as tk
from PIL import ImageTk, Image
import time
import sv_ttk
import math

from subprocess import call

import threading

class DisplayAsync(Node):

	
	# function that requests the dc motors to drive a certain distance			
	def send_request(self, distance, comeback):
		self.motorReq.distance = distance # defines the distance to move
		self.motorReq.comeback = comeback # defines whether the robot should move back the same distance		
		return self.cli.call_async(self.motorReq)

	# function that requests the flywheels and turnstile to move
	def send_flywheels_request(self, direction, balls):
		self.flywheelsReq.direction = direction # defines whether the flywheels should collect or spit out the balls
		self.flywheelsReq.balls = balls # the number of golf balls to expect
		future = self.flywheelsCli.call_async(self.flywheelsReq)
		rclpy.spin_until_future_complete(self, future) # spin until there's a result

	# function that requests the dc motors to spin in place
	def send_rotate_request(self, distance):
		self.rotateReq = RotateDistance.Request() # create the drive motors
		self.rotateReq.distance = distance # defines how many degrees for the robot to spin
		return self.rotateCli.call_async(self.rotateReq)

	# initialization function
	def __init__(self):
	
		super().__init__('display_async')

		# callback groups
		motor_cb_group = MutuallyExclusiveCallbackGroup()
		publisher_timer_cb_group = MutuallyExclusiveCallbackGroup()
		client_cb_group = MutuallyExclusiveCallbackGroup()
		flywheels_cb_group = MutuallyExclusiveCallbackGroup()
		position_cb_group = MutuallyExclusiveCallbackGroup()
		bbeam_cb_group = MutuallyExclusiveCallbackGroup()
		camera_cb_group = MutuallyExclusiveCallbackGroup()		
		listener_cb_group = MutuallyExclusiveCallbackGroup()		
		
		# clients
		self.cli = self.create_client(MoveDistance, 'move_distance', callback_group=motor_cb_group) # client to ask for the motors to move
		self.flywheelsCli = self.create_client(MoveFlywheels, 'move_flywheels',callback_group=flywheels_cb_group) # client to move the flywheel motors
		self.rotateCli = self.create_client(RotateDistance, 'rotate_distance',callback_group=motor_cb_group) # client to move drive wheel motors
		self.positionCli = self.create_client(CameraCheck, 'ball_check', callback_group=position_cb_group) # client to request position from the camera
		self.cameraCli = self.create_client(CameraData, 'ball_reset', callback_group=camera_cb_group) # client to request position from the camera
		self.listenerCli = self.create_client(ListenerSrv, 'listener', callback_group=listener_cb_group) # client to request position from the camera
		
		# publisher
		self.publisher_ = self.create_publisher(Display, 'display_data', 10, callback_group=publisher_timer_cb_group) # publisher that says how many golf balls and number of putts in the session

		# define variables used everywhere
		self.bbeam = 0
		self.speed = 0
		self.numLong = 0
		self.numShort = 0
		self.numGood = 0
		self.totalPutts = 0
		self.numPutts = 0
		self.numBalls = 0
		
		self.greenCoord = 0
		self.holeCoord = 0

		self.greenCenter = 200,200
		self.greenRadius = 320 # ~5 ft radius
		self.greenRadius = 192 # ~3 ft radius

		self.holeRadius = 8 # 10 = 4.25 inches
		self.ballRadius = self.holeRadius * 0.4 # approximate scale of the ball compared to the hole
		self.distanceScale = 16/4.25 # 1 inch according to system

		# START OF ROOT

		self.root = Tk() # define a root window
		
		self.root.bind('<Control-c>', quit) # set up exit keyboard interrupt

		# FONTS
		text_font = Font(family="Helvetica", size=30) # set up font of choice
		text_font_small = Font(family="Helvetica", size=26) 
		text_font_bold = Font(family="Helvetica", size=26, weight="bold") 
		text_font_small_bold = Font(family="Helvetica", size=26, weight="bold") 

		self.root.attributes("-fullscreen", True) # set the window to fullscreen
		
		bgImage = '/home/on-par/onpar_ws/green.png' # set up a variable to store the image
		bg= PhotoImage(file= bgImage) # input the image
		self.bgLabel= Label(self.root, i=bg) # use the image for a label
		self.bgLabel.place(x = 0, y = 0) # place the label
		self.bgLabel.lower() # lower to background

		while not self.flywheelsCli.wait_for_service(timeout_sec=1.0): # wait for the flywheel service to come online
			self.get_logger().info('waiting for the flywheels to come online...')
		while not self.cli.wait_for_service(timeout_sec=1.0): # wait for the flywheel service to come online
			self.get_logger().info('waiting for the motors to come online...')
		while not self.positionCli.wait_for_service(timeout_sec=1.0): # wait for the flywheel service to come online
			self.get_logger().info('waiting for the camera to come online...')

		# END OF ROOT
		
		# START OF SETUP FRAME

		self.setup = ttk.Frame(self.root, padding=25) # creates a frame to hold data
		self.setup.grid() # creates a grid on the frame to organize labels
		self.setup.place(x = 0, y = 0) # place the frame

		self.welcomeTitle = Label(self.setup, text="on PAR : Putting Aid Robot\n", font=text_font_small_bold).grid(column=0, row=0)

		# set up sliders
		Label(self.setup, text="Number of balls:", font=text_font).grid(column=0, row=1)
		self.ballSlider = Scale(self.setup, length=200, from_=1, to=3, orient=HORIZONTAL, font=text_font)
		self.ballSlider.grid(column=1, row=1)

		Label(self.setup, text="Number of putts:", font=text_font).grid(column=0, row=2)
		self.puttSlider = Scale(self.setup, length=200, from_=1, to=12, orient=HORIZONTAL, font=text_font)
		self.puttSlider.grid(column=1, row=2)

		# label for putting distances
		self.welcome = Label(self.setup, text="\nPutting distance (in ft):\n", font=text_font).grid(column=0, row=3)

		# set up options for putting distances
		self.puttDist = StringVar(self.root)
		self.puttingDistances = [5, 10, 15, 20]
		self.puttDist.set(self.puttingDistances[0])

		self.distMenu = OptionMenu(self.setup, self.puttDist, *self.puttingDistances)
		self.distMenu.config(font=text_font)
		self.distMenu.grid(column=1, row=3)

		self.menu = self.setup.nametowidget(self.distMenu.menuname)  # Get menu widget.
		self.menu.config(font=text_font)  # Set the dropdown menu's font

		self.startSession = tk.Button(self.setup, text="Start session", command=self.start, font=text_font).grid(column=0, row=6) # button to start a session
		self.stimpmeter = tk.Button(self.setup, text="Stimpmeter test", command=self.stimp, font=text_font).grid(column=1, row=6) # button to start a stimpmeter test
		self.gSpeed = Label(self.setup, text="Not calculated", font=text_font) # where the calculated green speed will be stored
		self.gSpeed.grid(column=1, row=7)		

		# power button -- shuts off the entire Pi
		self.powerOff = tk.Button(self.setup, text="Shutdown", command=self.turnOff, font=text_font_small, fg = "red")
		self.powerOff.grid(column=1, row=0)

		# END OF SETUP FRAME

		# START OF RUNTIME FRAME

		self.runtime = ttk.Frame(self.root, padding=30, height=500, width=900) # creates a frame to hold data
		self.runtime.grid() # creates a grid on the frame to organize labels
		self.runtime.place(x = 0, y = 0) # place the frame
		self.runtime.grid_propagate(False)

		self.statsTitle = Label(self.runtime, text="Session Statistics", font=text_font_bold)
		self.statsTitle.grid(column=0, row=0)

		# set labels for good, short, and long putts, the number of balls collected, and the accuracy
		self.goodLabel = Label(self.runtime, text=f"Good Putts = {self.numGood}", font=text_font_small)
		self.goodLabel.grid(column=0, row=1, pady=10)
		self.shortLabel = Label(self.runtime, text=f"Short Putts = {self.numShort}", font=text_font_small)
		self.shortLabel.grid(column=0, row=2, pady=10)
		self.longLabel = Label(self.runtime, text=f"Long Putts = {self.numLong}", font=text_font_small)
		self.longLabel.grid(column=0, row=3, pady=10)
		self.collectedLabel = Label(self.runtime, text="Balls collected = N/A", font=text_font_small)
		self.collectedLabel.grid(column=0, row=5, pady=10)
		self.accuracyLabel = Label(self.runtime, text="Accuracy = N/A", font=text_font_small)
		self.accuracyLabel.grid(column=0, row=4, pady=10)

		# button to preemptively end a session -- sends to final results screen
		self.endSession = tk.Button(self.runtime, text="End session", command=self.results, font=text_font)
		self.endSession.grid(column=0, row=6)
		
		# button to start a new session -- sends back to the setup screen
		self.startNewSession = tk.Button(self.runtime, text="New session?", command=self.restart, font=text_font)
		self.startNewSession.grid(column=0, row=6)
		self.startNewSession.grid_forget()

		#Create a canvas object
		self.c= Canvas(self.runtime, height=400, width=800)
		self.c.grid()
		self.c.place(x = 350, y = 0)
		
		# set up the green map
		self.greenCoord = self.greenCenter[0] - self.greenRadius, self.greenCenter[1] - self.greenRadius, self.greenCenter[0] + self.greenRadius, self.greenCenter[1] + self.greenRadius
		self.holeCoord = self.greenCenter[0] - self.holeRadius, self.greenCenter[1] - self.holeRadius, self.greenCenter[0] + self.holeRadius, self.greenCenter[1] + self.holeRadius
		self.c.create_rectangle(self.greenCoord, fill="dark green") # backdrop
		self.c.create_oval(self.greenCoord, fill="forest green", outline = "forest green") # green
		self.c.create_arc(self.greenCoord, start=135, extent=270, outline="dark olive green", fill="dark olive green")
		self.c.create_oval(self.holeCoord, fill="black") # hole

		# create 6 inch intervals on the green for clarity
		for dist in range(1, 9):
			radiiCoord = ((self.greenCenter[0] - dist*self.distanceScale*6), (self.greenCenter[1] - dist*self.distanceScale*6), (self.greenCenter[0] + dist*self.distanceScale*6), (self.greenCenter[1] + dist*self.distanceScale*6))
			self.c.create_oval(radiiCoord, outline="lime green") # green

		self.runtime.lower()
		
		# END OF RUNTIME FRAME

		sv_ttk.set_theme("dark") # fancy mode
		self.root.mainloop()

	def update_screen(self):

		# get most recent data from the listener thread
		self.listenerReq = ListenerSrv.Request()
		self.listenerReq.done = 0 # tells the listener the session isn't done
		future = self.listenerCli.call_async(self.listenerReq)
		rclpy.spin_until_future_complete(self, future) # spin until there's a result	
		updatedData = future.result() # set the result		

		# update the statistics based on the return values from the listener thread
		self.numGood = updatedData.good_putts
		self.numShort = updatedData.short_putts
		self.numLong = updatedData.long_putts
		self.totalPutts = self.numGood + self.numShort + self.numLong

		# calculate the accuracy
		if (self.totalPutts != 0):
			self.accuracy = round(self.numGood/self.totalPutts * 100,2)
		else:
			self.accuracy = 0
			
		# update the display
		self.goodLabel.config(text=f"Good Putts = {self.numGood}")
		self.shortLabel.config(text=f"Short Putts = {self.numShort}")
		self.longLabel.config(text=f"Long Putts = {self.numLong}")
		self.collectedLabel.config(text=f"Balls collected = {self.totalPutts}/{self.numPutts}")
		self.accuracyLabel.config(text=f"Accuracy = {self.accuracy}%")

		ball_font = Font(family="verdana bold", size=11) # set up font of choice

		# fill a list with the camera data from the listener
		ballData = updatedData.ball_data
		ballData = [i for i in zip(*[iter(ballData)]*3)] # take the list and make it into a list of tuples

		for ball in ballData: # for all balls detected
			ballLocation = ball # set the location to the current ball
				
			ballLocationScaled = ballLocation[0]*self.distanceScale, ballLocation[1]*self.distanceScale # scale the distance to look more accurate

			ballCoord = self.greenCenter[0] + ballLocationScaled[0] - self.ballRadius, self.greenCenter[1] - ballLocationScaled[1] - self.ballRadius, self.greenCenter[0] + ballLocationScaled[0] + self.ballRadius, self.greenCenter[1] - ballLocationScaled[1] + self.ballRadius
			# described by starting from the center, moving to the location coordinates, then shifted by the radius

			textCoord = str(round(math.sqrt(ballLocation[0]**2 + ballLocation[1]**2)/12, 1)) + " ft"

			if (ballLocation[2]) == 1: # if 1, the ball went missing
				self.c.create_oval(ballCoord, fill="dim gray") # ball
				self.c.create_text(self.greenCenter[0] + ballLocationScaled[0], self.greenCenter[1] - ballLocationScaled[1] - 15, text=textCoord, fill="light grey", font=ball_font) # show coordinates
			else: # if the ball stopped in view
				self.c.create_oval(ballCoord, fill="white") # ball				
				self.c.create_text(self.greenCenter[0] + ballLocationScaled[0], self.greenCenter[1] - ballLocationScaled[1] - 15, text=textCoord, fill="white", font=ball_font) # show coordinates

		# check if the golfer has completed all their putts for the session
		if (self.totalPutts == self.numPutts and self.numPutts != 0):
			self.results()
			self.listenerReq.done = 1 # send one more request to the listener, this one telling it the session is complete
			self.listenerCli.call_async(self.listenerReq)
			self.get_logger().info('session complete!')            				
		else: # if the golfer still has more putts, start the function over and keep updating the display
			self.root.after(1000, self.update_screen)

	# TKINTER FUNCTIONS

	# simple function to turn off the Pi
	def turnOff(self):
		#exit()
		call("systemctl poweroff", shell=True)

	# function that controls the stimpmeter test
	def stimp(self):
	
		distance = []
		self.gSpeed.config(text = f"stimpmeter active...")	
		
		# collect golf balls into the system by running the flywheels backwards
		self.flywheelsReq = MoveFlywheels.Request() # create the flywheels
		self.send_flywheels_request(0, 3)
		
		rotations = [1, -2, 1] # rotate 1 degree first, then 2 degrees the other direction, then 1 degree back
		# once three golf balls have been collected		
		for n in range(0,3):
			self.send_flywheels_request(1, 1) # eject one ball out with the servo and flywheels	
			self.send_rotate_request(rotations[n]) # rotate slightly so the ball doesn't hit the others
			# repeat for all golf balls

		# use camera or breakbeam to determine the distance of the golf balls
		self.positionReq = CameraCheck.Request() # create the updater for the display		
		future = self.positionCli.call_async(self.positionReq)
		rclpy.spin_until_future_complete(self, future) # spin until there's a result				
		response = future.result()
		# store all results
		for balls in response.ball_distance:
			distance.append(balls / 12)
		
		if len(distance) != 0: # if the camera detected balls that were launched out
			# calculate the green speed by averaging the distances
			gSpeedValue = round(sum(distance) / len(distance),1)
			# display results
			self.gSpeed.config(text = f"Green speed = {gSpeedValue}") # update the setup screen
		else: # if no golf balls were able to be detected
			self.get_logger().info('Stimpmeter test failed!')
			self.gSpeed.config(text = f"Failed, try again") # tell the user to try the test again
			gSpeedValue = 9.0			

	# functions to switch between menus at the start from the setup to the runtime menu
	def start(self):
		
		# get the distance for the robot to move, the number of balls the golfer has, and the number of putts they want for the session
		self.dist = int(self.puttDist.get())
		self.numBalls = self.ballSlider.get()
		self.numPutts = self.puttSlider.get() 
		
		# calculate accuracy	
		if (self.totalPutts != 0):
			self.accuracy = round(self.numGood/self.totalPutts * 100,2)
		else:
			self.accuracy = 0
		
		# reset runtime frame
		self.collectedLabel.grid(column=0, row=5)
		self.statsTitle.config(text="Session Statistics")

		self.startNewSession.grid_forget()
		self.endSession.grid(column=0, row=6)
			 
		# update the labels with the most recent data	 
		self.goodLabel.config(text=f"Good Putts = {self.numGood}")
		self.shortLabel.config(text=f"Short Putts = {self.numShort}")
		self.longLabel.config(text=f"Long Putts = {self.numLong}")
		self.collectedLabel.config(text=f"Balls collected = {self.totalPutts}/{self.numPutts}")
		self.accuracyLabel.config(text=f"Accuracy = {self.accuracy}%")

		# set up/reset the green map
		self.c.delete('all')
		greenCoord = self.greenCenter[0] - self.greenRadius, self.greenCenter[1] - self.greenRadius, self.greenCenter[0] + self.greenRadius, self.greenCenter[1] + self.greenRadius
		self.holeCoord = self.greenCenter[0] - self.holeRadius, self.greenCenter[1] - self.holeRadius, self.greenCenter[0] + self.holeRadius, self.greenCenter[1] + self.holeRadius
		self.c.create_rectangle(self.greenCoord, fill="dark green") # backdrop
		self.c.create_oval(self.greenCoord, fill="forest green", outline = "forest green") # green
		self.c.create_arc(self.greenCoord, start=135, extent=270, outline="dark olive green", fill="dark olive green")
		self.c.create_oval(self.holeCoord, fill="black") # hole

		# create 6 inch intervals
		for dist in range(1, 9):
			radiiCoord = ((self.greenCenter[0] - dist*self.distanceScale*6), (self.greenCenter[1] - dist*self.distanceScale*6), (self.greenCenter[0] + dist*self.distanceScale*6), (self.greenCenter[1] + dist*self.distanceScale*6))
			self.c.create_oval(radiiCoord, outline="lime green") # green

		self.setup.lower()
		self.runtime.lift()

		# send display data to listener
		self.publisher_callback()

		# reset the camera data for a new session
		self.cameraReq = CameraData.Request() # create the updater for the display	
		self.cameraReq.reset = 3
		self.cameraCli.call_async(self.cameraReq)

		# start updating the display
		self.update_screen()

	# publisher that tells the break-beam the number of balls and putts to expect
	def publisher_callback(self):		
		self.get_logger().info('Sending startup info to the listener!')
		msg = Display() # set up message
		# fill message
		msg.num_balls = self.numBalls
		msg.num_putts = self.numPutts
		msg.distance = int(self.puttDist.get())
		
		# publish message
		self.publisher_.publish(msg)

	# results screen that tells the robot to go back to the user and show the results
	def results(self):
		# stop the update timer
		self.collectedLabel.grid_forget()
		
		self.endSession.grid_forget()
		self.startNewSession.grid(column=0, row=6)
		
		self.statsTitle.config(text="Final Results")
		
	# function that resets everything at the end of a session once the user hits the restart button
	def restart(self):
		self.setup.lift()
		self.runtime.lower()
		self.numLong = 0
		self.numShort = 0
		self.numGood = 0
		self.totalPutts = 0
		self.numPutts = 0
		self.numBalls = 0
		
def main():
	rclpy.init()

	display = DisplayAsync()
	executor = MultiThreadedExecutor()
	executor.add_node(display)
	
	executor.spin()

	display.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
