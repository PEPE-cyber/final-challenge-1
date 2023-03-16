#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from motor_control_2.msg import set_point

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios



# Funcion obtenida de https://github.com/ROBOTIS-GIT/turtlebot3/blob/66681b33749c44e7d9022253ac210ef2da7843a0/turtlebot3_teleop/nodes/turtlebot3_teleop_key
# Codigo original de Rushikesh Kamalapurkar (rlkamalapurkar)
def getKey():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



class InputGenerator:
    def __init__(self):
        # Get the parameters from the launch file
        self.freq = rospy.get_param("/frequency")
        self.amplitude = rospy.get_param("/amplitude")
        self.signal = rospy.get_param("/signal_type")
        # Setup the publisher on the set_point topic
        self.setpointPub = rospy.Publisher("set_point", set_point, queue_size=10)
        # Setup the set point value
        self.setpointValue = 0.0
        # Setup the time and previous time
        self.time =  rospy.get_time()
        self.prevTime = rospy.get_time()
        self.key = ''
        self.set_point = set_point()
        if not (self.signal == "step" or self.signal == "square" or self.signal == "sinusoidal"):
            rospy.loginfo("Using keyboard control. Press 'w' to increase the speed and 's' to decrease the speed. Press 'q' to quit.")
           
        self.x = 0

    def run(self):
        # Get the current time
        self.time =  rospy.get_time()
        # Determine the signal type
        if self.signal == "sinusoidal": 
            # Calculate the set point value with a sinusoidal function
            self.setpointValue = self.amplitude * (np.sin(2 * np.pi * self.freq * self.time)) 
        elif self.signal == "square": 
            # Calculate the set point value with a square function
            # Check if the time is greater than the previous time + 1/freq (the period). If it is, change the pwm value
            if  self.time - self.prevTime > ( 1 / self.freq):
                # If the set value is positive, make it negative and vice versa
                if self.setpointValue == self.amplitude:
                    self.setpointValue = -self.amplitude
                else:
                    self.setpointValue =  self.amplitude
                # Update the previous time for the next period
                self.prevTime =  self.time
                self.x += 1
                if  self.x > 10:
                    print("Test ended")

        elif self.signal == "step": 
            # Calculate the pwm value with a step function
            # Check if the time is less than the previous time + 5 seconds. If it is, set the pwm value to 0
            if  self.time < (self.prevTime + 5):
                self.setpointValue = 0
            # If the time has passed, set the pwm value to the amplitude (start the step)
            else:
                self.setpointValue = self.amplitude
        else:
            if self.key != 'q':
                self.key = getKey()
            if self.key == 'w':
                if self.setpointValue < 100  or True:
                    self.setpointValue += 5
            elif self.key == 's':
                if self.setpointValue > -100 or True:
                    self.setpointValue -= 5
            if self.key == 'q':
                rospy.signal_shutdown("Quit Motor")
                print("Quitting from keyboard")
           
                

        # Publish the motor input
        self.set_point.value = self.setpointValue
        self.set_point.time = self.time
        self.setpointPub.publish(self.set_point)

    def stop(self):
        # Stop the motor (publish 0)
        self.set_point.value = 0
        self.set_point.time = rospy.get_time()
        self.setpointPub.publish(self.set_point)
        print("Stopping motor")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Input")
    # Set the rate of the node
    rate = rospy.Rate(100)
    # Create an instance of the InputGenerator class
    input = InputGenerator()
    print("Input Genertor is Running")
    # Setup the shutdown function to stop the motor
    rospy.on_shutdown(input.stop)

    #Run the node
    while not rospy.is_shutdown():
        # Run the input generator (publish the pwm value)
        input.run()
        #Write your code here
        rate.sleep()


    