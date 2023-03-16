#!/usr/bin/env python
import rospy
from motor_control_2.msg import motor_output
from motor_control_2.msg import motor_input 
from motor_control_2.msg import set_point
from std_msgs.msg import Float32

#Setup parameters, vriables and callback functions here (if required)

class PIDController:
    def __init__(self):
        # get parameters from parameter server
        self.kp = rospy.get_param("/kp")
        self.ki = rospy.get_param("/ki")
        self.kd = rospy.get_param("/kd")
        # initialise variables
        self.setpoint = 0.0

        self.error = 0.0
        self.error_sum = 0.0
        self.error_diff = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0
        self.controllerOutput = motor_input()
        self.prev_output = 0.0
        # setup publishers and subscribers
        self.motorInputPub = rospy.Publisher("/motor_input", motor_input, queue_size=10)
        self.motorOutputSub = rospy.Subscriber("/motor_output", motor_output, self.updateFromMotorOutput, queue_size=10)
        self.setpointSub = rospy.Subscriber("/set_point", set_point, self.updateSetpoint, queue_size=10)
        print("Controller Initialised")
        print("kp: " + str(self.kp) + " ki: " + str(self.ki) + " kd: " + str(self.kd))
        # performance variables
        self.sse = 0.0
        self.sae = 0.0
        self.startTime = 0.0
        self.settling_time = 0.0
        self.error_band = 2.0
        self.overshoot = 0.0
        self.prev_diff = 0.0


    
    def updateFromMotorOutput(self, output):
        time = rospy.get_time()
        # Check if this is the first time we have received a message
        if self.prev_time != 0:
            # Calculate the dt, error, error_sum (integral) and error_diff (derivative)
            # The dt is the time difference between messages
            dt = 0.02
            # Update the previous time for the next iteration
            self.prev_time = time
            # Calculate the error (difference between the setpoint and the output of the motor)
            self.error = self.setpoint - output.speed 
            # Calculate the error_sum (integral) with trapezoidal integration
            self.error_sum += (self.error + self.prev_error) * dt / 2
            # Calculate the error_diff (derivative) (actual error - previous error divided by dt)
            self.error_diff = (self.error - self.prev_error) / dt
            # Update the previous error for the next iteration
            self.prev_error = self.error

            # Calculate the performance variables
            self.sse += self.error**2
            self.sae += abs(self.error)
            # Check if it has passed the setpoint (error has different sign to setpoint)
            if self.error * self.setpoint < 0:
                # Check if the error difference has changed sign
                if self.error_diff * self.prev_diff < 0:
                    if self.error > self.overshoot:
                        self.overshoot = self.error
                    # Check if the error is within the error band
                    if abs(self.error) < self.error_band and self.settling_time == 0:
                        self.settling_time = rospy.get_time() - self.startTime
                self.prev_diff = self.error_diff
            self.controllerOutput.value = self.get_output()
            self.controllerOutput.time = rospy.get_time()
            self.motorInputPub.publish(self.controllerOutput)
        else:
            self.prev_time = time
        
        
        #print("sed tpoint", self.setpoint, "MotorOutput",  output.output, "Error", self.error, "MotorInput", self.controllerOutput.value)

    def get_output(self):
        # Calculate the output of the controller
        pidOutput = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        # Range limit the output of the controller
        pidOutput = pidOutput / 100
        pidOutput = pidOutput / 255
        if pidOutput > 1:
            pidOutput = 1.0
        elif pidOutput < -1:
            pidOutput = -1.0
        return pidOutput
            
  
    def stop(self):
        self.controllerOutput.value = 0
        self.controllerOutput.time = rospy.get_time()
        self.motorInputPub.publish(self.controllerOutput)
        print("Stopping controller")

    def updateSetpoint(self, setpoint):
        # Check if the setpoint has changed
        if self.setpoint != setpoint.value:
            print("Setpoint: " + str(self.setpoint) + ", Settling Time: " + str(self.settling_time) + ", Overshoot: " + str(self.overshoot) + ", SSE: " + str(self.sse) + ", SAE: " + str(self.sae))
            self.settling_time = 0
            self.overshoot = 0
            self.sse = 0
            self.sae = 0
            self.prev_diff = 0.0
            self.startTime = rospy.get_time()
            self.setpoint = setpoint.value
        #print("Setpoint Updated to: " + str(self.setpoint) + " at time: " + str(setpoint.time) )

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")

    rate = rospy.Rate(100)
    controller = PIDController()
    rospy.on_shutdown(controller.stop)
    errorPub = rospy.Publisher("/error", Float32, queue_size=10)
    #Setup Publishers and subscribers here
    
    
                          
    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():
        errorPub.publish(controller.error)
        rate.sleep()

