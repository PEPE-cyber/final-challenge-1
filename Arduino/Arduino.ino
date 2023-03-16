#include <ros.h>
#include <motor_control_2/motor_input.h>
#include <motor_control_2/motor_output.h>

// Define the output pins
#define dirPin1 8
#define dirPin2 9
#define pwmPin  10

// Define the input pins
#define encoderPinA 2
#define encoderPinB 3


long previousTime = 0;
int pulses = 0;
float speed = 0;
String direction = "stopped";
// Function to get the speed
void encoderInterruptEncoder(){
 // Get the current time
 long currentTime = micros();
 // Calculate the time difference
 long timeDiff = currentTime - previousTime;
 // Calculate the speed Radians per second
 speed = 6.2831853072 * 1000000.0 / (timeDiff * 385.0);// 11 pulses per motor turn and a 35:1 gearbox ratio
 // Update the previous time
 previousTime = currentTime;
 // Update the number of pulses depending on the direction
 if (digitalRead(encoderPinB) == LOW){
   pulses--;
   direction = "counterclockwise";
   speed = -speed;
 } else {
   pulses++;
   direction = "clockwise";
 }
}


ros::NodeHandle  nh;

// Function callback for the subscriber (updates the pwm value)
void motorInputCb( const motor_control_2::motor_input& motor_input){
  // Get the pwm value from the message
  float receivedValue = motor_input.value; 
  // Map the value to the range 0-255
  int pwmValue = abs(receivedValue) * 255;
  // map(abs(receivedValue), 0, 1, 0, 255);
  // Set the value to the pwm pin
  analogWrite(pwmPin, pwmValue);
  // Set the direction of the motor
  if (receivedValue < 0){
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    digitalWrite(13, LOW);
  } 
  
}

// Create the subscriber to motor_input topic
ros::Subscriber<motor_control_2::motor_input> sub("motor_input", motorInputCb);

// Create the message to publish
motor_control_2::motor_output motor_output;
// Define the publisher
ros::Publisher pub("/motor_output", &motor_output);


void setup()
{
  // Define the pins as output
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(13, OUTPUT);
  // Define the pins as input
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  // Create an interrupt for the encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderInterruptEncoder, RISING);
  // Initialize the node
  nh.initNode();
  // Create a publisher
  nh.advertise(pub);
  // Subscribe to the topic
  nh.subscribe(sub);
}

const int samplingPeriod = 2; // ms
long previousMillis = 0;
long currentMillis = 0;
float previousSpeed = 0;
void loop()
{
  
  currentMillis = millis();
  // If the sampling period has passed, publish the speed
  if (currentMillis - previousMillis > samplingPeriod) {
    float avg = (speed + previousSpeed) / 2;
    previousSpeed = speed;
    // Calculate the speed (RPS)
    motor_output.speed = speed;
    int len = direction.length() + 1;
    char buf[len];
    direction.toCharArray(buf, len);
    motor_output.direction = buf;
    float revolutions = pulses / (11 * 35);
    motor_output.rev = revolutions;
    pub.publish(&motor_output); 
  }
  nh.spinOnce();
}