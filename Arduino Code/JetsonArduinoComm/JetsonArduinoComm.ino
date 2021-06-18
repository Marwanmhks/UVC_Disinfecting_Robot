#include <L298NX2.h>
#include <PID_v1.h> 
#include <ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//_________________________________________________________________                                   

//initializing all the variables
//timers for sub-main loop
#define LOOPTIME           100                //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;

const int PIN_ENCOD_A_MOTOR_LEFT = 3;             //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 19;             //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 2;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 18;              //B channel for encoder of right motor 

// Driver-Motor initialisation
const unsigned int EN_A = 4;                        
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;
const unsigned int IN1_B = 7;
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);   //Driver setting

// Robot specifications
const double radius = 0.0425;                 //Wheel radius, in m
const double wheeltrack = 0.272;              //wheeltrack, in m
const double rpm_to_pwm = 0.9107;             //Converts rpm to pwm
const double encoderppr = 225;                //Encoder pulses per revolution
const double speed_to_pwm = 202.4;              //m/s to pwm

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_handle_left = 0;                   //Desired left motor speed from handle_cmd function
double speed_req_left = 0;                   //Desired speed set point for left wheel in looptime in m/s
double speed_act_left = 0;                   //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                   //Command speed for left wheel in m/s 

double speed_handle_right = 0;                   //Desired right motor speed from handle_cmd function
double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 

const double max_speed = 1.25;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

//Variables to determine each cycle to get robot odometry position
double pos_left_mm =0;
double pos_right_mm = 0;
double pos_old_left_mm = 0;
double pos_old_right_mm = 0;
double pos_left_diff = 0;
double pos_right_diff = 0;
double pos_average_diff = 0;
double pos_total = 0;

// tf variables to broadcast
double y=0;                                   // position in y direction
double x=0;                                   // position in x direction
double theta = 0;                             //yaw angle in radians
                                                      
// PID Parameters
const double PID_left_param[] = { 0.6, 3.5, 0.05 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] ={ 0.6, 3.5, 0.05 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

volatile float pulses_left = 0;    //number of left encoder pulses in each looptime
volatile float pulses_right = 0;   //number of right encoder pulses in each looptime

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication 
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message
  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  if (angular_speed_req == 0){
  speed_handle_left = speed_req;                 //Calculate the required speed for the left motor to comply with commanded linear and angular speeds  speed_req_right = speed_req + angular_speed_req*(wheeltrack/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  speed_handle_right = speed_req;                //Calculate the required speed for the left motor to comply with commanded linear and angular speeds  speed_req_right = speed_req + angular_speed_req*(wheeltrack/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  }
  else if( speed_req > 0 && angular_speed_req >0){
    speed_handle_left = speed_req + 0.1;
    speed_handle_right = speed_req - 0.1;
  }
  else if(speed_req > 0 && angular_speed_req <0){
    speed_handle_left = speed_req - 0.1;
    speed_handle_right = speed_req + 0.1;
  }
  else if( speed_req ==0 && angular_speed_req > 0 ){
    speed_handle_left = speed_req + 0.1;
    speed_handle_right = -(speed_req + 0.1);
  }
  else if( speed_req ==0 && angular_speed_req < 0 ){
    speed_handle_left = -(speed_req + 0.1);
    speed_handle_right = speed_req + 0.1;
  }
  else if( speed_req < 0 && angular_speed_req >0){
    speed_handle_left = (speed_req + 0.1);
    speed_handle_right =  (speed_req - 0.1);
  }
  else if(speed_req < 0 && angular_speed_req <0){
    speed_handle_left = -(speed_req - 0.1);
    speed_handle_right = -(speed_req + 0.1);
  }
}

ros::NodeHandle nh;     //initialise ros node nh
char odom[] = "/odom";
char base_link[] = "/base_link";
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
nav_msgs::Odometry odom_msg;                                            //create an "odom_msg" ROS message
ros::Publisher odom_pub("odometry", &odom_msg);                         //create a publisher to ROS topic "odometry" using the "odom_msg" type
tf::TransformBroadcaster broadcaster; 

//__________________________________________________________________________

void setup() {
  nh.getHardware()->setBaud(500000);         //set baud for ROS serial communication
  nh.initNode();                            //init ROS node
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(odom_pub);                  //prepare to publish speed in ROS topic
  broadcaster.init(nh);
  
  //setting motor speeds to zero
  motors.setSpeedA(0);
  motors.stopA();
  motors.resetA();

  motors.setSpeedB(0);
  motors.stopB();
  motors.resetA();
 
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
    
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME){                                                                           // enter timed loop
    lastMilli = millis();
    
    if (abs(pulses_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pulses_left/encoderppr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate left wheel speed in m/s
      pulses_left = 0;                                                                 // reset pulses to 0 for next loop
    }
    
    if (abs(pulses_right) < 5){                                                         //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pulses_right/encoderppr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate right wheel speed in m/s
    pulses_right=0;
    }
    
    speed_req_left = speed_handle_left;
    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 // compute PWM value for left motor
    PWM_leftMotor = constrain((speed_cmd_left/rpm_to_pwm)/0.00405, -255, 255);

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      speed_req_left=0;
      }
    else if (speed_req_left == 0){                        //Stopping
      motors.setSpeedA(0);
      motors.stopA();
      motors.resetA();
    }
    else if (PWM_leftMotor > 0){
      motors.setSpeedA(abs(PWM_leftMotor));               //Going forward
      motors.resetA();
      motors.runA(L298N::FORWARD);
    }
    else {                                               //Going backward
      motors.setSpeedA(abs(PWM_leftMotor));
      motors.resetA();
      motors.runA(L298N::BACKWARD);
    }
    speed_req_right = speed_handle_right;
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
    PID_rightMotor.Compute();                                                 // compute PWM value for right motor
    PWM_rightMotor = constrain((speed_cmd_right/rpm_to_pwm)/0.00405, -255, 255); // 

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      speed_req_right = 0;
      }
    else if (speed_req_right == 0){                       //Stopping
      motors.setSpeedB(0);
      motors.stopB();
      motors.resetB();
      }
    else if (PWM_rightMotor > 0){                         //Going forward
      motors.setSpeedB(abs(PWM_rightMotor));
      motors.resetB();
      motors.runB(L298N::FORWARD);
    }
    else {                                                //Going backward
      motors.setSpeedB(abs(PWM_rightMotor));
      motors.resetB();
      motors.runB(L298N::BACKWARD);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      //Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

// Determining position
    pos_left_mm = (pos_left/encoderppr) * 2*PI*radius;
    pos_right_mm = (pos_right/encoderppr) * 2 * PI * radius;
    pos_left_diff = pos_left_mm - pos_old_left_mm;
    pos_right_diff = pos_right_mm - pos_old_right_mm;
    pos_old_left_mm = pos_left_mm;
    pos_old_right_mm = pos_right_mm;

    pos_average_diff = (pos_left_diff + pos_right_diff)/2;
    pos_total += pos_average_diff;

    if (speed_handle_left == speed_handle_right);
    else
    theta += ((pos_left_diff-pos_right_diff)/wheeltrack);    

    if (theta > PI)
    theta -= TWO_PI;
    if(theta < -(PI))
    theta += TWO_PI;

    y += pos_average_diff * sin(theta);
    x += pos_average_diff * cos(theta);

   publishOdometry(LOOPTIME);   //Publish odometry on ROS topic
 }
}

void publishOdometry(double time) {
  geometry_msgs::TransformStamped t;
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = x;       //robot position in X direction in mm
  t.transform.translation.y = y;       // robot position in Y direction in mm
  t.transform.translation.z = 0;
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = nh.now();      //timestamp for odometry data
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
  odom_msg.twist.twist.linear.x = speed_act_left;    //left wheel speed (in m/s)
  odom_msg.twist.twist.linear.y = speed_act_right;   //right wheel speed (in m/s)
  odom_msg.twist.twist.angular.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  odom_pub.publish(&odom_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)){
    pos_left++;
    pulses_left++;
  }
  else{
    pos_left--;
    pulses_left--;
  } 
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)){
    pos_right--;
    pulses_right--;
  }
  else{
    pos_right++;
    pulses_right++;
  }
}
