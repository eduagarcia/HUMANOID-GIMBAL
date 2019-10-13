#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <config.h>
#include <setup.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

//13200 -> -85
//8800 -> 0
//3000 -> 90
#define ZERO_YAW 8800
#define MIN_YAW 2350
#define MAX_YAW 13200

//4700 -> 0
//10300 -> 90
#define ZERO_PITCH 4700
#define MIN_PITCH 2350
#define MAX_PITCH 12000

void gimbal_callback(const std_msgs::Int32MultiArray& msg);
void set_position_yaw();
void set_position_pitch();
void printImuEvent();
void printServoPos();

uint16_t SAMPLERATE_DELAY_MS = 100;
TwoWire myWire = TwoWire(2);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &myWire);

//PWM values YAW AND PITCH
int32_t pos_yaw = ZERO_YAW;
int32_t pos_pitch = ZERO_PITCH;

//IMU values
double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
double error_yaw = 0, error_pitch = 0;
double delta = 2;
//ros::NodeHandle nh;
//time_t last_spin;
//ros::Subscriber<std_msgs::Int32MultiArray> sub("Bioloid/gimbal_pos", gimbal_callback);

//Debug Mode
int debug = 1;

void setup() {
  setup_timer();
  setup_pin_modes();
  setup_serial_baud_rate();

  //Move motors to the zero
  pwmWrite(SERVO_YAW_PWM, ZERO_YAW);
  pwmWrite(SERVO_PITCH_PWM, ZERO_PITCH);
  delay(1000);
  
  //nh.getHardware()->setBaud(BAUD_RATE_CONTROL);
  //nh.initNode();
  //nh.subscribe(sub);
  //last_spin = millis();

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SERIAL_DEBUG.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
  delay(1000);
}

void loop() {

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

  x = event.orientation.x;
  y = event.orientation.y;
  z = event.orientation.z;

  if(x > 180) {
    x = x-360;
  }

  error_yaw = x;
  error_pitch = y;

  if(error_yaw>delta || error_yaw<-delta) {
    pos_yaw += (int)error_yaw;
    set_position_yaw();
  }

  if(error_pitch>delta || error_pitch<-delta) {
    pos_pitch -= (int)error_pitch;
    set_position_pitch();
  }
  
  if (debug){
    printImuEvent();
    printServoPos();
  }
  
  delay(SAMPLERATE_DELAY_MS);
}

void gimbal_callback(const std_msgs::Int32MultiArray& msg) {
  if (msg.data_length != NUM_SERVOS + 1)
    return;
}

void set_position_yaw() {
  if(pos_yaw > MAX_YAW) {
    pos_yaw = MAX_YAW;
  } else if (pos_yaw < MIN_YAW) {
    pos_yaw = MIN_YAW;
  }

  pwmWrite(SERVO_YAW_PWM, pos_yaw);
}

void set_position_pitch() {
  if(pos_pitch > MAX_PITCH) {
    pos_pitch = MAX_PITCH;
  } else if (pos_pitch < MIN_PITCH) {
    pos_pitch = MIN_PITCH;
  }

  pwmWrite(SERVO_PITCH_PWM, pos_pitch);
}


void printImuEvent() {
  SERIAL_DEBUG.println();
  SERIAL_DEBUG.print("IMU");

  SERIAL_DEBUG.print(": x= ");
  SERIAL_DEBUG.print(x);
  SERIAL_DEBUG.print(" | y= ");
  SERIAL_DEBUG.print(y);
  SERIAL_DEBUG.print(" | z= ");
  SERIAL_DEBUG.println(z);
}

void printServoPos() {
  SERIAL_DEBUG.println();
  SERIAL_DEBUG.print("SERVO");

  SERIAL_DEBUG.print(": YAW= ");
  SERIAL_DEBUG.print(pos_yaw);
  SERIAL_DEBUG.print(" | PITCH= ");
  SERIAL_DEBUG.println(pos_pitch);
}