#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>

// Include ROS libraries
#include <ros_uart.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <mech_ros_msgs/WheelsVelocities.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <Arduino.h>
#include <mech_ros_msgs/RawImu.h>

#define PI 3.1415926
#define PWM_MAX_VAL 4800
#define sign(x) ((x) > 0 ? 1: -1)
#define R1 30
#define R2 82

/* Pin mapping */
// Encoder pins
uint8_t mot_fr_A = 52;
uint8_t mot_fr_B = 51;
uint8_t mot_fl_A = 50;
uint8_t mot_fl_B = 49;
uint8_t mot_rr_A = 7;
uint8_t mot_rr_B = 6;
uint8_t mot_rl_A = 53;
uint8_t mot_rl_B = 54;

// Motor control pins
uint8_t drv_f_slp = 48;
uint8_t drv_r_slp = 38;
uint8_t mot_fr_dir = 46;
uint8_t mot_fr_pwm = 8;
uint8_t mot_fl_dir = 47;
uint8_t mot_fl_pwm = 9;
uint8_t mot_rr_dir = 1;
uint8_t mot_rr_pwm = 3;
uint8_t mot_rl_dir = 0;
uint8_t mot_rl_pwm = 4;

uint8_t lights = 12;
uint8_t servo_1 = 2;
uint8_t v_bat_pin = 15;

/* Variable definitions */
// Encoders related variables
volatile long int ticks_fr = 0;
volatile long int ticks_fl = 0;
volatile long int ticks_rr = 0;
volatile long int ticks_rl = 0;

double pangle_fr = 0;
double pangle_fl = 0;
double pangle_rr = 0;
double pangle_rl = 0;

double rpm_fr;
double rpm_fl;
double rpm_rr;
double rpm_rl;

double angle_fr = 0;
double angle_fl = 0;
double angle_rr = 0;
double angle_rl = 0;

/* Parametry regulatoru */
float kp = 1000; //100
float ki = 10; //1
int out_max = PWM_MAX_VAL;
int out_min = -PWM_MAX_VAL;
float rpm_min = 0.3;
uint8_t rpm_max = 5;
float err_fr_sum = 0;
float err_fl_sum = 0;
float err_rr_sum = 0;
float err_rl_sum = 0;

float rpm_fr_r = 0; // required rpm [rad/s]
float rpm_fl_r = 0;
float rpm_rr_r = 0;
float rpm_rl_r = 0;

float filter_w = 0.7;

float podelna_rychlost = 0;
float rotace = 0;
// Tereni kola
//float rozchod = 0.150; //[m]
//float r_kola = 0.048; //[m]
// Silnicni kola
float rozchod = 0.2386; //[m]
float r_kola = 0.045394; //[m]

float v_bat = 0;

uint8_t fork_down = 101;
uint8_t fork_up = 71;

ros::NodeHandle nh;

uint32_t IMU_last_time = 0;
uint8_t IMU_update_rate = 50; // [Hz]
uint8_t RPM_reg_update_rate = 100; // [Hz]
uint32_t RPM_reg_last_time = 0;
uint8_t RPM_update_update_rate = 50; // [Hz]
uint32_t RPM_update_last_time = 0;
uint32_t Batt_last_time = 0;
uint8_t Batt_update_rate = 1; // [Hz]
int current_time;
int vel_received_time;
bool flag_zero_velocity_set = false;

#define BAUD 115200
//#define BAUD 230400

bool is_first = true;

mech_ros_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

mech_ros_msgs::WheelsVelocities vel_msg;
ros::Publisher wheels_vel_pub("wheels_vel", &vel_msg);

std_msgs::Int8 volt_battery;
ros::Publisher volt_battery_pub("volt_battery", &volt_battery);

LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 4.395 // Declination (degrees) in Boulder, CO.

Servo servo1;

void vel_control( const geometry_msgs::Twist& cmd_msg) {

  //digitalWrite(13, HIGH-digitalRead(13));  // pro testovani blikani LED - potom smazat
  podelna_rychlost = cmd_msg.linear.x;
  rotace  = cmd_msg.angular.z;
  flag_zero_velocity_set = false;
  vel_received_time = millis();

  /* Calculate required rpm */
  rpm_fl_r = (2 * podelna_rychlost - rotace * rozchod) / (2 * r_kola);
  rpm_rr_r = (2 * podelna_rychlost + rotace * rozchod) / (2 * r_kola);

  if ((abs(rpm_rr_r) > abs(rpm_fl_r)) && (abs(rpm_rr_r) > rpm_max)) {
    rpm_fl_r = rpm_max / abs(rpm_rr_r) * rpm_fl_r;
    rpm_rr_r = rpm_max * sign(rpm_rr_r);
  }
  if ((abs(rpm_fl_r) > abs(rpm_rr_r)) && (abs(rpm_fl_r) > rpm_max)) {
    rpm_rr_r = rpm_max / abs(rpm_fl_r) * rpm_rr_r;
    rpm_fl_r = rpm_max * sign(rpm_fl_r);
  }

  rpm_fr_r = rpm_rr_r;
  rpm_rl_r = rpm_fl_r;
}

ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", vel_control);


void servo_control( const std_msgs::Bool& cmd_servo) {
  //
  if (cmd_servo.data == true) {
    servo1.write(fork_up);
  } else {
    servo1.write(fork_down);
  }
}

ros::Subscriber<std_msgs::Bool> sub_servo("cmd_servo", servo_control);


void led_control( const std_msgs::Bool& cmd_led) {
  //
  if (cmd_led.data == true) {
    digitalWrite(lights, HIGH);
  } else {
    digitalWrite(lights, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> sub_led("cmd_led", led_control);

void setup()
{
  pinMode(mot_fr_A, INPUT);
  pinMode(mot_fr_B, INPUT);
  pinMode(mot_fl_A, INPUT);
  pinMode(mot_fl_B, INPUT);
  pinMode(mot_rr_A, INPUT);
  pinMode(mot_rr_B, INPUT);
  pinMode(mot_rl_A, INPUT);
  pinMode(mot_rl_B, INPUT);

  pinMode(mot_fr_dir, OUTPUT);
  pinMode(mot_fr_pwm, OUTPUT);
  pinMode(mot_fl_dir, OUTPUT);
  pinMode(mot_fl_pwm, OUTPUT);
  pinMode(mot_rr_dir, OUTPUT);
  pinMode(mot_rr_pwm, OUTPUT);
  pinMode(mot_rl_dir, OUTPUT);
  pinMode(mot_rl_pwm, OUTPUT);

  pinMode(drv_f_slp, OUTPUT);
  pinMode(drv_r_slp, OUTPUT);
  digitalWrite(drv_f_slp, HIGH);
  digitalWrite(drv_r_slp, HIGH);

  pinMode(lights, OUTPUT);
  digitalWrite(lights, LOW);
  servo1.attach(servo_1);

  analogReadResolution(12);

  attachInterrupt(mot_fr_A, mot_fr_A_isr, CHANGE);
  attachInterrupt(mot_fr_B, mot_fr_B_isr, CHANGE);
  attachInterrupt(mot_fl_A, mot_fl_A_isr, CHANGE);
  attachInterrupt(mot_fl_B, mot_fl_B_isr, CHANGE);
  attachInterrupt(mot_rr_A, mot_rr_A_isr, CHANGE);
  attachInterrupt(mot_rr_B, mot_rr_B_isr, CHANGE);
  attachInterrupt(mot_rl_A, mot_rl_A_isr, CHANGE);
  attachInterrupt(mot_rl_B, mot_rl_B_isr, CHANGE);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  /*SerialUSB.begin(230400); // start serial for output
    while (!SerialUSB);
    SerialUSB.println("Ready");*/

  ki = ki * (1000 / RPM_reg_update_rate);

  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.subscribe(sub_servo);
  nh.subscribe(sub_led);
  nh.advertise(raw_imu_pub);
  nh.advertise(wheels_vel_pub);
  nh.advertise(volt_battery_pub);

  //SerialUSB.println("a");
  // Wait for ROSserial to connect
  //digitalWrite(mot_rr_dir, 0);
  while (!nh.connected())
  {
    nh.spinOnce();    
  }
  delay(5);
}

void loop()
{
  if (nh.connected())
  {

    current_time = millis();
    if (is_first)
    {

      nh.loginfo("Checking devices");

      if (!imu.begin())
      {
        nh.logerror("IMU NOT FOUND!");
      }

      is_first = false;
    }

    else if (current_time >= RPM_update_last_time + 1000 / RPM_update_update_rate)
    {
      RPM_update_last_time = current_time;

      vel_msg.front_left = rpm_fl;
      vel_msg.front_right = rpm_fr;
      vel_msg.rear_left = rpm_rl;
      vel_msg.rear_right = rpm_rr;
      wheels_vel_pub.publish(&vel_msg);
    }
    else if (current_time >= IMU_last_time + 1000 / IMU_update_rate)
    {
      IMU_last_time = current_time;

      imu.readAccel();
      raw_imu_msg.raw_linear_acceleration.x = imu.ax;
      raw_imu_msg.raw_linear_acceleration.y = imu.ay;
      raw_imu_msg.raw_linear_acceleration.z = imu.az;

      imu.readGyro();
      raw_imu_msg.raw_angular_velocity.x = imu.gx;
      raw_imu_msg.raw_angular_velocity.y = imu.gy;
      raw_imu_msg.raw_angular_velocity.z = imu.gz;

      imu.readMag();
      raw_imu_msg.raw_magnetic_field.x = imu.mx;
      raw_imu_msg.raw_magnetic_field.y = imu.my;
      raw_imu_msg.raw_magnetic_field.z = imu.mz;

      raw_imu_pub.publish(&raw_imu_msg);
    }

    else if (current_time >= Batt_last_time + 1000 / Batt_update_rate)
    {
      Batt_last_time = current_time;

      // Fill battery voltage state
      v_bat = (R1 + R2) * (float(analogRead(v_bat_pin)) / 4095 * 3.3) / R2;
      volt_battery.data = round(v_bat * 100) - 335;
      volt_battery_pub.publish(&volt_battery);

      // Check if zero velocity flag is set
      if(!flag_zero_velocity_set)
      {
        // If not set, check time from last received command
        if (current_time >= vel_received_time + 1000 / Batt_update_rate)
        {
            rpm_fl_r = 0.0;
            rpm_rr_r = 0.0;
            rpm_fr_r = 0.0;
            rpm_rl_r = 0.0;
            flag_zero_velocity_set = true;
        }

      }
    }
    else if (current_time >= RPM_reg_last_time + 1000 / RPM_reg_update_rate)
    {
      RPM_reg_last_time = current_time;


      /* Angle + filtration */
      angle_fr = ticks_fr / (5 * 4 * 312.5) * 2 * PI;
      angle_fr = filter_w * angle_fr + (1 - filter_w) * pangle_fr;

      angle_fl = ticks_fl / (5 * 4 * 312.5) * 2 * PI;
      angle_fl = filter_w * angle_fl + (1 - filter_w) * pangle_fl;

      angle_rr = ticks_rr / (5 * 4 * 312.5) * 2 * PI;
      angle_rr = filter_w * angle_rr + (1 - filter_w) * pangle_rr;

      angle_rl = ticks_rl / (5 * 4 * 312.5) * 2 * PI;
      angle_rl = filter_w * angle_rl + (1 - filter_w) * pangle_rl;

      /* Angular velocity */
      rpm_fr = (angle_fr - pangle_fr) / (1000 / RPM_reg_update_rate) * 1000.0;
      rpm_fl = (angle_fl - pangle_fl) / (1000 / RPM_reg_update_rate) * 1000.0;
      rpm_rr = (angle_rr - pangle_rr) / (1000 / RPM_reg_update_rate) * 1000.0;
      rpm_rl = (angle_rl - pangle_rl) / (1000 / RPM_reg_update_rate) * 1000.0;

      pangle_fr = angle_fr;
      pangle_fl = angle_fl;
      pangle_rr = angle_rr;
      pangle_rl = angle_rl;

      /* Regulator fr*/
      double up_fr, u_fr, rpm_fr_e;

      rpm_fr_e = rpm_fr_r - rpm_fr;

      up_fr = rpm_fr_e * kp;
      err_fr_sum += (ki * rpm_fr_e);

      if (err_fr_sum > out_max) err_fr_sum = out_max;
      else if (err_fr_sum < out_min) err_fr_sum = out_min;

      if (abs(rpm_fr_r) < rpm_min) err_fr_sum = 0;

      u_fr = up_fr + err_fr_sum;

      if (u_fr > out_max) u_fr = out_max;
      else if (u_fr < out_min) u_fr = out_min;

      /* Regulator fl*/
      double up_fl, u_fl, rpm_fl_e;

      rpm_fl_e = rpm_fl_r - rpm_fl;

      up_fl = rpm_fl_e * kp;
      err_fl_sum += (ki * rpm_fl_e);

      if (err_fl_sum > out_max) err_fl_sum = out_max;
      else if (err_fl_sum < out_min) err_fl_sum = out_min;

      if (abs(rpm_fl_r) < rpm_min) err_fl_sum = 0;

      u_fl = up_fl + err_fl_sum;

      if (u_fl > out_max) u_fl = out_max;
      else if (u_fl < out_min) u_fl = out_min;

      /* Regulator rr*/
      double up_rr, u_rr, rpm_rr_e;

      rpm_rr_e = rpm_rr_r - rpm_rr;

      up_rr = rpm_fr_e * kp;
      err_rr_sum += (ki * rpm_rr_e);

      if (err_rr_sum > out_max) err_rr_sum = out_max;
      else if (err_rr_sum < out_min) err_rr_sum = out_min;

      if (abs(rpm_rr_r) < rpm_min) err_rr_sum = 0;

      u_rr = up_rr + err_rr_sum;

      if (u_rr > out_max) u_rr = out_max;
      else if (u_rr < out_min) u_rr = out_min;

      /* Regulator rl*/
      double up_rl, u_rl, rpm_rl_e;

      rpm_rl_e = rpm_rl_r - rpm_rl;

      up_rl = rpm_rl_e * kp;
      err_rl_sum += (ki * rpm_rl_e);

      if (err_rl_sum > out_max) err_rl_sum = out_max;
      else if (err_rl_sum < out_min) err_rl_sum = out_min;

      if (abs(rpm_rl_r) < rpm_min) err_rl_sum = 0;

      u_rl = up_rl + err_rl_sum;

      if (u_rl > out_max) u_rl = out_max;
      else if (u_rl < out_min) u_rl = out_min;

      /* Motor Control */
      if (u_fr >= 0) fwd_mot_fr(int(u_fr));
      else rvs_mot_fr(int(-u_fr));

      if (u_fl >= 0) fwd_mot_fl(int(u_fl));
      else rvs_mot_fl(int(-u_fl));

      if (u_rr >= 0) fwd_mot_rr(int(u_rr));
      else rvs_mot_rr(int(-u_rr));

      if (u_rl >= 0) fwd_mot_rl(int(u_rl));
      else rvs_mot_rl(int(-u_rl));

      /*SerialUSB.print(rpm_rr_r);
        SerialUSB.print("\t");
        SerialUSB.println(rpm_rr);*/
      /*SerialUSB.print(rpm_fr);
        SerialUSB.print("\t");
        SerialUSB.print(rpm_fr_e);
        SerialUSB.print("\t");
        SerialUSB.print(up_fr);
        SerialUSB.print("\t");
        SerialUSB.println(err_fr_sum);*/
    }
  }
  nh.spinOnce();
}

/* FR */
void fwd_mot_fr(int pwm) {
  digitalWrite(mot_fr_dir, HIGH);
  analogWrite(mot_fr_pwm, PWM_MAX_VAL - pwm);
}

void rvs_mot_fr(int pwm) {
  digitalWrite(mot_fr_dir, LOW);
  analogWrite(mot_fr_pwm, pwm);
}

void stop_mot_fr() {
  digitalWrite(mot_fr_dir, LOW);
  analogWrite(mot_fr_pwm, 0);
}

/* FL */
void rvs_mot_fl(int pwm) {
  digitalWrite(mot_fl_dir, HIGH);
  analogWrite(mot_fl_pwm, PWM_MAX_VAL - pwm);
}

void fwd_mot_fl(int pwm) {
  digitalWrite(mot_fl_dir, LOW);
  analogWrite(mot_fl_pwm, pwm);
}

void stop_mot_fl() {
  digitalWrite(mot_fl_dir, LOW);
  analogWrite(mot_fl_pwm, 0);
}

/* RR */
void fwd_mot_rr(int pwm) {
  digitalWrite(mot_rr_dir, HIGH);
  analogWrite(mot_rr_pwm, PWM_MAX_VAL - pwm);
}

void rvs_mot_rr(int pwm) {
  digitalWrite(mot_rr_dir, LOW);
  analogWrite(mot_rr_pwm, pwm);
}

void stop_mot_rr() {
  digitalWrite(mot_rr_dir, LOW);
  analogWrite(mot_rr_pwm, 0);
}

/* RL */
void rvs_mot_rl(int pwm) {
  digitalWrite(mot_rl_dir, HIGH);
  analogWrite(mot_rl_pwm, PWM_MAX_VAL - pwm);
}

void fwd_mot_rl(int pwm) {
  digitalWrite(mot_rl_dir, LOW);
  analogWrite(mot_rl_pwm, pwm);
}

void stop_mot_rl() {
  digitalWrite(mot_rl_dir, LOW);
  analogWrite(mot_rl_pwm, 0);
}

/* ENCODER ISR */
void mot_fr_A_isr() {
  if ((digitalRead(mot_fr_A) == 1) && (digitalRead(mot_fr_B) == 0)) {
    ticks_fr++;
  } else if ((digitalRead(mot_fr_A) == 0) && (digitalRead(mot_fr_B) == 1)) {
    ticks_fr++;
  } else if ((digitalRead(mot_fr_A) == 1) && (digitalRead(mot_fr_B) == 1)) {
    ticks_fr--;
  } else if ((digitalRead(mot_fr_A) == 0) && (digitalRead(mot_fr_B) == 0)) {
    ticks_fr--;
  }
}

void mot_fr_B_isr() {
  if ((digitalRead(mot_fr_B) == 1) && (digitalRead(mot_fr_A) == 1)) {
    ticks_fr++;
  } else if ((digitalRead(mot_fr_B) == 0) && (digitalRead(mot_fr_A) == 0)) {
    ticks_fr++;
  } else if ((digitalRead(mot_fr_B) == 1) && (digitalRead(mot_fr_A) == 0)) {
    ticks_fr--;
  } else if ((digitalRead(mot_fr_B) == 0) && (digitalRead(mot_fr_A) == 1)) {
    ticks_fr--;
  }
}

void mot_fl_A_isr() {
  if ((digitalRead(mot_fl_A) == 1) && (digitalRead(mot_fl_B) == 0)) {
    ticks_fl--;
  } else if ((digitalRead(mot_fl_A) == 0) && (digitalRead(mot_fl_B) == 1)) {
    ticks_fl--;
  } else if ((digitalRead(mot_fl_A) == 1) && (digitalRead(mot_fl_B) == 1)) {
    ticks_fl++;
  } else if ((digitalRead(mot_fl_A) == 0) && (digitalRead(mot_fl_B) == 0)) {
    ticks_fl++;
  }
}

void mot_fl_B_isr() {
  if ((digitalRead(mot_fl_B) == 1) && (digitalRead(mot_fl_A) == 1)) {
    ticks_fl--;
  } else if ((digitalRead(mot_fl_B) == 0) && (digitalRead(mot_fl_A) == 0)) {
    ticks_fl--;
  } else if ((digitalRead(mot_fl_B) == 1) && (digitalRead(mot_fl_A) == 0)) {
    ticks_fl++;
  } else if ((digitalRead(mot_fl_B) == 0) && (digitalRead(mot_fl_A) == 1)) {
    ticks_fl++;
  }
}

void mot_rr_A_isr() {
  if ((digitalRead(mot_rr_A) == 1) && (digitalRead(mot_rr_B) == 0)) {
    ticks_rr++;
  } else if ((digitalRead(mot_rr_A) == 0) && (digitalRead(mot_rr_B) == 1)) {
    ticks_rr++;
  } else if ((digitalRead(mot_rr_A) == 1) && (digitalRead(mot_rr_B) == 1)) {
    ticks_rr--;
  } else if ((digitalRead(mot_rr_A) == 0) && (digitalRead(mot_rr_B) == 0)) {
    ticks_rr--;
  }
}

void mot_rr_B_isr() {
  if ((digitalRead(mot_rr_B) == 1) && (digitalRead(mot_rr_A) == 1)) {
    ticks_rr++;
  } else if ((digitalRead(mot_rr_B) == 0) && (digitalRead(mot_rr_A) == 0)) {
    ticks_rr++;
  } else if ((digitalRead(mot_rr_B) == 1) && (digitalRead(mot_rr_A) == 0)) {
    ticks_rr--;
  } else if ((digitalRead(mot_rr_B) == 0) && (digitalRead(mot_rr_A) == 1)) {
    ticks_rr--;
  }
}

void mot_rl_A_isr() {
  if ((digitalRead(mot_rl_A) == 1) && (digitalRead(mot_rl_B) == 0)) {
    ticks_rl--;
  } else if ((digitalRead(mot_rl_A) == 0) && (digitalRead(mot_rl_B) == 1)) {
    ticks_rl--;
  } else if ((digitalRead(mot_rl_A) == 1) && (digitalRead(mot_rl_B) == 1)) {
    ticks_rl++;
  } else if ((digitalRead(mot_rl_A) == 0) && (digitalRead(mot_rl_B) == 0)) {
    ticks_rl++;
  }
}

void mot_rl_B_isr() {
  if ((digitalRead(mot_rl_B) == 1) && (digitalRead(mot_rl_A) == 1)) {
    ticks_rl--;
  } else if ((digitalRead(mot_rl_B) == 0) && (digitalRead(mot_rl_A) == 0)) {
    ticks_rl--;
  } else if ((digitalRead(mot_rl_B) == 1) && (digitalRead(mot_rl_A) == 0)) {
    ticks_rl++;
  } else if ((digitalRead(mot_rl_B) == 0) && (digitalRead(mot_rl_A) == 1)) {
    ticks_rl++;
  }
}