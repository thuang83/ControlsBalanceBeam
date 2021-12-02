#include <Servo.h>
#include "circbuff.h"
#include <TinyMPU6050.h>

#define tracking_control true
#define integral_control false

// model parameters
#define M 0.131552          // mass of ball in kg
#define R 0.015875          // radius of ball in meters
#define G 9.8               // meters / second
#define I 2/5*M*R*R         // moment of inertia of ball in kg*m^2
      
// geometric parameters
#define L_A1 0.12 //0.162            // meters, distance from motor axis to center hinge axis
#define L_B1 0.08   //0.0279           // length of first motor linkage
#define L_A2 0.17 //0.161            // distance from center hinge axis to small hinge axis
#define L_B2 0.095           // length of second motor linkage
#define D_H 0.09            // height of beam obove motor when it is parallel to the ground
#define PHI_0 asin(D_H/L_A1)  // angle between beam and line connecting motor and center hinge axes when level

#define LOOP_RATE_HZ 50
#define MIN_MOTOR_ANGLE 45
#define MAX_MOTOR_ANGLE 124 //127
#define MAX_BEAM_ANGLE 15*M_PI/180

// MPU
MPU6050 mpu(Wire);
/////////////
// structs //
/////////////
struct PID_struct {
  double Kp;
  double Kd;
  double Ki;
  double err_sum;
  double prev_error;
  double min_output;
  double max_output;
};

const int ledPin = 13;
int sensor_pin = 14;
int servo_pin = 15;
double set_point = -0.1;
double reference = set_point;
double cur_pos = 0.0;
double prev_pos = 0.0;
double beam_angle = 0.0;
double beam_angular_velocity = 0.0;
int i = 0;
int t0 = 0;

CircBuff<LOOP_RATE_HZ * 5> circbuff;
double beam_angle_offset = 0;
bool beam_angle_offset_known = false;

double prev_beam_angular_velocity = 0.0;
double alpha = 0.2;
double beam_velocity_offset;
PID_struct pid = {5, 5, 0, 0, 0, -50, 50};
double Ki = 0.75;
Servo s1;

//////////////////////
// Helper Functions //
//////////////////////

// beam angle conversion
double beam_angle_conversion(double des_acceleration, double cur_pos, double beam_angular_velocity) {
  double a = (M*cur_pos*pow(beam_angular_velocity, 2) - (M+I/pow(R,2))*des_acceleration) / (M*G);
  
  //double a = asin((I/R/R+M)*des_acceleration + M*cur_pos*beam_angular_acceleration*beam_angular_acceleration);
  //Serial.print(',');
  //Serial.println(millis());
  //Serial.print(",");
  //Serial.print(beam_angular_velocity);
  //Serial.print(",");
  //Serial.println(des_acceleration);
  double temp = asin(a);
  //Serial.println(temp);
  return temp;
}
// helper for calculating output using PID
double calc_PID(PID_struct& pid, double y, double r, double dt) {
  double cur_error = r - y;
  double out = pid.Kp * cur_error;                      // proportional
  out += pid.Kd * (cur_error - pid.prev_error) / dt;    // derivative - this isn't filtered so it could oscillate if Kd is too large
  out += pid.Ki * pid.err_sum;                          // integral

  // anti-windup - don't accumulate error if we are saturated
  if (out > pid.min_output && out < pid.max_output) {
    pid.err_sum += (cur_error + pid.prev_error) * dt / 2;
  }

  // updating prev error
  pid.prev_error = cur_error;
  return min(max(out, pid.min_output), pid.max_output);
}

// uses previously found calibration curve to convert from voltage to distance from hinge
double convert_sensor_value(int val) {
  return -(0.5*val/(1023-15) -.25);
}

// very bad input linearization - do not use unless we can sense beam angular velocity directly
double input_linearization(double des_acceleration, double cur_pos, double beam_angular_velocity) {
  double temp = (M*cur_pos*pow(beam_angular_velocity, 2) - (M+pow(R,2)*I)*des_acceleration) / (M*G);
  return asin(max(min(temp, 1.0), -1.0));
}

// using law of cosines to convert from beam pitch to motor angle
double convert_to_motor_angle(double beam_angle) {
  double C_angle = PHI_0 - beam_angle; // angle between beam and line connecting motor and center hinge axes
  double c_dist = sqrt(pow(L_A1, 2) + pow(L_A2, 2) - 2*L_A1*L_A2*cos(C_angle)); // dist from motor axis to small hinge
  double phi_1 = acos((-pow(L_A2, 2) + pow(L_A1, 2) + pow(c_dist, 2)) / (2*L_A1*c_dist)); // upper triangle angle
  double phi_2 = acos((-pow(L_B2, 2) + pow(L_B1, 2) + pow(c_dist, 2)) / (2*L_B1*c_dist)); // lower triangle angle
  return PHI_0 + phi_1 + phi_2 - M_PI/2.0; // measured from vertical
}

//////////
// body // 
//////////

void setup() {
  Serial.begin(115200);
  while (!Serial);
  s1.attach(servo_pin);
  s1.write(90);

  pinMode(ledPin, OUTPUT);

  //initialize mpu
  mpu.Initialize();
  //Serial.println("Starting IMU Calibration");
  Serial.println("ActualPosition ReferenceInput DesiredTrajectory");
  //mpu.Calibrate();
  //Serial.println("Finished Calibration");
  mpu.Execute();
  
  
  double beam_velocity_offset= -mpu.GetGyroZ()*0.0174533;
}
void loop() {
  // Read sensor and convert - discard values that are too different from last value
  cur_pos = convert_sensor_value(analogRead(sensor_pin));
  
  // Read from imu
  mpu.Execute();
  double measurement = -mpu.GetGyroZ()*0.0174533; //- beam_velocity_offset; //read data and convert from degrees/s to radians/s
  beam_angular_velocity = (alpha*measurement) + (1 - alpha*prev_beam_angular_velocity);
  prev_beam_angular_velocity = beam_angular_velocity;

  // 5 second moving average of velocity
  circbuff.addSample((cur_pos - prev_pos) * LOOP_RATE_HZ);
  if (abs(cur_pos - prev_pos) > 0.19){
      cur_pos = prev_pos;
  }else {
    prev_pos = cur_pos;
  }
  double vel_avg = circbuff.getAverage();

  // when ball has reached an equilibrium point and stopped moving for 5 seconds, update beam_angle_offset
  if (!beam_angle_offset_known && !isnan(vel_avg) && vel_avg < 0.02) {
    beam_angle_offset = beam_angle;
    Serial.println(beam_angle_offset);
    t0 = millis();
    beam_angle_offset_known = true;
    digitalWrite(ledPin, HIGH);
  }
  
  //Trajectory Tracking

  if (tracking_control && beam_angle_offset_known && abs(cur_pos) < 9.1) {
    //set_point = 5.0*(sin(float(i++) / (1*LOOP_RATE_HZ)));
    //set_point = (i++ % (6*LOOP_RATE_HZ) < 3*LOOP_RATE_HZ) ? -3 : 3;
    //set_point = 5  * float(i++ % (7*LOOP_RATE_HZ)) / (7*LOOP_RATE_HZ) - 2.5; 
    
    //Trajectory following sine function with amplitude of 5
    float t = float(i++)/LOOP_RATE_HZ;
    Serial.print(cur_pos*100);
    //SINE WAVE
    //set_point = 0.1*sin(1*t);
    //set_point = 0.0045714285714285707396389076686017*cos(2.0*t) + 0.0011428571428571426849097269171504*exp(-1.0*t) + 0.097714285714285714630180546165699*sin(2.0*t);
    //set_point = 0.0045714285714285707396389076686017*cos(2.0*t) + 0.0011428571428571426849097269171504*exp(-1.0*t) + 0.097714285714285714630180546165699*sin(2.0*t);
    //set_point = 0.01075630252100840174032684157318*cos(4.0*t) + 0.00067226890756302510877042759832377*exp(-1.0*t) + 0.097310924369747899564918289606705*sin(4.0*t);

    //SQUARE WAVE
    //set_point =0.00097819390666487107280782538371357*exp(-1.0*t) + 0.0095023279376032378152487858512162*cos(t) + 0.040549058984334286053335406372881*sin(t) - 0.31849922402079887225180062660589*pow(cos(t),3) + 2.8647503879896001081452371094771*pow(cos(t),5) - 10.912297378863578973646155331646*pow(cos(t),7) + 20.002968090074891397456023953587*pow(cos(t),9) - 17.454410800385725431267157528298*pow(cos(t),11) + 5.817008403361343661168755922776*pow(cos(t),13) - 0.14997430653903123806570790388795*pow(cos(t),2)*sin(t) + 2.6078944640905268533877689154326*pow(cos(t),4)*sin(t) - 14.149296215259627604152653052227*pow(cos(t),6)*sin(t) + 34.255328639289878474660659162846*pow(cos(t),8)*sin(t) - 37.56990741563162023305980435808*pow(cos(t),10)*sin(t) + 15.306383968972204333756249544402*pow(cos(t),12)*sin(t);
    set_point = 0.0019563878133297421456156507674271*exp(-1.0*t) + 0.019004655875206475630497571702432*cos(t) + 0.081098117968668572106670812745762*sin(t) - 0.63699844804159774450360125321178*pow(cos(t),3) + 5.7295007759792002162904742189541*pow(cos(t),5) - 21.824594757727157947292310663291*pow(cos(t),7) + 40.005936180149782794912047907174*pow(cos(t),9) - 34.908821600771450862534315056597*pow(cos(t),11) + 11.634016806722687322337511845552*pow(cos(t),13) - 0.2999486130780624761314158077759*pow(cos(t),2)*sin(t) + 5.2157889281810537067755378308651*pow(cos(t),4)*sin(t) - 28.298592430519255208305306104453*pow(cos(t),6)*sin(t) + 68.510657278579756949321318325692*pow(cos(t),8)*sin(t) - 75.13981483126324046611960871616*pow(cos(t),10)*sin(t) + 30.612767937944408667512499088804*pow(cos(t),12)*sin(t);

    //reference += Ki * (set_point - cur_pos) / LOOP_RATE_HZ;
    reference = set_point;
    Serial.print(',');
    Serial.print(reference*100);
    //double sine_desired = 10*sin(4*t);
    double square_desired = 5*(sin(t) + sin(3*t)/3 + sin(5*t)/5 + sin(7*t)/7 + sin(9*t)/9 + sin(11*t)/11 + sin(13*t)/13);
    Serial.print(',');
    Serial.println(square_desired);
    //Serial.println(10*sin(2*t));
    
    //Serial.println(5*(sin(t) + sin(3*t)/3 + sin(5*t)/5 + sin(7*t)/7 + sin(9*t)/9 + sin(11*t)/11 + sin(13*t)/13));
  }

  if (millis() - t0 > 5000) {
    set_point = 0.1;
  }
  
  // PID on ball position with integral control
  if (beam_angle_offset_known && integral_control){
    reference += Ki * (set_point - cur_pos) / LOOP_RATE_HZ;
  }
  
  //Serial.print(',');
  //Serial.print(reference);
  //Serial.println(beam_angular_velocity*10);


  double des_acceleration = calc_PID(pid, cur_pos, reference, 1.0 / LOOP_RATE_HZ);

  beam_angle = beam_angle_conversion(des_acceleration, cur_pos, beam_angular_velocity) + beam_angle_offset;
  beam_angle = min(max(beam_angle, -MAX_BEAM_ANGLE), MAX_BEAM_ANGLE);

  // convert to motor angle using law of cosines relationship
  double motor_angle = convert_to_motor_angle(beam_angle);
  motor_angle *= 180 / M_PI;

  // clamp and write to servo
  s1.write(min(max(motor_angle, MIN_MOTOR_ANGLE), MAX_MOTOR_ANGLE));
  delay(1000 / LOOP_RATE_HZ);
}
