#include <Servo.h>
#include<TinyMPU6050.h>

// calibration - I accidentally did this in inches
#define A_ -0.02
#define B_ 9.33

// model parameters
#define M 0.131552          // mass of ball in kg
#define R 0.015875          // radius of ball in meters
#define G 9.8               // meters / second
#define I 2/5*M*R*R         // moment of inertia of ball in kg*m^2

// geometric parameters
#define L_A1 0.162            // meters, distance from motor axis to center hinge axis
#define L_B1 0.0279           // length of first motor linkage
#define L_A2 0.161            // distance from center hinge axis to small hinge axis
#define L_B2 0.09           // length of second motor linkage
#define D_H 0.09            // height of beam obove motor when it is parallel to the ground
#define PHI_0 asin(D_H/L_A1)  // angle between beam and line connecting motor and center hinge axes when level
                              // TODO: find this with calibration instead? Or use an accelerometer to sense it continuously?

// code TODO: update angle limits based on new motor mounting disk and second linkage length
#define LOOP_RATE_HZ 50
#define MIN_MOTOR_ANGLE 45
#define MAX_MOTOR_ANGLE 127
#define MAX_BEAM_ANGLE 15*M_PI/180

//MPU
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

int sensor_pin = 14;
int servo_pin = 15;
double set_point = 0;
double cur_pos = 0.0;
double beam_angle = 0.0;
double beam_angular_velocity = 0.0;
double prev_beam_angular_velocity = 0.0;
double alpha =0.2;
double beam_velocity_offset;

PID_struct pid = {0.1, 0.1, 0, 0, 0, -50, 50};
Servo s1;

//////////////////////
// Helper Functions //
//////////////////////

//beam angle conversion
double beam_angle_conversion(double des_acceleration, double cur_pos, double beam_angular_velocity) {
  double temp = (M*(cur_pos*0.0254)*pow(beam_angular_velocity,2) - (I/R/R+M)*des_acceleration)/(M*G);
  double a = asin(max(min(temp, 1.0), -1.0));
  return a;
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
  return A_*val + B_;
}

// very bad input linearization - do not use unless we can sense beam angular velocity directly
double input_linearization(double des_acceleration, double cur_pos, double beam_angular_velocity) {
  double temp = (M*0.0254*cur_pos*pow(beam_angular_velocity, 2) - (M+pow(R,2)*I)*des_acceleration) / (M*G);
  return asin(max(min(temp, 1.0), -1.0));
}

// using law of cosines to convert from beam pitch to motor angle
double convert_to_motor_angle(double beam_angle) {
  double C_angle = PHI_0 - beam_angle; // angle between beam and line connecting motor and center hinge axes
  double c_dist = sqrt(pow(L_A1, 2) + pow(L_A2, 2) - 2*L_A1*L_A2*cos(C_angle)); // dist from motor axis to small hinge
  double phi_1 = acos((-pow(L_A2, 2) + pow(L_A1, 2) + pow(c_dist, 2)) / (2*L_A1*c_dist)); // upper triangle angle
  double phi_2 = acos((-pow(L_B2, 2) + pow(L_B1, 2) + pow(c_dist, 2)) / (2*L_B1*c_dist)); // lower triangle angle
  return (phi_1 - (M_PI - PHI_0) + phi_2); // measured from vertical
}

//////////
// body // 
//////////

void setup() {
  Serial.begin(115200);
  while (!Serial);
  s1.attach(servo_pin);
  s1.write(90);
  
  //initialize mpu
  mpu.Initialize();
  Serial.println("Starting IMU Calibration");
  mpu.Calibrate();
  Serial.println("Finished Calibration");
  mpu.Execute();

  double beam_velocity_offset= mpu.GetGyroZ()*0.0174533;
}

void loop() {
  // Read sensor and convert
  cur_pos = convert_sensor_value(analogRead(sensor_pin)) + 1;
  Serial.println(cur_pos);

  // PID on ball position
  double des_acceleration = calc_PID(pid, cur_pos, set_point, 1.0 / LOOP_RATE_HZ);
  
  // Read from imu
  mpu.Execute();
  double measurement = mpu.GetGyroZ()*0.0174533;// - beam_velocity_offset; //read data and convert from degrees/s to radians/s
  //beam_angular_velocity = (alpha*measurement) + (1 - alpha*prev_beam_angular_velocity);
  //Serial.println(beam_angular_velocity);
  //Serial.println(measurement);
  prev_beam_angular_velocity = beam_angular_velocity;

  //Serial.println(cur_pos);
  // convert to beam angle using input linearization - not able to sense or calculate beam_angular_velocity well, so not using it at all
  //beam_angular_velocity = 0.0;
  //double beam_angle = input_linearization(des_acceleration, cur_pos, beam_angular_velocity);
  //beam_angle = min(max(beam_angle, -MAX_BEAM_ANGLE), MAX_BEAM_ANGLE);
  double beam_angle = beam_angle_conversion(des_acceleration, cur_pos,beam_angular_velocity);

  // convert to motor angle using law of cosines relationship
  double motor_angle = convert_to_motor_angle(beam_angle);
  motor_angle *= 180 / M_PI;

  // TODO: find this relationship using calibration or an acceleromter?
  motor_angle = 90 + motor_angle;

  // clamp and write to servo
  s1.write(min(max(motor_angle, MIN_MOTOR_ANGLE), MAX_MOTOR_ANGLE));
  delay(1000 / LOOP_RATE_HZ);
}
