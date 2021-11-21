#include <ArduinoEigenDense.h>

#define NUM_SAMPLES 5

int sensorPin = 14;
int i = 0;
Eigen::MatrixXd sensor_values(NUM_SAMPLES, 2);
Eigen::VectorXd calibration_values(NUM_SAMPLES);
char buff[32] = {0};

void handle_input(int val) {
  sensor_values(i, 0) = val;
  sensor_values(i, 1) = 1;
  memset(buff, 0, 32);
  int buff_index = 0;

  while (Serial.available()) 
    buff[buff_index++] = Serial.read();

  calibration_values[i++] = atof(buff);
  Serial.print(sensor_values(i-1, 0));
  Serial.print(',');
  Serial.println(calibration_values[i-1]);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
}

void loop() {
  int val = analogRead(sensorPin);
  if (Serial.available()) {
    handle_input(val);
  }

  if (i == NUM_SAMPLES) {
    // least squares once have all values
    Eigen::Vector2d val = (sensor_values.transpose() * sensor_values).ldlt().solve(sensor_values.transpose() * calibration_values);
    Serial.print("calibration (a,b):");
    Serial.print(val[0]);
    Serial.print(',');
    Serial.println(val[1]);
    while (1);
  }

  delay(20);
}
