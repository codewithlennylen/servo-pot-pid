#include <Arduino.h>
#include <Servo.h>

// Pin where the servo is connected
int servoPin = 9;

// Pin where pot is connected
int potPin = A0;

// Create a Servo object
Servo myServo;

// current servo angle
int current_angle = 0;

// PID Variables
double current_time;
double previousError;
double duration;
double derivative;
double integral;
int desired_set_point;

// Function definitions
bool initiate_PID_loop();
int control_signal(int);
int pid(int);
void reset_pid_loop();
int calculate_error(int);
int clockwise(int);
int counterclockwise(int);
int get_current_angle();
int set_current_angle(int);
int get_pot_value();

void setup()
{

  pinMode(servoPin, OUTPUT);
  pinMode(potPin, INPUT);

  myServo.attach(servoPin);

  current_time = millis();

  Serial.begin(9600);
}

void loop()
{

  bool start = initiate_PID_loop();

  if (start == true)
  {
    control_signal(desired_set_point);
  }
}

bool initiate_PID_loop()
{

  int potValue = get_pot_value();
  Serial.print("potValue");
  Serial.println(potValue);

  if (potValue == 0)
  {

    // calibrate params for first run
    reset_pid_loop();

    return true;
  }
}

int control_signal(int desired_set_point)
{
  int error = calculate_error(desired_set_point);

  int output = pid(error);
  int currentAngle = get_current_angle();

  if (output < 0)
  {
    clockwise(output * -1);
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.println(" Clockwise Motion");
  }
  else
  {
    counterclockwise(output);
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.println(" CouterClockwise Motion");
  }
}

int pid(int error)
{
  int Kp = 1;
  int Ki = 0;
  int Kd = 0;

  double dT = millis() - current_time;

  derivative = (error - previousError) / dT;

  integral += (error + previousError) * (dT / 2);

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  current_time = millis();
  previousError = error;

  // Debugging Purposes
  Serial.print("PID Gains");
  Serial.print(" Kp: ");
  Serial.print(Kp);
  Serial.print(" Ki: ");
  Serial.print(Ki);
  Serial.print(" Kd: ");
  Serial.print(Kd);

  Serial.print(" PID Variables");
  Serial.print(" error: ");
  Serial.print(error);
  Serial.print(" PID Variables");
  Serial.print(" dT: ");
  Serial.print(dT);
  Serial.print(" derivative: ");
  Serial.print(derivative);
  Serial.print(" integral: ");
  Serial.print(integral);
  Serial.print(" output: ");
  Serial.println(output);

  return output;
}

void reset_pid_loop()
{
  derivative = 0.0;

  // calibrate servo
  current_angle = 0;
}

int calculate_error(int set_point)
{
  int currentAngle = get_current_angle();

  int error = set_point - currentAngle;

  return error;
}

int clockwise(int units)
{

  int currentAngle = get_current_angle();
  int newAngle = currentAngle + units;

  set_current_angle(newAngle);

  myServo.write(newAngle);
}

int counterclockwise(int units)
{

  int currentAngle = get_current_angle();
  int newAngle = currentAngle + units;

  set_current_angle(newAngle);

  myServo.write(newAngle);
}

int get_current_angle()
{

  return current_angle;
}

int set_current_angle(int angle)
{

  current_angle = angle;

  return current_angle;
}

int get_pot_value()
{
  int potValue = analogRead(potPin);

  return potValue;
}