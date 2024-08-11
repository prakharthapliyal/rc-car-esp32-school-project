#include <Ps3Controller.h>
#include <ESP32Servo.h>
#include <vector>

// FrontRight servomotor
int enableFrontRightMotor = 22;
int FrontRightMotorPin1 = 16;
int FrontRightMotorPin2 = 17;
// FrontLeft servomotor
int enableFrontLeftMotor = 23;
int FrontLeftMotorPin1 = 18;
int FrontLeftMotorPin2 = 19;

// RearRight motor
int enableRearRightMotor = 24;
int RearRightMotorPin3 = 20;
int RearRightMotorPin4 = 21;
// RearLeft motor
int enableRearLeftMotor = 25;
int RearLeftMotorPin3 = 22;
int RearLeftMotorPin4 = 23;

const int pingPin = 7;
const int rx = 180;  // Assuming 180 as the maximum value for rx

#define MAX_MOTOR_SPEED 255
#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};

std::vector<ServoPins> servoPins = 
{
  { Servo(), 27 , "left", rx},
  { Servo(), 26 , "right", rx},
};

const int PWMFreq = 1000;  /* 1KHz  */
const int PWMResolution = 8;
const int PWMSpeedChannel = 4;

void writeServoValues(int servoIndex, int servoMoveStepSize, bool servoStepSizeIsActualServoPosition = false)
{
  int servoPosition;
  if (servoStepSizeIsActualServoPosition)
  {
    servoPosition = servoMoveStepSize; 
  }
  else
  {
    servoPosition = servoPins[servoIndex].servo.read();
    servoPosition += servoMoveStepSize;
  }
  if (servoPosition > rx || servoPosition < 0)
  {
    return;
  }

  servoPins[servoIndex].servo.write(servoPosition);   
}

void notify()
{
  int yAxisValue = (Ps3.data.analog.stick.ly); // Left stick - y axis - forward/backward movement
  int rx = (Ps3.data.analog.stick.rx); // Right stick - x axis - left/right car movement
 
  if (rx > 50)
  {
    writeServoValues(0, SERVO_BACKWARD_STEP_ANGLE);  
  }
  else if (rx < -50)
  {
    writeServoValues(0, SERVO_FORWARD_STEP_ANGLE);  
  }
  if (yAxisValue <= 50)
  {
    rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  }
  else if (yAxisValue >= 50)
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  if (cm <= 25)
  {
    rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
  }
  else
  {
    rotateMotor(0, 0, 0, 0);
  }
}

void onConnect()
{
  Serial.println("Connected!");
}

void onDisconnect()
{
  rotateMotor(0, 0, 0, 0);
}

void rotateMotor(int RearLeftMotorSpeed, int RearRightMotorSpeed, int FrontRightMotorSpeed, int FrontLeftMotorSpeed)
{
  if (FrontRightMotorSpeed < 0)
  {
    digitalWrite(FrontRightMotorPin1, LOW);
    digitalWrite(FrontRightMotorPin2, HIGH);
  }
  else
  {
    digitalWrite(FrontRightMotorPin1, HIGH);
    digitalWrite(FrontRightMotorPin2, LOW);
  }

  if (FrontLeftMotorSpeed < 0)
  {
    digitalWrite(FrontLeftMotorPin1, LOW);
    digitalWrite(FrontLeftMotorPin2, HIGH);
  }
  else
  {
    digitalWrite(FrontLeftMotorPin1, HIGH);
    digitalWrite(FrontLeftMotorPin2, LOW);
  }

  if (RearLeftMotorSpeed < 0)
  {
    digitalWrite(RearLeftMotorPin3, LOW);
    digitalWrite(RearLeftMotorPin4, HIGH);
  }
  else
  {
    digitalWrite(RearLeftMotorPin3, HIGH);
    digitalWrite(RearLeftMotorPin4, LOW);
  }

  if (RearRightMotorSpeed < 0)
  {
    digitalWrite(RearRightMotorPin3, LOW);
    digitalWrite(RearRightMotorPin4, HIGH);
  }
  else
  {
    digitalWrite(RearRightMotorPin3, HIGH);
    digitalWrite(RearRightMotorPin4, LOW);
  }
}

void setUpPinModes()
{
  pinMode(enableFrontLeftMotor, OUTPUT);
  pinMode(FrontLeftMotorPin1, OUTPUT);
  pinMode(FrontLeftMotorPin2, OUTPUT);
  pinMode(enableFrontRightMotor, OUTPUT);
  pinMode(FrontRightMotorPin1, OUTPUT);
  pinMode(FrontRightMotorPin2, OUTPUT);
  pinMode(enableRearLeftMotor, OUTPUT);
  pinMode(RearLeftMotorPin3, OUTPUT);
  pinMode(RearLeftMotorPin4, OUTPUT);
  pinMode(enableRearRightMotor, OUTPUT);
  pinMode(RearRightMotorPin3, OUTPUT);
  pinMode(RearRightMotorPin4, OUTPUT);

  // Set up PWM for speed
  ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableFrontLeftMotor, PWMSpeedChannel);
  ledcAttachPin(enableFrontRightMotor, PWMSpeedChannel);
  ledcAttachPin(enableRearLeftMotor, PWMSpeedChannel);
  ledcAttachPin(enableRearRightMotor, PWMSpeedChannel);
  ledcWrite(PWMSpeedChannel, MAX_MOTOR_SPEED);

  rotateMotor(0, 0, 0, 0);
}

void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin("01:02:03:04:05:06");
  Serial.println("Ready.");

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
}

void loop()
{
  long duration, inches, cm;
  duration = pulseIn(pingPin, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(100);
  {
    if (Ps3.isConnected()){

        // Turn rumble on full intensity for 1 second
        Ps3.setRumble(100.0, 1000);
        delay(2000);

        // Turn rumble on full intensity indefinitely
        Ps3.setRumble(100.0);
        delay(2000);

        // Turn off rumble
        Ps3.setRumble(0.0);
    }

    delay(2000);
}
}

long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}