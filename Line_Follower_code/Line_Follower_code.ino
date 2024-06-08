#include <QTRSensors.h>
#include <NewPing.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define left_motor_positive 5
#define left_motor_negative 4
#define right_motor_positive 2
#define right_motor_negative 3
#define en1 11
#define en2 10
const int trigPin = 8; // Trigger pin of the ultrasonic sensor
const int echoPin = 9; // Echo pin of the ultrasonic sensor

// Define variables
long duration;
int distance;

int check = 0;
int attempt = 0;
int rightState = 0;

int w = 920 ;
int b = 950;
int c = 500;

#define led 13


QTRSensors qtra;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];





int initial_motor_speed = 40;//100
int rotating_speed = 40;//60
int forward_speed =  40;//100
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;



int error;
float kp = 0.8; //proportional constant 0.05
float ki = 0;
float kd = 5; //5
float P, I, D, previousError = 0;
int pid_value;

char mode;
int Status = 0;
int buttom_reading;

int ObstaclePin = 10;
int ObstacleRead;
void led_signal(int times);

void calculatePID();
void PIDmotor_control();
uint16_t position;

void readIRvalue();     //to read sensor value and calculate error as well mode
// void Set_motion();

void dryrun();
// void actualrun();

void recIntersection(char);
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
// void setmotionactual();
// void mazeTurn (char dir);

void forward(int spd1, int spd2);
void left(int spd);
void right(int spd);
void stop_motor();
void goAndTurnLeft();
void maze_end();        
void move_inch();
void backward(int spd1, int spd2);


void setup()
{
//    Serial.begin(9600);
  // put your setup code here, to run once:
  for (int i = 0; i <= 5; i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
    digitalWrite(left_motor_positive, LOW);
    digitalWrite(left_motor_negative, LOW);
    digitalWrite(right_motor_positive, LOW);
    digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
  delay(500);
  pinMode(13, OUTPUT);

  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {
    A5, A4, A3, A2, A1, A0
  }, SensorCount);
  // qtra.setEmitterPin(9);


  digitalWrite(13, HIGH);


  // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 300; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on


}


void loop()
{
  dryrun();


}


void led_signal(int times)
{
  for (int i = 0; i <= times; i = i + 1)
  {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }
}

//dryrun begins------------------------------------------------------------------------------------------------------------------
void dryrun()
{
  readIRvalue();
  set_motion();

}
void Ultrasonic(){
  // Clear the trigger pin
  
  // Delay before next measurement
  delay(500);}
//ReadIRvalue begins----------------------------------------------------------------------------------------------------------------------------------
void readIRvalue()
{ 
  uint16_t position = qtra.readLineBlack(sensorValues);

  error = position - 2500;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the duration of the pulse from the echo pin
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in centimeters
  distance = duration * 0.034 / 2;
  if(distance<=10){
    mode = 'S';
  }
  
  // Print distance to the serial monitor
//  Serial.print("Distance: ");
//  Serial.print(distance);
//  Serial.println(" cm");
//  
//
  //   mode = 'F';



  //--------------------------------------------------------------------------------------
  if (sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w)
  {
    mode = 'S';  //NO LINE
    Serial.println("W");
    error = 0;
  }
  //
  //  //--------------------------------------------------------------------------------------
  else if ( sensorValues[0] > b && sensorValues[1] > b && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b)
  {
    mode = 'N';//Stop Condition
    error = 0;
  }
  //  //--------------------------------------------------------------------------------------
  //
  else if (position < 1500)
  {
    mode = 'R';//90 degree turn
    Serial.println("W");
    error = 0;
  }
  else if (position > 3500)
  {
    mode = 'L';//90 degree turn
    Serial.println("L");
    error = 0;

  }
     else if ( sensorValues[0] > b && sensorValues[1] > b && sensorValues[4] < w && sensorValues[5] < w)
     {
       mode = 'L';//90 degree turn
       Serial.println("L");
       error = 0;
     }

  //
  //
  //
  //
    else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] > w && sensorValues[5] > b)
    {
      mode = 'R'; //90 degree turn
      Serial.println("R");
      error = 0;
    }
     else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[4] > b && sensorValues[5] > b)
     {
       mode = 'R'; //90 degree turn
       error = 0;
       Serial.println("R");
     }
  else {
    mode = 'F';
  }
//  mode = 'F';

}



//set motion begins------------------------------------------------------------------------------------------------------------------------------
void set_motion()
{
  switch (mode)
  {
    case 'N':
      //stop_motor();
      move_inch();
      goAndTurnRight();
      recIntersection('B');
      break;
    case 'S':
      move_inch();
      readIRvalue();
      move_inch();

      if (mode == 'S')
      {
        goAndTurnLeft();
        recIntersection('L');
      }
      else
      {
        goAndTurnLeft();
        recIntersection('L');
      }
      break;
    case 'R':
      move_inch();
      move_inch();
      move_inch();
      readIRvalue();

      move_inch();
      move_inch();

      if (mode == 'F')
      {
        recIntersection('S');
      }
      else
      {
        goAndTurnRight();
      }
      break;
    case 'L':
      move_inch();
      move_inch();
      move_inch();
      move_inch();
      move_inch();


      goAndTurnLeft();
      break;
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
  }
}

//-----------------------------------------------------------------------------------------------------------
void move_inch()
{
  forward(forward_speed, forward_speed);
  delay(75);
  stop_motor();
}
//----------------------------------------------------------------------------------------------------------

void stop_motor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}
//-----------------------------------------------------------------------------------------------------------
void goAndTurnLeft()
{
  previousError = 0;
  left(rotating_speed);
  delay(300);
  do
  {
    left(rotating_speed);
    readIRvalue();
  } while (mode != 'F' );
  left(rotating_speed);
  delay(50);
}

//------------------------------------------------------------------------------------------------
void goAndTurnRight()
{ previousError = 0;
  right(rotating_speed);
  delay(300);
  do
  {
    right(rotating_speed);
    readIRvalue();
  } while (mode != 'F' );
  right(rotating_speed);
  delay(50);

}
//-----------------------------------------------------------------------------------------------
void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
}
//------------------------------------------------------------------------------------------------
void calculatePID()
{

  P = error;
  I = I + error;
  D = error - previousError;
  pid_value = (kp * P) + (ki * I) + (kd * D);
  previousError = error;

}
//-----------------------------------------------------------------------------------------------
void PIDmotor_control()
{
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);

}

//-------------------------------------------------------------------------------------------------------
void forward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, LOW);
}

void backward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, HIGH);
}

void left(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, LOW);
  digitalWrite(left_motor_negative, HIGH);
  digitalWrite(right_motor_positive, HIGH);
  digitalWrite(right_motor_negative, LOW);
  digitalWrite(led, HIGH);
}

void right(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  digitalWrite(left_motor_positive, HIGH);
  digitalWrite(left_motor_negative, LOW);
  digitalWrite(right_motor_positive, LOW);
  digitalWrite(right_motor_negative, HIGH);
  digitalWrite(led, HIGH);
}
//--------------------------------------------------------------------------------------


void recIntersection(char Direction)
{
  Serial.println(Direction);
}
