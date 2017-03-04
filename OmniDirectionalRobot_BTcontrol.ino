/******************************************************************************/
/*                                      **                                    */
/*                    Omnidirectional Robot Bluetooth Control                 */
/*                                      **                                    */
/*                                                                            */
/* Connects to a Bluetooth module and parses data sent from a device. Works   */
/* with Arduino Uno and also tested to work with                              */
/* a Digispark (remove all Serial.print)                                      */
/* Expects string data to come as follows:                                    */
/*    sX=[xVal],Y=[yVal]e  - used for sending X/Y data.                       */
/* Example:                                                                   */
/*    sX=75,Y=55e.  Handles +/-                                               */
/* Command button data comes as follows:                                      */
/*    sC=[cVal]e                                                              */
/* Example:                                                                   */
/*    sC=1e                                                                   */
/*                                                                            */
/* Written By: Umur Ozhan SENGUL (umursengul@ieee.org)                        */
/* Based on: http://www.plastibots.com/index.php/2013/03/07/btbotcontrol/     */
/*                                                                            */
/******************************************************************************/

/*/////////////*/
/*  LIBRARIES  */
/*/////////////*/
//#include <SoftwareSerial.h> // Debugging
#include <Servo.h>

/*///////////////////*/
/*  PIN DEFINITIONS  */
/*///////////////////*/
// Define test pins for bluetooth
//#define RxPin 8 //goes to TX pin on BT module - Debugging
//#define TxPin 7 //goes to RX pin on BT module - Debugging
// Define remote command start and end
#define SOP 's'
#define EOP 'e'
// Define motor control pins
#define leftMtrDirPin1 A0
#define leftMtrDirPin2 A1
#define leftMtrSpdPin 6
#define rightMtrDirPin1 A2
#define rightMtrDirPin2 A3
#define rightMtrSpdPin 11

/*/////////////////////*/
/*  GLOBAL PARAMETERS  */
/*/////////////////////*/
//
char inData[20];
byte index;
int xVal = 0, xAdj = 0, yVal = 0, yAdj = 0, cVal = 1;
int speedFactor = 1;
int leftSpeed = 0, rightSpeed = 0;
int leftDir = HIGH, rightDir = HIGH;
int theta = 0, theta_norm = 0, theta_norm_old = 0, theta_norm_new = 0;
boolean started = false;
boolean ended = false;
boolean receivingCommands = false;  // Dont send serial data when joystick being removed.
int counter = 0;
int delayVal = 50;
Servo fLeft, fRight, rLeft, rRight;

/*////////////////////////*/
/*  DEBUGGING PARAMETERS  */
/*////////////////////////*/
//SoftwareSerial blueToothSerial(RxPin, TxPin); // RX, TX - Debugging

/*////////////////*/
/*  SETUP SYSTEM  */
/*////////////////*/
void setup()
{
  // Start serial communications
  Serial.begin(9600); // 57600 while debugging
  //blueToothSerial.begin(9600);  // Debugging
  // Define pin modes for communication pins
  //pinMode(RxPin, INPUT);  // Debugging
  //pinMode(TxPin, OUTPUT); // Debugging
  // Define pin modes for motor pins
  pinMode(leftMtrSpdPin, OUTPUT);
  pinMode(leftMtrDirPin1, OUTPUT);
  pinMode(leftMtrDirPin2, OUTPUT);
  pinMode(rightMtrSpdPin, OUTPUT);
  pinMode(rightMtrDirPin1, OUTPUT);
  pinMode(rightMtrDirPin2, OUTPUT);
  // Define servo pins
  fLeft.attach(2);
  fRight.attach(3);
  rLeft.attach(4);
  rRight.attach(5);
  //Serial.println("Omnidirectional Robot Bluetooth Control");  // Debugging
}

/*////////////////*/
/*  MAIN PROGRAM  */
/*////////////////*/
void loop()
{
  xVal=0;
  yVal=0;

  //while(blueToothSerial.available() > 0)  // Debugging
  while(Serial.available() > 0)
  {
    receivingCommands = true;
    //char inChar = blueToothSerial.read();
    char inChar = Serial.read();

    if(inChar == SOP)
    {
      index = 0;
      inData[index] = '\0';
      started = true;
      ended = false;
    }
    else if(inChar == EOP)
    {
      ended = true;
      break;
    }
    else
    {
      if(index < 19)
      {
        inData[index] = inChar;
        index++;
        inData[index] = '\0';
      }
    }
  }

  // We are here either because all pending serial
  // data has been read OR because an end of
  // packet marker arrived. Which is it?
  if(started && ended)
  {
    // The end of packet marker arrived. Process the packet
    char *name = strtok(inData, "=");
    while(name)
    {
      char *valToken = strtok(NULL, ",");
      if(valToken)
      {
        int val = atoi(valToken);
        if(strcmp(name, "X") == 0)
        {
          xVal = val;
        }
        else if(strcmp(name, "Y") == 0)
        {
          yVal = val;
        }
        else if(strcmp(name, "C") == 0)
        {
          cVal = val;
        }
      }
      name = strtok(NULL, "=");
    }

    // Note - debugging to the Serial Monitor will cause Serial.available to crap out and make it seem
    // like the device has stopped outputting BT data. The reason is that the Serial buffer overflowed due to
    // printing debuggin statements.  Turn debugging on only to verify you are getting data and test a few values

    //output values to Serial Monitor
    Serial.print("X=");
    Serial.print(xVal);
    Serial.print("  Y=");
    Serial.print(yVal);
    Serial.print("  C=");
    Serial.print(cVal);

    if ((xVal !=0) || (yVal != 0))
    {
      // Allow for a state where the robot is not moving with small joystick
      // movements.
      /*
      if (((xVal < 5) && (xVal > -5)) && ((yVal < 5) && (yVal > -5)))
      {
        xVal=0;
        yVal=0;
      }
      */
      positionServos(xVal, yVal);
      driveMotors(xVal, yVal);
      receivingCommands = false;
    }
    else
    {
      //turn motors off
      //positionServos(0,1);
      analogWrite(leftMtrSpdPin, 0);
      analogWrite(rightMtrSpdPin, 0);
      receivingCommands = false;

    }
    // Reset for the next packet
    started = false;
    ended = false;
    index = 0;
    inData[index] = '\0';
  }

  xVal=0, yVal=0;

}

void positionServos(int xVal, int yVal)
{
  theta = round(atan2(yVal,xVal)*180/3.14159265);

  if(theta <= 180 && theta >= 0)
  {
    theta_norm = theta;
  }
  else if(theta > 180 && theta <= 360)
  {
    theta_norm = 360 - theta;
  }
  else if(theta < 0 && theta >= -180)
  {
    theta_norm = 180 + theta;
  }
  //Serial.print(" theta_norm "); // Debugging
  //Serial.print(theta_norm);     // Debugging
  // Don't set servo positions if change is smaller then 20 degrees
  theta_norm_new = theta_norm;
  //Serial.print(" theta_norm_new "); // Debugging
  //Serial.print(theta_norm_new);     // Debugging
  if((abs(theta_norm_new - theta_norm_old)) < 20)
  {
    theta_norm = theta_norm_old;
  }
  else
  {
    theta_norm = theta_norm_new;
  }
  theta_norm_old = theta_norm;
  //Serial.print(" theta_norm_old "); // Debugging
  //Serial.print(theta_norm_old);     // Debugging
  //Serial.print(" theta_norm ");     // Debugging
  //Serial.println(theta_norm);       // Debugging

  // Position servos to the desired angles
  fLeft.write(theta_norm + 7);
  fRight.write(theta_norm - 4);
  rLeft.write(theta_norm + 6);
  rRight.write(theta_norm - 6);
  delay(50);
  /*
  // For debugging purposes
  Serial.print(" Servos ");
  Serial.println(theta_norm);
  delay(15);
  */
}

void driveMotors(int xVal, int yVal)
{
  float xPct=1.0;
  int baseVal = 0;

  switch (cVal)
  {
  case 1:
    baseVal = 0;
    break;
  case 2:
    baseVal = 20;
    break;
  case 3:
    baseVal = 50;
    break;
  }

  xAdj = map(abs(xVal), 0, 100, 0, 255);
  yAdj = map(abs(yVal), 0, 100, 0, 255);

  // If Y is positive, both motors are set to move forward, else reverse.
  // Left and Right from center will determine the speed of each motor.
  // At extremes will reverse each motor for fast turns
  // the value for X will determine the relative speed of each motor.

  // Determine the direction
  if (yVal > 0)
  {
    // Both motors moving fwd
    digitalWrite(leftMtrDirPin1, LOW);
    digitalWrite(leftMtrDirPin2, HIGH);
    digitalWrite(rightMtrDirPin1, LOW);
    digitalWrite(rightMtrDirPin2, HIGH);
  }
  else
  {
    // Both motors moving rev
    digitalWrite(leftMtrDirPin1, HIGH);
    digitalWrite(leftMtrDirPin2, LOW);
    digitalWrite(rightMtrDirPin1, HIGH);
    digitalWrite(rightMtrDirPin2, LOW);
  }
  // Determine the speed (based on abs(Yval) proportion of speed based on the xVal (-xx to + xx)
  // to each motor and drive them.

  if (xVal < 0)
  {
    //going left
    leftSpeed = yAdj - abs(xAdj) ;
    rightSpeed = yAdj;
  }
  else
  {
    //going right
    leftSpeed = yAdj;
    rightSpeed = yAdj - abs(xAdj) ;
  }

  //drive the motors
  analogWrite(leftMtrSpdPin, leftSpeed);
  analogWrite(rightMtrSpdPin, rightSpeed);

  //slight delay
  delay(delayVal);
  //Serial.println();
  leftSpeed=0, rightSpeed=0, xAdj=0, yAdj=0;
}

//use to map values to float.
float mapf (float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
