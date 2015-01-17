/*
Vlad Silin
January 16, 2013
Description: This is code for a line following robot. 
This code enables the robot to go back and forth along black lines and curves on a white background.
The robot is also capable of finding its way out of a line maze using the right hand rule.
*/

int lighting = 4;

int leftSensor, midSensor, rightSensor, periphLeft, periphRight;
int leftDifference = 0, rightDifference = 0, centre = 0;

int rightMotorSpeed = 3, leftMotorSpeed = 9, rightMotorForward = 7, leftMotorForward = 12, 
  rightMotorReverse = 8, leftMotorReverse = 2;
int startSpeed = 55, scan = 20, threshold = 27;
int left = startSpeed, right = startSpeed;

int whiteAvg, stopAvg = 0; 
int blackLeft, blackMid, blackRight = 0;

boolean sawLeft, sawRight = false;

/*
This method does a general calibration of the 3 main (central) LDRs. Sensor values are read in 10 times.
The average values are found and the left and right sensors' offset from the central LDR is determined.
*/
void calibrate() 
{
  Serial.begin (9600);
  for (int x = 0; x < 10; x++) 
  { 
    digitalWrite(lighting, HIGH); 
    delay(100);
    leftSensor = analogRead(1); 
    midSensor = analogRead(2);
    rightSensor = analogRead(3);
    leftDifference = leftDifference + leftSensor; 
    centre = centre + midSensor;
    rightDifference = rightDifference + rightSensor; 
    delay(100);
    digitalWrite(lighting, LOW);
    delay(100);
  }

  leftDifference = leftDifference / 10;
  rightDifference = rightDifference / 10;
  centre = centre / 10;

  leftDifference = centre - leftDifference;
  rightDifference = centre - rightDifference;
}

/*
This method takes the average of 15 readings of the LDR sensors on a white background. The if structure
is used to take the readings starting from the 85th one to the 100th one. This is required for accurate values.
*/
void calibrateWhite () 
{
  delay (3000);

  for (int x=0; x<100; x++) 
  {
    digitalWrite(lighting, HIGH);
    leftSensor = analogRead(1) + leftDifference;
    midSensor = analogRead(2);
    rightSensor = analogRead(3) + rightDifference;
    
    if (x > 84) 
    {
      whiteAvg = whiteAvg + (leftSensor + midSensor + rightSensor)/3;
    }
  }
  
  whiteAvg = whiteAvg/15;
  
  digitalWrite(lighting, LOW);
}

/*
This method takes the average of 15 readings of the LDR sensors on a black background. The if structure
is used to take the readings starting from the 85th one to the 100th one. This is required for accurate values.
*/
void calibrateBlack () 
{
  delay (3000);
  
  for (int x=0; x<100; x++) 
  {
    digitalWrite(lighting, HIGH);
    periphLeft = analogRead(0);
    leftSensor = analogRead(1) + leftDifference;
    midSensor = analogRead(2);
    rightSensor = analogRead(3) + rightDifference;
    periphRight = analogRead(4);

    if (x > 84) 
    {
      blackLeft = blackLeft + periphLeft;
      blackMid = blackMid + midSensor;
      blackRight = blackRight + periphRight;
    }
  }
  
  blackLeft = blackLeft/15;
  blackMid = blackMid/15;
  blackRight = blackRight/15;

  digitalWrite(lighting, LOW);
}

/*
This method takes the average of 15 readings of the LDR sensors on the background used for stopping the car. 
The if structure is used to take the readings starting from the 85th one to the 100th one. This is required for 
accurate values.
*/
void calibrateStop () 
{
  delay (3000);

  for (int x=0; x<100; x++) 
  {
    digitalWrite(lighting, HIGH);
    leftSensor = analogRead(1) + leftDifference;
    midSensor = analogRead(2);
    rightSensor = analogRead(3) + rightDifference;
    
    if (x > 84) 
    {
      stopAvg = stopAvg + (leftSensor + midSensor + rightSensor)/3;
    }
  }
  
  stopAvg = stopAvg / 15;
  
  digitalWrite(lighting, LOW);
}

/*
This method sets up the outputs, turns on the lighting and the motors, and calibrates the LDR sensors.
*/
void setup()
{
  Serial.begin (9600);

  pinMode(lighting, OUTPUT);
  
  pinMode(rightMotorSpeed, OUTPUT);
  pinMode(leftMotorSpeed, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);

  calibrate();
  calibrateWhite ();
  calibrateBlack ();
  calibrateStop ();

  delay(3000);
  digitalWrite(lighting, HIGH);
  delay(100);
  
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(leftMotorForward, HIGH);

  analogWrite(rightMotorSpeed,left);
  analogWrite(leftMotorSpeed,right);
}

/*
This is the main method of the program. All of the processing takes place here. First, 
the values of the 3 central LDRs are read in. Next, the left and right sensors are 
compared to the middle one with the use of their differences from the middle LDR. If 
the left or right sensors are are off of the required calibrated value, speed ajustments 
are made for each motor to adjust the direction that the car is going in.

Then, the peripheral LDR values are read in. The peripheral sensors are used mainly for 
right angle turns and maze solving. If black is detected on the left sensor, the corresponding 
boolean expression is switched to false and the time at which the detection occured is recorded. 
If the right peripheral sensor sees black, a right turn is initiated right away, disregarding 
other circumstances. The next if structure checks if the sensors see white. If yes, other 
conditions are checked to decide the direction and method of turn to be initiated. As soon as 
white is seen, the millis () time is checked. If the sensors saw left less than 1 second ago, 
a left turn is initiated. Otherwise, a hard turn on the spot is intialized instead, since the 
car is assumed to have reached the end of the line. The following 2 else if's are added mainly 
as safeguards, since the car is designed to turn right without even taking into account the 
conditions described in the else if's. More specifically, reaching white before attempting a 
turn is unnecessary. Also, in the situation when both peripheral sensors detect black 
(a t-intersection), the car always defaults to a right hand turn, since it is designed to 
solve mazes using the right hand rule.

Here is are the car's direction preferences:
1. A right turn is always preferred over going straight or turning left
2. Going straight is preferred over going left.
Using the right hand rule method, the car is able to solve any maze that does not involve loops.
*/
void loop() 
{
  left = startSpeed;
  right = startSpeed;

  unsigned long leftDetected, onWhite;

  leftSensor = analogRead(1) + leftDifference;
  midSensor = analogRead(2);
  rightSensor = analogRead(3) + rightDifference;

  if (leftSensor > (midSensor+threshold)) 
  {
    left = startSpeed + scan;
    right = startSpeed - scan;
  }
  if (rightSensor > (midSensor+threshold)) 
  {
    left = startSpeed - scan;
    right = startSpeed + scan;
  }
  analogWrite(rightMotorSpeed,left);
  analogWrite(leftMotorSpeed,right);
  
  periphLeft = analogRead(0);
  if (periphLeft > blackLeft - 15 && periphLeft < blackLeft + 15) 
  {
    leftDetected = millis ();
    sawLeft = true;
  }
  periphRight = analogRead(4);
  if (periphRight > blackRight - 15 && periphRight < blackRight + 15) 
  {
    sawRight == true;
    spinRight ();
  }

  leftSensor = analogRead(1) + leftDifference;
  midSensor = analogRead(2);
  rightSensor = analogRead(3) + rightDifference; 
  if ((leftSensor+midSensor+rightSensor)/3 > whiteAvg - 15 && (leftSensor+midSensor+rightSensor)/3 < whiteAvg + 15) 
  {
    onWhite = millis ();
    if (sawLeft == true) 
    {
      if (onWhite - leftDetected < 1000)
        spinLeft ();
      else
      {
        sawLeft == false;
        hardSpin ();
      }
    }
    else if (sawRight == true) 
    {
      spinRight ();
    } 
    else if (sawLeft == true && sawRight == true)
    {
      sawLeft = false;
      spinRight ();
    }
    else if ((leftSensor+midSensor+rightSensor)/3 > whiteAvg - 15 && 
      (leftSensor+midSensor+rightSensor)/3 < whiteAvg + 15) 
    {
      hardSpin ();
    }  
  }
  
  if ((leftSensor+midSensor+rightSensor)/3 > stopAvg - 15 && 
    (leftSensor+midSensor+rightSensor)/3 < stopAvg + 15) 
  {
    digitalWrite(rightMotorForward, LOW);
    digitalWrite(leftMotorForward, LOW);
  }

  analogWrite(rightMotorSpeed,left);
  analogWrite(leftMotorSpeed,right);
}

/*
This method is used to turn the car on right angle left turns. The left motor is stopped, 
so the car spins left on its left wheel until the 3 middle censors reach the preferred values 
and the car is centered on the line.
*/
void spinLeft ()
{
  sawRight, sawLeft == false;
  do 
  {
    left = startSpeed-5;
    right = startSpeed-5;

    digitalWrite(leftMotorForward, LOW);
    analogWrite(rightMotorSpeed,left);
    analogWrite(leftMotorSpeed,right);

    leftSensor = analogRead(1) + leftDifference;
    midSensor = analogRead(2);
    rightSensor = analogRead(3) + rightDifference;
  } 
  while (!(leftSensor > midSensor - 45 && leftSensor < midSensor + 45 && rightSensor > midSensor - 45 &&
    rightSensor < midSensor + 45));
  
  digitalWrite(leftMotorForward, HIGH);
}

/*
This method is used to turn the car on right angle right turns. The right motor is stopped, 
so the car spins right on its right wheel until the 3 middle censors reach the preferred values 
and the car is centered on the line.
*/
void spinRight ()
{
  sawRight, sawLeft = false;
  do 
  {
    left = startSpeed-5;
    right = startSpeed-5;

    digitalWrite(rightMotorForward, LOW);
    analogWrite(rightMotorSpeed,left);
    analogWrite(leftMotorSpeed,right);
  
    leftSensor = analogRead(1) + leftDifference;
    midSensor = analogRead(2);
    rightSensor = analogRead(3) + rightDifference;
  } 
  while (!(leftSensor > midSensor - 45 && leftSensor < midSensor + 45 && rightSensor > midSensor - 45 && rightSensor < midSensor + 45));
  
  digitalWrite(rightMotorForward, HIGH);
}

/*
This method is used to turn the car 180 degrees when white is detected by the 3 middle censors, 
signifying that the end of the line has been reached. One of the motors runs in reverse, while 
the other one keeps moving forward. This has the effect of turning the car more sharply, with 
a smaller turn radius.
*/
void hardSpin ()
{
  sawRight, sawLeft = false;
  do 
  {
    left = startSpeed;
    right = startSpeed;
    
    digitalWrite(leftMotorForward, LOW);
    analogWrite(rightMotorSpeed,left);
    analogWrite(leftMotorSpeed,right);
    digitalWrite(leftMotorReverse, HIGH);
    
    leftSensor = analogRead(1) + leftDifference;
    midSensor = analogRead(2);
    rightSensor = analogRead(3) + rightDifference;
  } 
  while (!(leftSensor > midSensor - 45 && leftSensor < midSensor + 45 && rightSensor > midSensor - 45 && rightSensor < midSensor + 45));
  
  digitalWrite (leftMotorReverse, LOW);
  digitalWrite(leftMotorForward, HIGH);
}


















