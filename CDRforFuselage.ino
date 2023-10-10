/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h> //This is used to connect the pins of Microcontroller to the Servo Driver (Came with Servo Driver Program)
#include <Adafruit_PWMServoDriver.h> // This library is required to run the Servo Driver (Came with Servo Driver Program)

#define trigPin 9 // This is to allocate the Digital Pin Number 9 for the Trigger Pin of Ultrasonic Sensor
#define echoPin 10 // This is to allocate the Digital Pin Number 10 for the Echo Pin of Ultrasonic Sensor 
#define buzzer 12 // This is to allocate the Digital Pin Number 10 for the positive side of the Buzzer. This Pin gives the required signal to operate the buzzer. The other pin of the Buzzer is connected to one of Ground Pin.
long duration; // Allocating the data type for duration as long
int distance; // Allocating the data type for distance as int
int safetyDistance; // Allocating the data type for safetyDistance as int

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //Not sure but it might allot the value of the function (right hand side) to the variable pwn (left hand side) (Came with Servo Driver Program)
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0; //Counter for the Servo (Came with Servo Driver Program)

void moveServo(int servo, int angle) //Function to move the servos according to the angle. The original program before writing this function was based on giving the pulselength or something. This function takes the servo number & the angle to run the servo.
{
    int pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servo, 0, pulselength);
}

void TopLeft_BaseInitial() //Function to initialize the position for the Base of Top Left Arm (the arm which is connected among the first four pins of the servo driver).
{
  moveServo(0,0); // First '0' is the pin number to which the servo motor of the base is connected to. The Second '0' is the angle which it has to move. Since this is to initialize the position the value of angle is 0.
  delay(500); // Delay of 500ms assumed by me to give stability & also to lower the speed of the motor in whole.
}
void TopLeft_BaseChange() //Function to rotate the Base of Top Left Arm. This is rotate the arm by 90 degree.
{
  moveServo(0,90);
  delay(500);
}
void TopLeft_PentDown() //Function to rotate the servo motor on the pentagonal face so that the arm moves down. All the values found in this, above & below functions are based on the testing & finding the perfect values for that position. The values can be changed or will change each time the arm is taken apart and calibrated.
{
  moveServo(1,120);
  delay(500);
}
void TopLeft_PentUp() //Function to rotate the servo motor on the pentagonal face so that the arm moves up.
  moveServo(1,70);
  delay(500);
}
void TopLeft_Position() //Function to position the servo motor on the square face so that the arm remains straight in terms of two joints.
{
  moveServo(2,36);
  delay(500);
}

void TopRight_BaseInitial() //Same goes for Top Right arm.
{
  moveServo(4,90);
  delay(500);
}
void TopRight_BaseChange()
{
  moveServo(4,0);
  delay(500);
}
void TopRight_PentDown()
{
  moveServo(5,55);
  delay(500);
}
void TopRight_PentUp()
{
  moveServo(5,5);
  delay(500);
}
void TopRight_Position()
{
  moveServo(6,45);
  delay(500);
}

void BottomRight_BaseInitial()
{
  moveServo(8,0);
  delay(500);
}
void BottomRight_BaseChange()
{
  moveServo(8,90);
  delay(500);
}
void BottomRight_PentDown()
{
  moveServo(9,122);
  delay(500);
}
void BottomRight_PentUp()
{
  moveServo(9,72);
  delay(500);
}
void BottomRight_Position()
{
  moveServo(10,30);
  delay(500);
}

void BottomLeft_BaseInitial()
{
  moveServo(12,90);
  delay(500);
}
void BottomLeft_BaseChange()
{
  moveServo(12,0);
  delay(500);
}
void BottomLeft_PentDown()
{
  moveServo(13,115);
  delay(500);
}
void BottomLeft_PentUp()
{
  moveServo(13,65);
  delay(500);
}
void BottomLeft_Position()
{
  moveServo(14,36);
  delay(500);
}

void Initial() //Function to initialize all the motor. Either this or the above initialization program was used for the main program. The best one was selected during the time to prevent the errors.
{
  moveServo(0,0);
  moveServo(1,120);
  moveServo(2,36);

  moveServo(4,90);
  moveServo(5,55);
  moveServo(6,45);

  moveServo(8,0);
  moveServo(9,122);
  moveServo(10,30);

  moveServo(12,90);
  moveServo(13,1150);
  moveServo(14,36);
  delay(500);
}

void Move() //The function to move all the motors so that the robot moves front. This is the step in the middle of all the operations of the robot.
{
  moveServo(0,0);
  moveServo(4,90);
  moveServo(8,90);
  moveServo(12,0);
  delay(50);
  
  moveServo(0,10);
  moveServo(4,80);
  moveServo(8,80);
  moveServo(12,10);
  delay(50);
  
  moveServo(0,20);
  moveServo(4,70);
  moveServo(8,70);
  moveServo(12,20);
  delay(50);

  moveServo(0,30);
  moveServo(4,60);
  moveServo(8,60);
  moveServo(12,30);
  delay(50);

  moveServo(0,40);
  moveServo(4,50);
  moveServo(8,50);
  moveServo(12,40);
  delay(50);

  moveServo(0,50);
  moveServo(4,40);
  moveServo(8,40);
  moveServo(12,50);
  delay(50);

  moveServo(0,60);
  moveServo(4,30);
  moveServo(8,30);
  moveServo(12,60);
  delay(50);

  moveServo(0,70);
  moveServo(4,20);
  moveServo(8,20);
  moveServo(12,70);
  delay(50);

  moveServo(0,80);
  moveServo(4,10);
  moveServo(8,10);
  moveServo(12,80);
  delay(50);

  moveServo(0,90);
  moveServo(4,0);
  moveServo(8,0);
  moveServo(12,90); 
}

void Scan() // Program to detect the cracks using Ultrasonic Sensors. This function was copied.
{
 digitalWrite(trigPin, LOW); //Makes the Trigger Pin Low.
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH); //Trigger Pin is made High after 2 micro seconds. This step sends the waves from ultrasonic sensor which will be used to calculate the distance later.
delayMicroseconds(10);
digitalWrite(trigPin, LOW); //Trigger Pin is made Low. Here the sample mention in the above explanation is completely created.

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH); //Calculates the time taken by the echopin to detect the sample wave sent by the trigger pin.

// Calculating the distance
distance= duration*0.034/2; //Using the following relation, it converts the duration into distance.

safetyDistance = distance; // This is to allot the distance measured for comparing with the actual safety distance (value is 15) allocated in the program. Even if the name of the variable is "safetyDistance", it is not the actual safety distance value but the value to compare with the actual safety distance. 
if (safetyDistance > 15){ //The step where the program compares whether the given value 15 (around 6 to 7 cms) is lesser than the measured value.
  digitalWrite(buzzer, HIGH); //If the measured value is greater than the actual safety distance, then it means that the distance between the sensor and the surface to which it reflects is more. This means that there is a presence of crack and crack is causing the distance between the surface and the sensor to increase. With this the buzzer is made high to alarm. Since LED's pins were shorted with the buzzer, the LED will glow.
  delay(1000); // Delay of 1000ms was given demonstration purpose. The value can be increased. Because of this, the buzzer will continue to ring for a long time.
}
else{
  digitalWrite(buzzer, LOW); // If the distance is less, then the buzzer will remain Low and it won't ring.
}

// Prints the distance on the Serial Monitor
Serial.print("Distance: "); //This part will be used to print the distance.
Serial.println(distance);
}

void setup() { //Function that does the initial setup of all the devices in the robot.
  Serial.begin(9600); // This will be used to initiate the serial communication so that the distance information from the microcontroller will be sent to the PC.
  pwm.begin(); // Function to initiate the Servo Driver.
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates //Sets the frequency of the servo driver. I just realized that by changing the frequency value over here, we can the speed of the motor. I should have saw this part a month back. 

  yield(); //Some initialization which I don't remember. Please Google.
  pinMode(trigPin, OUTPUT); //Initializing the Trigger pin as output
  pinMode(echoPin, INPUT); //Initializing the Echo pin as input
   pinMode(buzzer, OUTPUT); //Initializing the Buzzer as Output
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) { //Came with servo driver program. Not sure how it works.
  double pulselength;
  
  pulselength =  1000000;  // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() //Main Progam
{
  Scan(); //Call the scan function to start the detection operations. This works for a moment.
  Initial(); //Function to initialize the motor positions.
  Scan(); //Call the scan function in the next moment. I am not sure about the unit of the 'moment' in terms of time.
  delay(500);
  Scan();
  BottomRight_PentUp();
  Scan();
  BottomRight_BaseChange();
  Scan();
  BottomRight_PentDown(); 
  Scan();

  BottomLeft_PentUp();
  Scan();
  BottomLeft_BaseChange();
  Scan();
  BottomLeft_PentDown(); 
  Scan();
  delay(500);
  Scan();
  
  Move(); //Calls the Move function to move the whole robot front
  Scan();
  delay(500);
  Scan();
  
  TopLeft_PentUp();
  Scan();
  TopLeft_BaseInitial();
  Scan();
  TopLeft_PentDown();
  Scan();
  delay(500);
  Scan();

  TopRight_PentUp();
  Scan();
  TopRight_BaseInitial();
  Scan();
  TopRight_PentDown(); 
  Scan();

  delay(500);
  Scan();
  //
  //TopLeft_BaseInitial();
  //TopLeft_PentDown();  
  //TopLeft_Position();
  //TopLeft_PentUp();
  //TopLeft_BaseChange();
  //TopLeft_PentDown(); 
  
  //TopRight_BaseInitial();
  //TopRight_PentDown();  
  //TopRight_Position();
  //TopRight_PentUp();
  //TopRight_BaseChange();
  //TopRight_PentDown(); 
  
  //BottomRight_BaseInitial();
  //BottomRight_PentDown();  
  //BottomRight_Position();
  //BottomRight_PentUp();
  //BottomRight_BaseChange();
  //BottomRight_PentDown(); 
 
  //BottomLeft_BaseInitial();
  //BottomLeft_PentDown();  
  //BottomLeft_Position();
  //BottomLeft_PentUp();
  //BottomLeft_BaseChange();
  //BottomLeft_PentDown(); 
  //delay(20000);
}
