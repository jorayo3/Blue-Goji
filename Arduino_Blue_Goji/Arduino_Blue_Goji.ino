//Digital Pin Numbers
int dcMotorPWMA = 5;
int dcMotorPWMB = 6;
int signal2belt = 9;

//Analog Pin Numbers
int motorPot = 1;
int ResistanceCTRL = 5;

//Motor Parameters
//int potMin = 0; //Determine this reading from the bike
//double potMaxV = 1.67; //Determine this reading from the bike
//int potMax = potMaxV/5*1023; //potMaxV in terms of 1024 analog reading (max around 360)
int potMin = 290;
int potMax = 590;
int potDesRaw;
int potDesRawMap;
double potDes; //Resistance value here
double tol = 5; //Resistance pot reading error tolerance

//Variables
int resistance;
int timeHigh;
int timeLow;
int PWM = 100; //values 0-255

//Pedal Velocity Calculations//

#define readEncoder bitRead(PIND,2)//faster than digitalRead()

const byte encoderPin = 2;//encoder output, digital pin

volatile long count = 0; //current number of total encoder counts
volatile long previousCount = 0; //current number of total encoder counts
volatile long intervalCount = 0; //current number of total encoder counts

long vel;
float pedalSpeed = 0; //RPM

unsigned long currentTime; 
unsigned long previousTime = 0;
const int interval = 1000; // One second

void isr() { //increment or decrement the encoder count if channel changes
    count ++; //increment
}


void calcSpeed() {
    intervalCount = count - previousCount;
    pedalSpeed = intervalCount*60000/8 /interval; //Revolutions per interval(1000 msec) times 60000 millisec = RPM
    previousCount = count;
    previousTime = currentTime;
}

//End Pedal Velocity Calculations// 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(motorPot, INPUT);

  //Pedal Velocity Setup
  pinMode(encoderPin, INPUT);  
  attachInterrupt(digitalPinToInterrupt(encoderPin), isr, HIGH); //encoder  initialization
}

void loop() {
  //Pedal Velocity Calculation//
  currentTime = millis();
  if (currentTime - previousTime >= interval) {calcSpeed();} //Every interval (1second right now), calculate speed
  //End Pedal Velocity Calculation//

  potDesRaw = analogRead(ResistanceCTRL);
  potDesRawMap = map(potDesRaw, 0, 1023, 0, 255);
  analogWrite(signal2belt, potDesRawMap); //Send signal to belt
  potDes = map(potDesRaw, 0, 1023, potMax, potMin); //Pot controlled Resistance
  //Serial.println(count);
  resistance = analogRead(motorPot);
  
  if (resistance <= potDes - tol && resistance >= potMin){
    analogWrite(dcMotorPWMA,0);
    analogWrite(dcMotorPWMB,PWM);
  }
  else if (resistance >= potDes + tol && resistance <= potMax){
    analogWrite(dcMotorPWMA,PWM);
    analogWrite(dcMotorPWMB,0);
  }
  else{
    analogWrite(dcMotorPWMA,0);
    analogWrite(dcMotorPWMB,0);
  }

  //LOGGER
 Serial.print("COUNT: ");
 Serial.print(count);
 Serial.print("  CurrTime: ");
 Serial.print(currentTime);
 Serial.print("  IntervalCount: ");
 Serial.print(intervalCount);
 Serial.print("  Motor Pot: ");
 Serial.print(resistance);
 Serial.print("  Desired MotorPos: ");
 Serial.print(potDes);
 Serial.print("  potDesRaw: ");
 Serial.print(potDesRaw);
 Serial.print("  potDesRawMap: ");
 Serial.print(potDesRawMap);
 Serial.print("  PedalSpeed: ");
 Serial.println(pedalSpeed);
}
