#include <PID_v1.h>
#define TICKS_PER_REVOLUTION 356.0

byte encA = 3, encB = 2;
byte dir1 = 4, dir2 = 5;
byte pwn = 6;

// use volatile to tell the compile that these variables 
// can be changed by the hardware (i.e., by ISR)
volatile int ticks_cntr = 0;

// pid vairalble 
double input=0, output=0, setPoint=0, kp=3.6, ki=2.6, ke=0;
PID speed_pid =  PID(&input,&output, &setPoint, kp, ki, ke, DIRECT); 

void pinsSetup(){
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwn, OUTPUT);
}

void setMotor(byte dir){
  if (dir ==1){
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  }else if (dir == -1){
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  }else{
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
  }
}

void setup(){
  Serial.begin(9600);
  
  // pins
  pinsSetup();
  attachInterrupt(digitalPinToInterrupt(encA), encA_ISR, RISING);
  
  // motor
  byte dir = 1;
  setMotor(dir);
  
  // control
  setPoint = 50;  // this is in rpm since the input to the pid in rpm
  speed_pid.SetMode(AUTOMATIC);
  speed_pid.SetSampleTime(1);
}

float elapsedTime(){
  // This function returns the time elapsed between two calls.
  // First first call return the time between the call and the 
  // the start of the program.
  static long prevT = 0;
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/1.0e6 ;
  prevT = currT;
  return deltaT;
}

float distanceTraveled(){
  static int prevticks_cntr = 0;
  int _ticks_cntr = 0;
  noInterrupts(); // access the volitale variable safely. 
  _ticks_cntr = ticks_cntr;
  interrupts();
  int deltaD = _ticks_cntr - prevticks_cntr;
  prevticks_cntr = _ticks_cntr;
  return deltaD;
}

void encA_ISR(){
  byte b = digitalRead(encB);
  int increment = 0;
  if(b == 1){
    increment = 1;
  }else{
    increment  = -1;
  }
  ticks_cntr += increment;
}

void loop(){
  static long prevControlTime = 0;
  // low pass filter 
  static float vFilt = 0;
  static float vPrev = 0;
  long currControlTime = millis();
//  int contrTime = map(setPoint, 0,120, 6, 40);
  if (currControlTime - prevControlTime > 10  ){
//    setPoint = 100*(sin(currControlTime/1e3)>0);
    float time = elapsedTime();
    int dis = distanceTraveled();
    float velocity = dis / time;  // velocity in ticks per second
    // velocity in RPM
    float v = velocity / TICKS_PER_REVOLUTION * 60.0 ;

     // Low-pass filter (25 Hz cutoff)
    vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
    vPrev = v;
    
    input = vFilt; 
    speed_pid.Compute();
    analogWrite(pwn, output);
    
    Serial.print(vFilt);
    Serial.print(" ");
    Serial.println(output);
    prevControlTime = currControlTime;
  }
}
