// set the power. Max is 255
#define POWER 255

#define TICKS_PER_REVOLUTION 224.4
byte encA = 2 ; // interrupt pin for the wheel encoder 
byte pwn  = 8 ; // power control through PWM
byte dir1 = 9 ; // for contoring the direction of the motor rotation 
byte dir2 = 10; // for contoring the direction of the motor rotation 

long prev_time = 0;
long prev_ticks_cntr = 0;
// use volatile to tell the compile that these variables 
// can be changed by the hardware (i.e., by ISR)
volatile long ticks_cntr = 0;

void setup(){
  
  Serial.begin(9600);
  // set GPIO pin modes 
  pinMode(encA, INPUT_PULLUP);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwn, OUTPUT);
  // attach an interrupt to pin `encA`
  // call encA_ISR on a rising edge of the incoming signal 
  attachInterrupt(digitalPinToInterrupt(encA), encA_ISR, RISING);
  
  // set the direction of the motor rotation 
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);

  // power the motor
  analogWrite(pwn, POWER);
}

long elapsedTime(){
  long curr_time = micros();
  long delta_time = curr_time - prev_time ;
  return delta_time;
}

int distanceTraveled(){
  int ticks_dif     = (int) ticks_cntr - prev_ticks_cntr;
  prev_ticks_cntr   = ticks_cntr;
  return ticks_dif;
}

void loop(){
  long time_dif  = elapsedTime();
  if (time_dif >= 5e4){ // check every 10 milliseconds
    int   dis   = distanceTraveled();
    prev_time += time_dif;  // update the previous time for the elapsedTime()
    float velocity = dis / (time_dif/1.0e6);  // velocity in ticks per second
  
    // velocity in rotation per minute
    float v = velocity / TICKS_PER_REVOLUTION * 60.0 ;
  
    // output to the screen through serial port
    Serial.println(v);
  }
}

// The interrupt service routine will be called by the hardware
void encA_ISR(){
  ticks_cntr += 1;
}
