 
  // set the power. Max is 255
  #define POWER 150
  // Rising and falling edges per rotation
  #define TICKS_PER_REVOLUTION 449
  
  byte encA = 2 ; // interrupt pin for the wheel encoder 
  byte pwn  = 8 ; // power control through PWM
  byte dir1 = 9 ; // for contoring the direction of the motor rotation 
  byte dir2 = 10; // for contoring the direction of the motor rotation 
  
  unsigned long prev_time = 0;
  unsigned long prev_ticks_cntr = 0;
  // use volatile to tell the compile that these variables 
  // can be changed by the hardware (i.e., by ISR)
  volatile unsigned long ticks_cntr = 0;

  float prev_delta_ticks1 = 0;
  float prev_delta_ticks2 = 0;

  // Low pass filter
  float rpmFilt = 0.0;
  float prevRpmFilt = 0.0;
  float prevRpm = 0.0;
  int idx = 0;
  void setup(){
    Serial.begin(115200);
    Serial.println("Filtered:, Raw:");
    // set GPIO pin modes 
    pinMode(encA, INPUT_PULLUP);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwn, OUTPUT);
    
    // attach an interrupt to pin `encA`
    // call encA_ISR on a rising/falling edge of the incoming signal 
    attachInterrupt(digitalPinToInterrupt(encA), encA_ISR, CHANGE);
    
    // set the direction of the motor rotation 
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  
    // power the motor
    analogWrite(pwn, POWER);
  }

  // The interrupt service routine will be called by the hardware
  void encA_ISR(){ ticks_cntr += 1; }

float rotation_count(){
    float delta_ticks   = ((float) (ticks_cntr - prev_ticks_cntr) + 
    prev_delta_ticks1 + prev_delta_ticks2) / 3.0;
    prev_delta_ticks2 = prev_delta_ticks1;
    prev_delta_ticks1 = delta_ticks;
    prev_ticks_cntr     = ticks_cntr;
    return delta_ticks / TICKS_PER_REVOLUTION;
  }

  void loop(){
    float time_dif = (float) (micros() - prev_time) ;
    
    if (time_dif >= 10000){ 
      float  rotations  = rotation_count();
      prev_time += time_dif;  
      float minutes  = time_dif/ 60.0e6;
      float rpm = rotations / minutes; 

       // Low pass filter
      float rpmFilt = .521 * prevRpmFilt + 0.239 * rpm + 0.239 * prevRpm;
      prevRpmFilt = rpmFilt;
      prevRpm = rpm;

      // output to the screen through serial port
      idx +=1;
      if (idx >= 3) {
        Serial.print(rpmFilt);
        Serial.print(" ");
        Serial.print(rpm);
        Serial.println();
        idx=0;
      }
    }
  }








  
  
