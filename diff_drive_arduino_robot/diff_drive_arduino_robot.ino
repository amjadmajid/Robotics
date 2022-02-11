#include <PID_v1.h>


# define TICKS_PER_10_CM 104
# define LOOPTIME 10

unsigned long current_millis=0;
unsigned long prev_millis=0;

// Tachometer
byte m1_int_pin =  2;
byte m2_int_pin =  3;
volatile unsigned long m1_ticks_cntr=0;
volatile unsigned long m2_ticks_cntr=0;
unsigned long m1_prev_ticks_cntr=0;
unsigned long m2_prev_ticks_cntr=0;
unsigned long m1_ticks_diff=0;
unsigned long m2_ticks_diff=0;

// Motor 1
byte m1_dir_pin_a = 8;
byte m1_dir_pin_b = 9;
byte m1_pin_speed = 10;
double m1_speed     = 0;  // cm per second ( max 100cm/s)

// Motor 2
byte m2_dir_pin_a = 12;
byte m2_dir_pin_b = 13;
byte m2_pin_speed = 11;
double m2_speed     = 0;

// Control
double m1_setpoint=0, m1_input=0, m1_output=0;
double m2_setpoint=0, m2_input=0, m2_output=0;
double m1_kp = 3.7, m1_ki=0, m1_kd=0;
double m2_kp = 3.7, m2_ki=0, m2_kd=0;

PID m1_pid(&m1_input, &m1_output, &m1_setpoint, m1_kp, m1_ki, m1_kd, DIRECT);
PID m2_pid(&m2_input, &m2_output, &m2_setpoint, m2_kp, m2_ki, m2_kd, DIRECT);

void m1_stop(){ analogWrite(m1_pin_speed, 0); }
void m2_stop(){ analogWrite(m2_pin_speed, 0); }

void m1_direction( int dir){
  if(dir == 1){
    digitalWrite(m1_dir_pin_a, HIGH);
    digitalWrite(m1_dir_pin_b, LOW);
  }else if (dir == -1){
    digitalWrite(m1_dir_pin_a, LOW);
    digitalWrite(m1_dir_pin_b, HIGH);   
  }else{
    digitalWrite(m1_dir_pin_a, LOW);
    digitalWrite(m1_dir_pin_b, LOW);  
  }
}

void m2_direction( int dir){
  if(dir == 1){
    digitalWrite(m2_dir_pin_a, HIGH);
    digitalWrite(m2_dir_pin_b, LOW);
  }else if (dir == -1){
    digitalWrite(m2_dir_pin_a, LOW);
    digitalWrite(m2_dir_pin_b, HIGH);   
  }else{
    digitalWrite(m2_dir_pin_a, LOW);
    digitalWrite(m2_dir_pin_b, LOW);  
  }
}

void m1_pins_setup(){
  pinMode(m1_dir_pin_a, OUTPUT);
  pinMode(m1_dir_pin_b, OUTPUT);
  pinMode(m1_pin_speed, OUTPUT);
  pinMode(m1_int_pin, INPUT_PULLUP);
  m1_direction(1);
  attachInterrupt(digitalPinToInterrupt(m1_int_pin), m1_encoder, RISING);
}

void m2_pins_setup(){
  pinMode(m2_dir_pin_a, OUTPUT);
  pinMode(m2_dir_pin_b, OUTPUT);
  pinMode(m2_pin_speed, OUTPUT);
  pinMode(m2_int_pin, INPUT_PULLUP);
  m2_direction(1);
  attachInterrupt(digitalPinToInterrupt(m2_int_pin), m2_encoder, RISING);
}

void m1_encoder(){ m1_ticks_cntr++; }

void m2_encoder(){ m2_ticks_cntr++; }

void setup() {
      m1_pins_setup();
      m2_pins_setup();
    
      m1_pid.SetMode(AUTOMATIC);
      m1_pid.SetSampleTime(1);
      m1_setpoint = m1_speed;
    
      m2_pid.SetMode(AUTOMATIC);
      m2_pid.SetSampleTime(1);
      m2_setpoint = m2_speed;
    
      Serial.begin(9600);
}

void bluetooth_control(){
    while(Serial.available()>0){
      char inputByte= Serial.read();
      Serial.println(inputByte);
      if (inputByte=='L'){
          m1_speed = 80;
          m2_speed = 30;
      }else if (inputByte=='R'){
          m1_speed = 30;
          m2_speed = 80;
      } else if (inputByte == 'F'){
          m1_direction(1);
          m2_direction(1);
          m1_speed = 70;
          m2_speed = 70;
      }else if(inputByte == 'S'){
          m1_speed = 0;
          m2_speed = 0;
      }else if(inputByte == 'B'){
          m1_direction(-1);
          m2_direction(-1);
          m1_speed = 60;
          m2_speed = 60;
      }
            Serial.println(inputByte);
     }
}

int distance_in_ticks(unsigned long currTicks, unsigned long *prevTicks){
    int ticksDif =  currTicks - *prevTicks;
    *prevTicks = currTicks;
    return ticksDif;
}

// low pass filters
float m1_filt_act_speed = 0, m2_filt_act_speed = 0;
float m1_prev_act_speed = 0, m2_prev_act_speed = 0;

void loop() {
  current_millis = millis();
  bluetooth_control();
  if(current_millis - prev_millis >= LOOPTIME){
      m1_setpoint = m1_speed;
      m2_setpoint = m2_speed;
      prev_millis = current_millis;

      m1_ticks_diff = distance_in_ticks(m1_ticks_cntr, &m1_prev_ticks_cntr);
      m2_ticks_diff = distance_in_ticks(m2_ticks_cntr, &m2_prev_ticks_cntr);

      // (we are checking every 10 milliseconds) speed in centimeter per second (cm/s)
      int m1_act_speed = (m1_ticks_diff * 1000)  / (TICKS_PER_10_CM-(16 * m1_speed)/100 );
      
      // (we are checking every 10 milliseconds) speed in centimeter per second (cm/s)
      int m2_act_speed = (m2_ticks_diff * 1000)  / (TICKS_PER_10_CM);

     // Low-pass filter (25 Hz cutoff)
      m1_filt_act_speed = 0.854*m1_filt_act_speed + 0.0728*m1_act_speed + 0.0728*m1_prev_act_speed;
      m1_prev_act_speed = m1_act_speed;
      m2_filt_act_speed = 0.854*m2_filt_act_speed + 0.0728*m2_act_speed + 0.0728*m2_prev_act_speed;
      m2_prev_act_speed = m2_act_speed;
    
      m1_input =  m1_filt_act_speed;
      m2_input =  m2_filt_act_speed;
      
      m1_pid.Compute();
      m2_pid.Compute();
      analogWrite(m1_pin_speed,m1_output);
      analogWrite(m2_pin_speed,m2_output);

//      Serial.println(m1_output);
//      Serial.println(m2_output);

  }
}
