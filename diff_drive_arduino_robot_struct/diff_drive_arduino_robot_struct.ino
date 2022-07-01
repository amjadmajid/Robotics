#include <PID_v1.h>

# define TICKS_PER_10_CM 104
# define LOOPTIME 10

// global
volatile long m_1_TicksCntr=0;
volatile long m_2_TicksCntr=0;

typedef void (*func_ptr)(void);

struct tachometer_t{
  // using volatile as this variable will be changed by hardware (ISR)
  volatile long ticks;
  long prevTicks;
  byte intPin;
  func_ptr func_t;
};

struct motor_t {
  byte dir_a;
  byte dir_b;
  byte pwm;
  double speed; 
  tachometer_t tacho;
};

unsigned long current_millis=0;
unsigned long prev_millis=0;

// Control
double m1_setpoint=0, m1_input=0, m1_output=0;
double m2_setpoint=0, m2_input=0, m2_output=0;
double m1_kp = 3.7, m1_ki=0, m1_kd=0;
double m2_kp = 3.7, m2_ki=0, m2_kd=0;

PID m1_pid(&m1_input, &m1_output, &m1_setpoint, m1_kp, m1_ki, m1_kd, DIRECT);
PID m2_pid(&m2_input, &m2_output, &m2_setpoint, m2_kp, m2_ki, m2_kd, DIRECT);

void m1_encoder(void);
void m2_encoder(void);

tachometer_t m1_tacho = {0,0,2, &m1_encoder};
tachometer_t m2_tacho = {0,0,3, &m2_encoder};
motor_t m_1 = {10,9,8, 0, m1_tacho};
motor_t m_2 = {12,11,13, 0, m2_tacho};

void m1_encoder(void){ m_1.tacho.ticks++; }
void m2_encoder(void){ m_2.tacho.ticks++; }


void stop(motor_t m){ analogWrite(m.pwm, 0); }

void direction( motor_t m, int dir){
  if(dir == 1){
    digitalWrite(m.dir_a, HIGH);
    digitalWrite(m.dir_b, LOW);
  }else if (dir == -1){
    digitalWrite(m.dir_a, LOW);
    digitalWrite(m.dir_b, HIGH);   
  }else{
    digitalWrite(m.dir_a, LOW);
    digitalWrite(m.dir_b, LOW);  
  }
}

void pins_setup(motor_t m){
  pinMode(m.dir_a, OUTPUT);
  pinMode(m.dir_b, OUTPUT);
  pinMode(m.pwm, OUTPUT);
  pinMode(m.tacho.intPin, INPUT_PULLUP);
  direction(m, 1);
  attachInterrupt(digitalPinToInterrupt(m.tacho.intPin), m.tacho.func_t, RISING);
}


void setup() {
      pins_setup(m_1);
      pins_setup(m_2);

      m1_pid.SetMode(AUTOMATIC);
      m1_pid.SetSampleTime(1);
      m1_setpoint = m_1.speed;
    
      m2_pid.SetMode(AUTOMATIC);
      m2_pid.SetSampleTime(1);
      m2_setpoint = m_2.speed;
    
      Serial.begin(9600);
      Serial1.begin(9600);
}

void bluetooth_control(){
    while(Serial1.available()>0){
      delay(1);
      char inputByte= Serial1.read();
      Serial.println(inputByte);

      if(inputByte == 'B'){
          // move backward
          direction(m_1,-1);
          direction(m_2,-1);
          m_1.speed = 60;
          m_2.speed = 60;
      }else{
          // wheels turn forward (maybe at zero speed!)
          direction(m_1,1);
          direction(m_2,1);
          if (inputByte=='L'){
              m_1.speed = 80;
              m_2.speed = 30;
          }else if (inputByte=='R'){
              m_1.speed = 30;
              m_2.speed = 80;
          } else if (inputByte == 'F'){
              m_1.speed = 70;
              m_2.speed = 70;
          }else{
              m_1.speed = 0;
              m_2.speed = 0;
          }
      }
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
      m1_setpoint = m_1.speed;
      m2_setpoint = m_2.speed;
      prev_millis = current_millis;

      int m1_ticks_diff = distance_in_ticks(m_1.tacho.ticks, &m_1.tacho.prevTicks);
      int m2_ticks_diff = distance_in_ticks(m_2.tacho.ticks, &m_2.tacho.prevTicks);

      // (we are checking every 10 milliseconds) speed in centimeter per second (cm/s)
      int m1_act_speed = (m1_ticks_diff * 1000)  / (TICKS_PER_10_CM-(16 * m_1.speed)/100 );
      
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
      analogWrite(m_1.pwm,m1_output);
      analogWrite(m_2.pwm,m2_output);

      Serial.println(m1_output);
      Serial.println(m2_output);

  }
}
