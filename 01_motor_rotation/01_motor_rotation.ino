 
  
  byte power = 127  // set the power. Max is 255
  
  byte pwn  = 8 ; // power control through PWM
  byte dir1 = 9 ; // for contoring the direction of the motor rotation 
  byte dir2 = 10; // for contoring the direction of the motor rotation 
  
  void setup(){
    
    Serial.begin(9600);
    // set GPIO pin modes 
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(pwn , OUTPUT);
      
    // set the direction of the motor rotation 
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  
    // power the motor
    analogWrite(pwn, power);
  }
  
  void loop(){ }
  
