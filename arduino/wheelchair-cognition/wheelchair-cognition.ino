#include <Servo.h>

int servoMPin = 9;
int servoLPin = 6;
//int servoRPin = 5;

int mid_servo_right = 50;
int mid_servo_base = 100;
int mid_servo_left = 200;

//int front_right_servo_forward = 130;
//int front_right_servo_base = 100;

int front_left_servo_forward = 100;
int front_left_servo_base = 200;
Servo servoM;
Servo servoL;
//Servo servoR;

void setup() {   
  Serial.begin(9600); // sets the boud rate to 9600 bts
  pinMode(13,OUTPUT); // configures the onboard led as output
  
  servoM.attach(servoMPin);
  servoM.write(mid_servo_base);
  delay(200); 
  servoL.attach(servoLPin);
  servoL.write(front_left_servo_base);
  delay(200);  
//  servoR.attach(servoRPin);
//  servoR.write(front_right_servo_base);
//  delay(200); 
}

void loop() {

  bool x = true;
  while(x){

    // String s1 = "Hello World";
    // s1.replace('e','a');
    // myString.startsWith(myString2)
    // myString.toInt()
    
    if(Serial.available()){
      switch(Serial.read()){
        case '0': digitalWrite(13,LOW);
                  break;
        case '1': digitalWrite(13,HIGH);
                  break;
        default:
                  break;
      }
      
    }
//      // 1. Up middle right base
//      servoM.write(mid_servo_right);  
//      delay(500); 
//
//      // 2. Forward left servo 
//      servoL.write(front_left_servo_forward);
//      delay(500);      
//      
//      // 3. Up middle left base
//      servoM.write(mid_servo_left);  
//      delay(500); 
//
//      // 5. Base middler servo
//      servoM.write(mid_servo_base);  
//      delay(500); 
//
//      servoL.write(front_left_servo_base);
//      delay(500); 
  }

 

//  
//  servoR.write(front_servo_base);  
//  delay(2000);        
//  servoR.write(front_servo_forward);  
//  delay(2000);
//
//  // 3. Up middle left base
//  servoM.write(mid_servo_base);
//  delay(2000); 
//  servoM.write(mid_servo_left);  
//  delay(2000); 
//  servoM.write(mid_servo_base);
//  delay(2000);  
    
//  servoM.write(mid_servo_base);  
//  delay(1000);        
//  servoM.write(mid_servo_left);  
//  delay(1000); 



         
//  servoL.write(mid);  
//  delay(15);     
//  servoR.write(170);
//  delay(15);  
//
//  angle = 115;
//  servo.write(angle);
//  delay(500);
//
//  angle = 170;
//  servo.write(angle);
//  delay(20);    
//  for(angle = 0; angle < 180; angle++){
//    servo.write(angle);
//    delay(15);
//  }
//
//  for(angle = 180; angle > 0; angle){
//    servo.write(angle);
//    delay(15);
//  }  
}
