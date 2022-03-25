#include <Servo.h>

int servo_pan_pin = 9;
int servo_tilt_pin = 6;

int servo_pan_start = (80);
int servo_tilt_start = (100);

int servo_pan_current = servo_pan_start;
int servo_tilt_current = servo_tilt_start;

bool green_target_led = false;

Servo servoPan;
Servo servoTilt;


void setup() {   

  // 1. Serial interface setup 
  Serial.begin(9600); // sets the boud rate to 9600 bts
  pinMode(13,OUTPUT); // configures the onboard led as output
  pinMode(4,OUTPUT); // Laser led pin (red)
  pinMode(2,OUTPUT); // Led (green)
  
  // 2. Pan servo setup
  servoPan.attach(servo_pan_pin);
  servoPan.write(servo_pan_start);
  delay(200); 

  // 3. Tilt servo setup
  servoTilt.attach(servo_tilt_pin);
  servoTilt.write(servo_tilt_start);
  delay(200); 
 
}

void loop() {

  bool x = true;
  while(x){
    
    if(Serial.available()){
      
      // ----------------------------------------- //
      // ----- Incoming command validation ------- //
      // ----------------------------------------- //
      String serial_data;
      serial_data = Serial.readString();
      if(serial_data.startsWith("pan_")){
        digitalWrite(13,HIGH);
        serial_data.replace("pan_","");
        servo_pan_current = serial_data.toInt();
        green_target_led = false;
      }else if(serial_data.startsWith("tilt_")){
        digitalWrite(13,HIGH);
        serial_data.replace("tilt_","");
        servo_tilt_current = serial_data.toInt();
        green_target_led = false;
      }else if(serial_data.startsWith("led_")){
        serial_data.replace("led_","");
        if(serial_data == "on"){
          green_target_led = true;
        }else{
          green_target_led = false;
        }
      }

      // ----------------------------------------- //
      // ------------ Command action ------------- //
      // ----------------------------------------- //      
      servoPan.write(servo_pan_current);
      servoTilt.write(servo_tilt_current);
      delay(100);
      
    }
    
    if(green_target_led){
      digitalWrite(2,HIGH);
    }else{
      digitalWrite(2,LOW);
    }
    // Laser Led Always Active
    digitalWrite(4,HIGH);
    
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
