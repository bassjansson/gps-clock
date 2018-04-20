void setup() {                
 // Motor_1 controll pin initiate;
 pinMode(4, OUTPUT);     
 pinMode(5, OUTPUT);    
 pinMode(9, OUTPUT); // Speed control
 
 // Motor_2 controll pin initiate;
 pinMode(7, OUTPUT);     
 pinMode(8, OUTPUT);    
 pinMode(10, OUTPUT);  // Speed control
 
 //Enable the Motor Shield output;  
 pinMode(6, OUTPUT); 
 digitalWrite(6, HIGH);  
}
void loop() {
    
   analogWrite(9,250);    // set the motor_1 speed ;
   digitalWrite(4, HIGH);   
   digitalWrite(5, LOW);  // Set the rotation of motor_1
   
//   analogWrite(10,50);    // set the motor_2 speed ;
//   digitalWrite(7, HIGH);  
//   digitalWrite(8, LOW);  // Set the rotation of motor_2
 
  delay(5000);               // wait for a 5 seconds
  // And we change the motor speed and  rotation direction
    analogWrite(9,100);    // set the motor_1 speed to 100 ;
   digitalWrite(4, LOW);   
   digitalWrite(5, HIGH);  // Set the rotation of motor_1
   
//   analogWrite(10,150);    // set the motor_2 speed to 150
//   digitalWrite(7, LOW);  
//   digitalWrite(8, HIGH);  // Set the rotation of motor_2
   delay(5000);               // wait for a 5 seconds
}
