//The program starts in calibration mode.
//Send following characters/numbers via the serial monitor to change the mode
//
//r = print receiver signals.
//a = print quadcopter angles.
//1 = check rotation / vibrations for motor 1 (right front CCW).
//2 = check rotation / vibrations for motor 2 (right rear CW).
//3 = check rotation / vibrations for motor 3 (left rear CCW).
//4 = check rotation / vibrations for motor 4 (left front CW).
//5 = check vibrations for all motors together.

#include <Wire.h>                                    
#include <EEPROM.h>                                 
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte nrf_address[6] = {'D', 'R', 'O', 'N', 'E'};

struct __attribute__((packed)) Data {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
};
Data receivedData;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36], start, data;
boolean new_function_request,first_angle;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int esc_1, esc_2, esc_3, esc_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int receiver_input[5];
int loop_counter, gyro_address, vibration_counter;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int acc_axis[4], gyro_axis[4];
double gyro_pitch, gyro_roll, gyro_yaw;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
int cal_int;
double gyro_axis_cal[4];

void updateReceiverFromRadio() {
  static unsigned long lastSignalTime = 0;

  if (radio.available()) {
    radio.read(&receivedData, sizeof(Data));
    lastSignalTime = millis();

    receiver_input_channel_1 = map(receivedData.roll, 0, 1023, 1000, 2000);
    receiver_input_channel_2 = map(receivedData.pitch, 0, 1023, 1000, 2000);
    receiver_input_channel_3 = map(receivedData.throttle, 0, 1023, 1000, 2000);
    receiver_input_channel_4 = map(receivedData.yaw, 0, 1023, 1000, 2000);
  }
}

void setup(){
  Serial.begin(115200);                                                                 
  Wire.begin();                                                                         
  TWBR = 12;                                                                            

  DDRD |= B11110000;                                                                    
  pinMode(2, OUTPUT);  

  radio.begin();
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(0, nrf_address);
  radio.startListening();
  
  for(data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);               //Reading EEPROM for faster data access

  gyro_address = eeprom_data[32];                                                     

  set_gyro_registers();                                                                

  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B'){
    delay(500);                                                                         
    digitalWrite(2, !digitalRead(2));                                                
  }
  wait_for_receiver();                                                                 
  zero_timer = micros();                                                                

  while(Serial.available())data = Serial.read();                                       
  data = 0;                                                                            
}

//Main program loop
void loop(){
  while(zero_timer + 4000 > micros());                                                  
  zero_timer = micros();                                                              

  if(Serial.available() > 0){
    data = Serial.read();                                                            
    delay(100);                                                                        
    while(Serial.available() > 0)loop_counter = Serial.read();                        
    new_function_request = true;                                                      
    loop_counter = 0;                                                                  
    cal_int = 0;                                                                      
    start = 0;                                                                        
    first_angle = false;                                                              
    if(data == 'r')Serial.println("Reading receiver signals.");
    if(data == 'a')Serial.println("Print the quadcopter angles.");
    if(data == 'a')Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if(data == '1')Serial.println("Test motor 1 (right front CCW.)");
    if(data == '2')Serial.println("Test motor 2 (right rear CW.)");
    if(data == '3')Serial.println("Test motor 3 (left rear CCW.)");
    if(data == '4')Serial.println("Test motor 4 (left front CW.)");
    if(data == '5')Serial.println("Test all motors together");

    for(vibration_counter = 0; vibration_counter < 625; vibration_counter++){           
      delay(3);                                                                         
      esc_1 = 1000;                                                                    
      esc_2 = 1000;                                                                    
      esc_3 = 1000;                                                                     
      esc_4 = 1000;                                                                    
      esc_pulse_output();                                                               
    }
    vibration_counter = 0;                                                             
  }

  updateReceiverFromRadio();
  if(receiver_input_channel_3 < 1025)new_function_request = false;                      


  ////////////////////////////////////////////////////////////////////////////////////////////
  //Run the ESC calibration program to start with.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 0 && new_function_request == false){                                      
    updateReceiverFromRadio();
    esc_1 = receiver_input_channel_3;                                                   
    esc_2 = receiver_input_channel_3;                                                  
    esc_3 = receiver_input_channel_3;                                                
    esc_4 = receiver_input_channel_3;                                                  
    esc_pulse_output();                                                              
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'r' print the receiver signals.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'r'){
    loop_counter ++;                                                                    
   
    updateReceiverFromRadio();

    if(loop_counter == 125){                                                            
      print_signals();                                                                 
      loop_counter = 0;                                                                
    }

    //For starting the motors: throttle low and yaw left (step 1).
    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

    esc_1 = 1000;                                                                      
    esc_2 = 1000;                                                                       
    esc_3 = 1000;                                                                       
    esc_4 = 1000;                                                                     
    esc_pulse_output();                                                               
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a '1, 2, 3, 4 or 5 test the motors.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){          
    loop_counter ++;                                                                   
    if(new_function_request == true && loop_counter == 250){                          
      Serial.print("Set throttle to 1000 (low). It's now set to: ");                  
      Serial.println(receiver_input_channel_3);                                      
      loop_counter = 0;                                                              
    }
    if(new_function_request == false){                                                
      updateReceiverFromRadio();
      if(data == '1' || data == '5')esc_1 = receiver_input_channel_3;                  
      else esc_1 = 1000;                                                               
      if(data == '2' || data == '5')esc_2 = receiver_input_channel_3;                
      else esc_2 = 1000;                                                              
      if(data == '3' || data == '5')esc_3 = receiver_input_channel_3;                  
      else esc_3 = 1000;                                                               
      if(data == '4' || data == '5')esc_4 = receiver_input_channel_3;                  
      else esc_4 = 1000;                                                              

      esc_pulse_output();                                                             

      if(eeprom_data[31] == 1){                                                         
        Wire.beginTransmission(gyro_address);                                         
        Wire.write(0x3B);                                                               
        Wire.endTransmission();                                                        
        Wire.requestFrom(gyro_address,6);                                               
        while(Wire.available() < 6);                                                   
        acc_x = Wire.read()<<8|Wire.read();                                            
        acc_y = Wire.read()<<8|Wire.read();                                           
        acc_z = Wire.read()<<8|Wire.read();                                             

        acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));          

        acc_av_vector = acc_total_vector[0];                                           

        for(start = 16; start > 0; start--){                                          
          acc_total_vector[start] = acc_total_vector[start - 1];                     
          acc_av_vector += acc_total_vector[start];                                   
        }

        acc_av_vector /= 17;                                                           

        if(vibration_counter < 20){                                                    
          vibration_counter ++;                                                      
          vibration_total_result += abs(acc_total_vector[0] - acc_av_vector);       
        }
        else{
          vibration_counter = 0;                                                       
          Serial.println(vibration_total_result/50);                                  
          vibration_total_result = 0;                                                 
        }
      }
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'a' display the quadcopter angles.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'a'){

    if(cal_int != 2000){
      Serial.print("Calibrating the gyro");
      for (cal_int = 0; cal_int < 2000 ; cal_int ++){                                
        if(cal_int % 125 == 0){
          digitalWrite(2, !digitalRead(2));  
          Serial.print(".");
        }
        gyro_signalen();                                                               
        gyro_axis_cal[1] += gyro_axis[1];                                              
        gyro_axis_cal[2] += gyro_axis[2];                                             
        gyro_axis_cal[3] += gyro_axis[3];                                             
        PORTD |= B11110000;                                                            
        delayMicroseconds(1000);                                                       
        PORTD &= B00001111;                                                            
        delay(3);                                                                      
      }
      Serial.println(".");
      gyro_axis_cal[1] /= 2000;                                                         
      gyro_axis_cal[2] /= 2000;                                                         
      gyro_axis_cal[3] /= 2000;                                                       
    }
    else{
      PORTD |= B11110000;                                                              
      delayMicroseconds(1000);                                                          
      PORTD &= B00001111;                                                               

      gyro_signalen();

      angle_pitch += gyro_pitch * 0.0000611;                                          
      angle_roll += gyro_roll * 0.0000611;                                           

      angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                         
      angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                        

      acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));         

      angle_pitch_acc = asin((float)acc_y/acc_total_vector[0])* 57.296;               
      angle_roll_acc = asin((float)acc_x/acc_total_vector[0])* -57.296;             
      
      if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 
        angle_roll = angle_roll_acc;                                                 
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;               
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                  
      }

      if(loop_counter == 0)Serial.print("Pitch: ");
      if(loop_counter == 1)Serial.print(angle_pitch ,0);
      if(loop_counter == 2)Serial.print(" Roll: ");
      if(loop_counter == 3)Serial.print(angle_roll ,0);
      if(loop_counter == 4)Serial.print(" Yaw: ");
      if(loop_counter == 5)Serial.println(gyro_yaw / 65.5 ,0);

      loop_counter ++;
      if(loop_counter == 60)loop_counter = 0;      
    }
  }
}

void wait_for_receiver(){
  byte zero = 0;
  unsigned long timer = millis() + 10000;
  Serial.println("Waiting for receiver...");
  while(timer > millis() && zero < 15){
    updateReceiverFromRadio();
    if(receiver_input_channel_1 < 2100 && receiver_input_channel_1 > 900)zero |= 0b00000001;
    if(receiver_input_channel_2 < 2100 && receiver_input_channel_2 > 900)zero |= 0b00000010;
    if(receiver_input_channel_3 < 2100 && receiver_input_channel_3 > 900)zero |= 0b00000100;
    if(receiver_input_channel_4 < 2100 && receiver_input_channel_4 > 900)zero |= 0b00001000;
    delay(500);
  }
  if(zero < 15) Serial.println("WARNING: Not all channels valid!");
  else Serial.println("Receiver OK.");
}

void print_signals(){
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Roll:");
  if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Pitch:");
  if(receiver_input_channel_2 - 1480 < 0)Serial.print("^^^");
  else if(receiver_input_channel_2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Throttle:");
  if(receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
  else if(receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if(receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}

void esc_pulse_output(){
  zero_timer = micros();
  PORTD |= B11110000;                                           
  timer_channel_1 = esc_1 + zero_timer;                          
  timer_channel_2 = esc_2 + zero_timer;                          
  timer_channel_3 = esc_3 + zero_timer;                         
  timer_channel_4 = esc_4 + zero_timer;                          

  while(PORTD >= 16){                                          
    esc_loop_timer = micros();                                  
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;    
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;     
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;    
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;   
  }
}

void set_gyro_registers(){
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                      
    Wire.write(0x6B);                                          
    Wire.write(0x00);                                          
    Wire.endTransmission();                                   

    Wire.beginTransmission(gyro_address);                      
    Wire.write(0x1B);                                         
    Wire.write(0x08);                                         
    Wire.endTransmission();                                   

    Wire.beginTransmission(gyro_address);                       
    Wire.write(0x1C);                                          
    Wire.write(0x10);                                         
    Wire.endTransmission();                                   

    Wire.beginTransmission(gyro_address);                      
    Wire.write(0x1B);                                          
    Wire.endTransmission();                                    
    Wire.requestFrom(gyro_address, 1);                        
    while(Wire.available() < 1);                             
    if(Wire.read() != 0x08){                                   
      digitalWrite(2,HIGH);                                   
      while(1)delay(10);                                      
    }

    Wire.beginTransmission(gyro_address);                       
    Wire.write(0x1A);                                          
    Wire.write(0x03);                                      
    Wire.endTransmission();                               
  }  
}

void gyro_signalen(){
  //Reading the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                      
    Wire.write(0x3B);                                        
    Wire.endTransmission();                                   
    Wire.requestFrom(gyro_address,14);                       
    while(Wire.available() < 14);                             
    acc_axis[1] = Wire.read()<<8|Wire.read();                
    acc_axis[2] = Wire.read()<<8|Wire.read();                  
    acc_axis[3] = Wire.read()<<8|Wire.read();                  
    temperature = Wire.read()<<8|Wire.read();                 
    gyro_axis[1] = Wire.read()<<8|Wire.read();                 
    gyro_axis[2] = Wire.read()<<8|Wire.read();               
    gyro_axis[3] = Wire.read()<<8|Wire.read();                
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                         
    gyro_axis[2] -= gyro_axis_cal[2];                           
    gyro_axis[3] -= gyro_axis_cal[3];                         
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];          
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;           
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];       
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;           
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];          
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;            

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];            
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                  
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];              
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                  
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];             
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                
}
