/*----------------------------------------------------------------------------------------------------
   Function:                 path planner car master Arduino_program

   I2C ADDRESS:    MASTER

   Five push_buttons(recetor:analogRead(A0)):  buttonLeft(HIGH/LOW)
                                               buttonRight(HIGH/LOW)
                                               buttonDown(HIGH/LOW)
                                               buttonUp(HIGH/LOW)
                                               buttonSure(HIGH/LOW)

    Control:     1.slave_board color calibration
                 2.motor control
                 3.path planning prgram designed by Processing

    remote front_sensor_board :  1 - color sensor white calibration (auto read )
                       2 - color sensor black calibration (auto read )
                       3 - color sensor color_1 calibration (auto read )
                                 4 - color sensor color_2 calibration (auto read )
                                 5 - color sensor color_3 calibration (auto read )
                                 6 - color sensor color_4 calibration (auto read )
                                 7 - color sensor color_5 calibration (auto read )
                                 8 - color sensor color_6 calibration (auto read )
                                 9 - color sensor color_7 calibration (auto read )
                       10 - line  sensor W/B calibration (auto read )
                       11 - save the calibration data to EEPROM 

    Desc:                    master_board for reiceving data from slave and the main brain of the car
                             The line  following program is using PID controler technology

                             color_sensor:  distance_Serial[0];      1:White
                                                                                      2:Black
                                                                                      3:color_1
                                                                                      4:color_2
                                                                                      5:color_3
                                                                                      6:color_4
                                                                                      7:color_5
                                                                                      8:color_6
                                                                                      9:color_7

-----------------------------------------------------------------------------------------------------*/
#define debug HIGH

byte mock_counter = 0;
bool mock_interrup;
bool mock_state, mock_last_state;
String read_processing_String;   //if all the program is okey change it or delete it!!!!
//String String_state, last_String_state;
byte mock_loop_counter = 0;
bool mock_loop_controler = LOW;

#include <Wire.h>
#include <EEPROM.h>

#define front_sensor_board 0x01
#define LCD_address 0x27
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(LCD_address, 20, 4);

#define E1     5
#define M1     4
#define E2     6
#define M2     7
#define botton_range      10                       //the accepted botton_range of error
byte motor_speed = 100;                           //seting the maximum motor_speed

float kp = 20;                                    //for the small rough
float kd = 2;                                     //for the large rough
bool stoper = LOW;                                //Main control motor ON/OFF      (LOW == OFF / HIGH == ON)
bool remote_stoper = HIGH;                         //remote cotrol motor ON/OFF     (LOW == OFF /  HIGH == ON --- when main_stoper is ON)
byte lcd_delay_time = 50;                         //the delay time of the screen to update
bool state, last_state;                           //inside_program:control the main_stoper on the push_botton
bool state2, last_state2;                         //inside_program:control the remote_stoper by the remote control
String lcd_analyse_cross_interrupt;               //inside_program:the cross or line  for lcd printing
bool inside_turning_back_interrup = LOW;          //inside_program:controling the turning back action interrup
bool back_counter_interrup;
int back_inside_counter = -1;
bool inside_cross_turning_interrup = LOW;
int output;                                       //for controling the speed of the motor to turning
#define SampleTime  1                            //for the "control_motors" to caculate the kd value
byte wire_read_front_board[1];                    //use a value to contain the i2c receive data
unsigned long timer1, timer2, timer3, timer4, timer5, timer6, timer7, timer8;             //the timer for replacing "delay()"
bool bottonLeft, bottonRight, bottonDown, bottonUp, bottonSure;
int input = 0;                                    //to analyse the BIN_data
int last_input;                                   //to caculate the kd in "control_motors"
String print_light_data;                          //the data of position of motor which only for LCD printinig
bool analyse_cross_angle_front_sensors;                 //to analyse whether it is a cross (the first step)
int delay_counter = -1;
String analyse_cross_interrupt = "line ";
bool LL_sensor = LOW;
bool RR_sensor = LOW;
#define delay_time_analyse_cross_angle   10              //unit is micro second
char turning_position = 'R';                      //"L" is Left; "R" is Right; "F" is Front; "B" is Returning; "N" is Nothing


long delay_counter_L_inside = -1;
long delay_counter_R_inside = -1;

int page = 0;
#define maximum_page        5                    //actually page (maximum_page + 1) If you add page please add the switch case in "page control"
byte receiver[2];                                 //the data from the remote:[0] is the remote_stoper;[1] is the cross_turning_position = turning_position;

#define RL_digit_delaytime 10                     //20the delay_time of "cross_sensor" when the "analyse_cross_angle_front_sensors" == HIGH
int nothing_stoper_counter = -1;              //the counter_time when the light_Sensor nothing for stoping

int joinstick_x;                                 //join_stick X data
int joinstick_y;                                 //join_stick Y data
bool joinstick_click = LOW;                      //join_stick click data
int turning_special_case_counter = -1;
//bool turning_special_case_interrupt = LOW;
bool turning_special_case_counter_interrupt = LOW;
int nothing_counter = -1;


void setup() {
  delay_counter_L_inside = -1;
  delay_counter_R_inside = -1;

  pinMode(A0, INPUT);                             //joinstick X INPUT
  pinMode(A1, INPUT);                             //joinstick Y INPUT
  pinMode(12, INPUT_PULLUP);                             //joinstick click_INPUT

  pinMode(13, OUTPUT);                            //the LED display
  pinMode(M1, OUTPUT);                            //left motor
  pinMode(M2, OUTPUT);                          //right motor
  pinMode(E1, OUTPUT);                          //left motor PWM data
  pinMode(E2, OUTPUT);                          //right motor PWM data
  Wire.begin();
  lcd.init();
  lcd.backlight(); 
  EEPROM.get(0, kp);
  EEPROM.get(5, kd);
  EEPROM.get(10, motor_speed);

  Serial.begin(115200);

} 

void loop() {

  control_joinstick();
  if(page == 11 ){if(bottonLeft == HIGH || bottonRight == HIGH) lcd_delay_time = 1000;}
  else lcd_delay_time = 50;

  if(millis() - timer2 >= lcd_delay_time){
    pages_control();
    timer2 = millis();
  }

  analyse_cross();
  turning_cross();
  turning_back();
  control_remote_stoper();
 
  receiver_controler();                          //the conversion of parameters from remote_control

  mock_map();                                    //if all the program is okey change it or delete it!!!!

  if(wire_read_front_board[0] == B00000000){
    if(millis() - timer8 >= 1000){
      nothing_counter ++;
      timer8 = millis();
    }
    if(nothing_counter >= 2){
      stoper = LOW;                             //if all the program is okey change it or delete it!!!!
     // remote_stoper = LOW;                    //if all the program is okey change it or delete it!!!!
      lcd.clear();
      mock_loop_controler = LOW;
      nothing_counter = -1;
    }
  }
  else nothing_counter = -1;



    if(stoper == LOW || remote_stoper == LOW){
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 0 );
      analogWrite(E2, 0 );

      mock_loop_controler = LOW;
    }

  if(millis() - timer1 >= SampleTime){
if(analyse_cross_interrupt == "line "){
  control_motors();
}
    Wire.requestFrom(front_sensor_board, 1);
  if(Wire.available()){
    wire_read_front_board[0] = Wire.read();
      analyse_motor_position();

    timer1 = millis();
}



  }
  if(debug == HIGH){
   /* Serial.print("BIN_DATA: ");
    Serial.print(wire_read_front_board[0], BIN);
   // Serial.print(" color_data: ");
    //Serial.print(wire_read_front_board[1]);
    Serial.print(" analyse_cross_angle_front_sensors: ");
    Serial.print(analyse_cross_angle_front_sensors);
    //Serial.print(" analyse_cross_interrupt: ");
    //Serial.print(analyse_cross_interrupt);
    Serial.print("       LL_sensor: ");
    Serial.print(LL_sensor);
    Serial.print("  RR_sensor: ");
    Serial.print(RR_sensor);
    Serial.print("       g_array_back: ");
    Serial.print(g_array_back[0]);
    Serial.print(" , ");
    Serial.print(g_array_back[1]);
    Serial.print(" , ");
    Serial.print(g_array_back[2]);
    Serial.print("         ");
    Serial.print(print_light_data);
    Serial.print("     ");
   Serial.println(analyse_cross_interrupt);*/

   /*Serial.print("mock_interrup:  ");
   Serial.print(mock_interrup);
   Serial.print("    mock_loop_counter:  ");
   Serial.print(mock_loop_counter);
   Serial.print("    mock_counter:  ");
   Serial.println(mock_counter);*/

 //  Serial.print("BIN_DATA: ");
  //  Serial.print(wire_read_front_board[0], BIN);
   // Serial.print(" color_data: ");
    //Serial.print(wire_read_front_board[1]);
   // Serial.print(" analyse_cross_angle_front_sensors: ");
   // Serial.print(analyse_cross_angle_front_sensors);
    //Serial.print(" analyse_cross_interrupt: ");
    //Serial.print(analyse_cross_interrupt);
    Serial.print("       analyse_cross_angle_front_sensors: ");
    Serial.print(analyse_cross_angle_front_sensors);
    Serial.print("       input: ");
    Serial.print(input);
    Serial.print("       LL_sensor: ");
    Serial.print(LL_sensor);
    Serial.print("  RR_sensor: ");
    Serial.println(RR_sensor);
    //Serial.print("       turning_special_case_counter: ");
   // Serial.println(turning_special_case_counter);
   /* Serial.print(" , ");
    Serial.print(g_array_back[1]);
    Serial.print(" , ");
    Serial.print(g_array_back[2]);
    Serial.print("         ");
    Serial.print(print_light_data);*/
 //   Serial.print("     ");
   //Serial.println(analyse_cross_interrupt);
 }

}

void receiver_controler(){
  receiver[0] = 1;    remote_stoper = receiver[0];                                             //if the remote program is okay please change it or delete it!!
  //turning_position = receiver[1];                               //if the remote program is okay please change it or delete it!!
}

void analyse_motor_position(){
  switch (wire_read_front_board[0]) {
    case B10000000:
      input = -6;
      break;
    case B11000000:
      input = -5;
      break;
    case B01000000:
      input = -4;
     // analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01100000:
      input = -3;
      break;
    case B00100000:
      input = -2;
      break;
    case B00110000:
      input = -1;
      break;
    case B00010000:
      input = 0;
      break;
    case B00011000:
      input = 1;
      break;
    case B00001000:
      input = 2;
      break;
    case B00001100:
      input = 3;
      break;
    case B00000100:
      input = 4;
     // analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00000110:
      input = 5;
      break;
    case B00000010:
      input = 6;
      break;
    case B01111100:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00111000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00111100:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01111000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B11111000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00111110:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00101000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B11111110:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01111110:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B11111100:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B11110000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01110000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00011110:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00011100:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01010000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00010100:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B00100100:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01001000:
      analyse_cross_angle_front_sensors = HIGH;
      break;
    case B01000100:
      analyse_cross_angle_front_sensors = HIGH;
      break;

  }
}

void control_motors(){
    if (motor_speed > 255) {
      motor_speed = 255;
    }
    else {
      if (motor_speed < 0) {
        motor_speed = 0;
      }
    }
    if (kp < 0) {
      kp = 255;
    }
    else {
      if (kp > 255) {
        kp = 0;
      }
    }
  
    if (kd < 0) {
      kd = 255;
    }
    else {
      if (kd > 255) {
        kd = 0;
      }
    }

    output = -kp * input - kd * (input - last_input) / (SampleTime * 1e-3);
    output = constrain(output , -2 * motor_speed, 2 * motor_speed);
    last_input = input;

    if (stoper == HIGH && remote_stoper == HIGH) {
    if (output > -motor_speed && output <0)
    {
     digitalWrite(M1,HIGH);   
      digitalWrite(M2, HIGH);       
      analogWrite(E1, motor_speed );   
      analogWrite(E2, motor_speed - abs(output));  
    }
    else if(output >= 0 && output < motor_speed)
    {
      digitalWrite(M1,HIGH);   
      digitalWrite(M2, HIGH);       
      analogWrite(E1, motor_speed - abs(output));   
      analogWrite(E2, motor_speed );    
    }
    else if(output >= motor_speed && output <= 2*motor_speed)
    {
      digitalWrite(M1,LOW);   
      digitalWrite(M2, HIGH);       
      analogWrite(E1, motor_speed);   
      analogWrite(E2, abs(output) - motor_speed); 
    } 
    else if(output >= -2*motor_speed && output <= -motor_speed)
    {
      digitalWrite(M1,HIGH);   
      digitalWrite(M2, LOW);       
      analogWrite(E1, abs(output) - motor_speed);   
      analogWrite(E2, motor_speed);
    }
 }
    else{
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(E1, 0 );
      analogWrite(E2, 0 );
    }
}

void turning_back(){
  if(turning_position == 'B'){
    if(stoper == HIGH && remote_stoper == HIGH){
      if(back_counter_interrup == LOW){
    digitalWrite(M1,HIGH);   
    digitalWrite(M2,LOW);       
    analogWrite(E1, motor_speed - 10);   
    analogWrite(E2, motor_speed - 10); 
    input = 6;
  
  if(wire_read_front_board[0] == B00000000) {
    if(millis() - timer6 >= 1) {
      back_inside_counter ++;
      timer6 = millis();
    }
    if(back_inside_counter > 10){
    back_counter_interrup = HIGH;
    back_inside_counter = -1;
  }
}
else back_inside_counter = -1;
}

  if(back_counter_interrup == HIGH){
    if(bitRead(wire_read_front_board[0], 3) == HIGH || bitRead(wire_read_front_board[0], 2) == HIGH || bitRead(wire_read_front_board[0], 4) == HIGH || bitRead(wire_read_front_board[0], 1) == HIGH){
    turning_position = 'N';
    analyse_cross_interrupt = "line ";
    inside_turning_back_interrup = LOW;
    back_counter_interrup = LOW;
  }
}
}
}
}

void analyse_cross(){  

 // if(analyse_cross_interrupt == "cross" || analyse_cross_interrupt == "LLLL" || analyse_cross_interrupt == "RRRR" || analyse_cross_angle_front_sensors == HIGH){

      if(millis() - timer5 >= 1) { delay_counter_L_inside ++; delay_counter_R_inside ++; timer5 = millis();}
  if(bitRead(wire_read_front_board[0], 1) == HIGH) {LL_sensor = HIGH; delay_counter_L_inside = -1;}
  else{
    if(delay_counter_L_inside > RL_digit_delaytime){
      LL_sensor = LOW;
      delay_counter_L_inside = -1;
    }
  }
  if(bitRead(wire_read_front_board[0], 7) == HIGH) {RR_sensor = HIGH; delay_counter_R_inside = -1;}
  else{
    if(delay_counter_R_inside > RL_digit_delaytime){
      RR_sensor = LOW;
      delay_counter_R_inside = -1;
    }
  }
 // }
 /* else{
      delay_counter_L_inside = -1;
      delay_counter_R_inside = -1;
      if(bitRead(wire_read_front_board[0], 1) == HIGH) LL_sensor = HIGH;
      else LL_sensor = LOW;
      if(bitRead(wire_read_front_board[0], 7) == HIGH) RR_sensor = HIGH;
      else RR_sensor = LOW;
  }*/


  if(analyse_cross_angle_front_sensors == HIGH){
  if(millis() - timer4 >= 1){
    delay_counter ++;
    timer4 = millis();
  }
  
      if(delay_counter <= delay_time_analyse_cross_angle){
      if(LL_sensor == HIGH && RR_sensor == HIGH){
    analyse_cross_interrupt = "cross";
    analyse_cross_angle_front_sensors = LOW;
    delay_counter = -1;
    turning_special_case_counter = -1;
    turning_special_case_counter_interrupt = LOW;
  }
    }
    else{
      if(LL_sensor == HIGH || RR_sensor == HIGH) turning_special_case_counter_interrupt = HIGH;

   if(turning_special_case_counter_interrupt == HIGH){
         if(millis() - timer7 >= 1){
        turning_special_case_counter ++;
        timer7 = millis();
      }
    }

        if(bitRead(wire_read_front_board[0], 3) == HIGH || bitRead(wire_read_front_board[0], 4) == HIGH || bitRead(wire_read_front_board[0], 5) == HIGH){
          if(turning_special_case_counter >= 10){                      //Seting the special case delay time
          analyse_cross_interrupt = "cross";
          analyse_cross_angle_front_sensors = LOW;
          delay_counter = -1;
          turning_special_case_counter = -1;
          turning_special_case_counter_interrupt = LOW;
        }
        }
        else{
        if(LL_sensor == HIGH && RR_sensor == LOW){
        analyse_cross_interrupt = "LLLL";
        analyse_cross_angle_front_sensors = LOW;
        delay_counter = -1;
        turning_special_case_counter = -1;
        turning_special_case_counter_interrupt = LOW;
        }
        else{
        if(LL_sensor == LOW && RR_sensor == HIGH){
        analyse_cross_interrupt = "RRRR";
        analyse_cross_angle_front_sensors = LOW;
        delay_counter = -1;
        turning_special_case_counter = -1;
        turning_special_case_counter_interrupt = LOW;
        }
      }
      }
}

        /*else  {
        if(analyse_cross_interrupt == "line "){
        analyse_cross_interrupt = "line ";
        analyse_cross_angle_front_sensors = LOW;
        delay_counter = -1;
       // turning_special_case_counter = -1;
        turning_special_case_counter = -1;
        turning_special_case_counter_interrupt = LOW;
        }
        }*/
}
}

void turning_cross(){
  if(analyse_cross_interrupt == "LLLL"){
    if(LL_sensor == LOW && RR_sensor == LOW){
      inside_cross_turning_interrup = HIGH;
  }
  if(inside_cross_turning_interrup == HIGH){
    if(stoper == HIGH && remote_stoper == HIGH){
    turning_cross_Left();
  }
  if(bitRead(wire_read_front_board[0], 2) == HIGH){
    inside_cross_turning_interrup = LOW;
    analyse_cross_interrupt = "line ";
  }
}
  }

    if(analyse_cross_interrupt == "RRRR"){
    if(LL_sensor == LOW && RR_sensor == LOW){
      inside_cross_turning_interrup = HIGH;
  }
  if(inside_cross_turning_interrup == HIGH){
    if(stoper == HIGH && remote_stoper == HIGH){
    turning_cross_Right();
  }
  if(bitRead(wire_read_front_board[0], 6) == HIGH){
    inside_cross_turning_interrup = LOW;
    analyse_cross_interrupt = "line ";
  }
}
  }

  if(analyse_cross_interrupt == "cross"){
    if(LL_sensor == LOW && RR_sensor == LOW){
      inside_cross_turning_interrup = HIGH;
  }
  if(inside_cross_turning_interrup == HIGH){
    if(stoper == HIGH && remote_stoper == HIGH){
    if(turning_position == 'L') turning_cross_Left();
    if(turning_position == 'R') turning_cross_Right();
    if(turning_position == 'F' || turning_position == 'N') turning_cross_Front();
  }
  

  if(turning_position == 'L') {
    if(bitRead(wire_read_front_board[0], 2) == HIGH){
    turning_position = 'N'; 
    analyse_cross_interrupt = "line ";
    inside_cross_turning_interrup = LOW;
  }
    
  }

  if(turning_position == 'R') {
    if(bitRead(wire_read_front_board[0], 6) == HIGH) {
    turning_position = 'N'; 
    analyse_cross_interrupt = "line ";
    inside_cross_turning_interrup = LOW;
   }
   }

  if(turning_position == 'F') {turning_position = 'N'; analyse_cross_interrupt = "line "; inside_cross_turning_interrup = LOW;}
  if(turning_position == 'N') {turning_position = 'N'; analyse_cross_interrupt = "line "; inside_cross_turning_interrup = LOW;}
  }

}

  if(stoper == LOW && remote_stoper == LOW){
    turning_position = 'N'; 
    analyse_cross_interrupt = "line ";
    inside_cross_turning_interrup = LOW;
  }

/*  if(turning_position == 'N'){
    if(input <= -4) input = -4;
    if(input >= 4) input = 4;
  } */

}

void turning_cross_Left(){
  digitalWrite(M1,HIGH);   
  digitalWrite(M2, LOW);       
  analogWrite(E1, motor_speed);   
  analogWrite(E2, motor_speed);  
  input = -6;
}
void turning_cross_Right(){
  digitalWrite(M1,LOW);   
  digitalWrite(M2, HIGH);       
  analogWrite(E1, motor_speed);   
  analogWrite(E2, motor_speed);  
  input = 6;
}
void turning_cross_Front(){
  digitalWrite(M1,HIGH);   
  digitalWrite(M2, HIGH);       
  analogWrite(E1, motor_speed);   
  analogWrite(E2, motor_speed); 
  input = 0;
}

    

void control_joinstick(){
  joinstick_x = analogRead(A0);
  joinstick_y = analogRead(A1);
  joinstick_click = digitalRead(12);

  if(joinstick_x <= 10 && bottonUp == LOW && bottonDown == LOW) bottonLeft = HIGH;  
  else bottonLeft = LOW;
  if(joinstick_x >= 1000 && bottonUp == LOW && bottonDown == LOW) bottonRight = HIGH;
  else bottonRight = LOW;
  if(joinstick_y <= 10 && bottonLeft == LOW && bottonRight == LOW) bottonUp = HIGH;  
  else bottonUp = LOW;
  if(joinstick_y >= 1000 && bottonLeft == LOW && bottonRight == LOW) bottonDown = HIGH;
  else bottonDown = LOW;
  if(joinstick_click == LOW && bottonLeft == LOW && bottonRight == LOW && bottonUp == LOW && bottonDown == LOW) bottonSure = HIGH; 
  else bottonSure = LOW; 
}

void pages_control(){
 // if(stoper == LOW && remote_stoper == LOW){                                     //if all the program is okey change it or delete it!!!!
  if(stoper == LOW){
  if(bottonLeft == HIGH) page --;
  if(bottonRight == HIGH) page ++;
}
  if(page > maximum_page) page = 0;
  if(page < 0 ) page = maximum_page;

  if(page == 0) control_main_stoper();                                         //control the motor ON/OFF

  if(bottonSure == HIGH){
    if(page == 1) {remote_calibration_WB_line (); lcd.clear();}                   //calibration_light_sensor
    if(page == 2) {remote_slave_save_data(); lcd.clear();}                      //save all data to EEPROM including slave and master
   }

   if(bottonUp == HIGH){
    if(page == 3) kp += 0.1;
    if(page == 4) kd += 0.1;
    if(page == 5) motor_speed ++;
   }

   if(bottonDown == HIGH){
    if(page == 3) kp -= 0.1;
    if(page == 4) kd -= 0.1;
    if(page == 5) motor_speed --;
   }

   if(bottonLeft == HIGH) lcd.clear();
   if(bottonRight == HIGH) lcd.clear();

   switch(page){
    case 1:
      lcd.setCursor(0,0);
      lcd.print("  Calibration_W/B  1");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("     on the W/B     ");
    break;
    case 2:
      lcd.setCursor(0,0);
      lcd.print("  Saving the data  2");
      lcd.setCursor(0,1);
      lcd.print("                    ");
      lcd.setCursor(0,2);
      lcd.print("    Press to save   ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
    break;
    case 0:
    if(stoper == LOW || remote_stoper == LOW){
       switch (input) {
    case -6:
      print_light_data = "0000001";
      break;
    case -5:
      print_light_data = "0000011";
      break;
    case -4:
      print_light_data = "0000010";
      break;
    case -3:
      print_light_data = "0000110";
      break;
    case -2:
      print_light_data = "0000100";
      break;
    case -1:
      print_light_data = "0001100";
      break;
    case 0:
      print_light_data = "0001000";
      break;
    case 1:
      print_light_data = "0011000";
      break;
    case 2:
      print_light_data = "0010000";
      break;
    case 3:
      print_light_data = "0110000";
      break;
    case 4:
      print_light_data = "0100000";
      break;
    case 5:
      print_light_data = "1100000";
      break;
    case 6:
      print_light_data = "1000000";
      break;
  }
      lcd.setCursor(3,0);
      lcd.print("Motor_position  0");
      lcd.setCursor(6,1);
      lcd.print(print_light_data);
      lcd.setCursor(0,2);
      lcd.print(analyse_cross_interrupt);
      lcd.setCursor(0,3);
      lcd.print(read_processing_String);
    }
    else{
      if(analyse_cross_interrupt == "cross") lcd_analyse_cross_interrupt = "C ";
      if(analyse_cross_interrupt == "line ") lcd_analyse_cross_interrupt = "L ";
      if(analyse_cross_interrupt == "LLLL") lcd_analyse_cross_interrupt = "LL";
      if(analyse_cross_interrupt == "RRRR") lcd_analyse_cross_interrupt = "RR";
      lcd.setCursor(0,0);
      lcd.print(lcd_analyse_cross_interrupt);
      lcd.setCursor(0,1);
      lcd.print(turning_position);
      lcd.setCursor(2,1);
      lcd.print(mock_counter);
      lcd.setCursor(0,2);
      lcd.print(input);
    }
    break;
    case 3:
      lcd.setCursor(0,0);
      lcd.print("    seting kp      3");
      lcd.setCursor(0,1);
      lcd.print("       kp = ");
      lcd.print(kp);
    break;
    case 4:
      lcd.setCursor(0,0);
      lcd.print("    seting kd      4");
      lcd.setCursor(0,1);
      lcd.print("       kd = ");
      lcd.print(kd);
    break;
    case 5:
      lcd.setCursor(0,0);
      lcd.print("   seting speed    5");
      lcd.setCursor(0,1);
      lcd.print("      speed = ");
      lcd.print(motor_speed);
    break;

}
}

//--------------------------------------------------------------------------remote the front_sensor_board BEGIN


void remote_calibration_WB_line (){
    Wire.beginTransmission(front_sensor_board);
      Wire.write(10);                                             //light_sensor_W/B
      Wire.endTransmission();    
}

void remote_slave_save_data(){
    Wire.beginTransmission(front_sensor_board);
      Wire.write(11);                                             //save all data
      Wire.endTransmission();    

      EEPROM.put(0, kp);
      EEPROM.put(5, kd);
      EEPROM.put(10, motor_speed);
      
}


//--------------------------------------------------------------------------remote the front_sensor_board END

void control_main_stoper(){
  state = bottonSure;
  if(state != last_state && state == HIGH){
  lcd.clear();
  if(stoper == HIGH) stoper = LOW; 
  else if(stoper == LOW) {
    stoper = HIGH; 
    analyse_cross_interrupt = "line ";
    mock_counter = 0;
    mock_loop_counter = 0;
    mock_loop_controler = HIGH;
    mock_interrup = LOW;
    mock_state = LOW; 
    mock_last_state = LOW;
    delay_counter_L_inside = -1;
    delay_counter_R_inside = -1;
    delay_counter = -1;
    analyse_cross_interrupt = "line ";
    analyse_cross_angle_front_sensors = LOW;
    delay_counter = -1;
    LL_sensor = LOW;
    RR_sensor = LOW;
    inside_cross_turning_interrup = LOW;
  }
}
  last_state = state;
}

void control_remote_stoper(){
  if(receiver[0] >= 1) receiver[0] = 1;
  else receiver[0] = 0;

  if(stoper == LOW) remote_stoper = LOW;
  else {
  state2 = receiver[0];
  if(last_state2 != state2 && state2 == HIGH){
  lcd.clear();
  if(remote_stoper == HIGH) remote_stoper = LOW; 
  else if(remote_stoper == LOW) {
    remote_stoper = HIGH; 
    analyse_cross_interrupt = "line ";
    mock_counter = 0;
    mock_loop_counter = 0;
    mock_loop_controler = HIGH;
    mock_interrup = LOW;
    mock_state = LOW; 
    mock_last_state = LOW;
    delay_counter_L_inside = -1;
    delay_counter_R_inside = -1;
    delay_counter = -1;
    analyse_cross_interrupt = "line ";
    analyse_cross_angle_front_sensors = LOW;
    delay_counter = -1;
    LL_sensor = LOW;
    RR_sensor = LOW;
    inside_cross_turning_interrup = LOW;
  }
  }
    last_state2 = state2;
  }
}

void mock_map(){
  if (stoper == HIGH && remote_stoper == HIGH) {
  if(turning_position == 'N'){
    if(millis() - timer3 >= 1000){
      nothing_stoper_counter ++;
      timer3 = millis();
    }
    if(nothing_stoper_counter >= 2) {
      stoper = LOW;                             //if all the program is okey change it or delete it!!!!
     // remote_stoper = LOW;                    //if all the program is okey change it or delete it!!!!
      lcd.clear();
      mock_loop_controler = LOW;
      nothing_stoper_counter = -1;
    }
  }
  else nothing_stoper_counter = -1;
}

/*
  if(turning_position == 'N') mock_interrup = HIGH;
  else mock_interrup = LOW;
  mock_state = mock_interrup;
  if(mock_last_state != mock_state && mock_state == HIGH){
    if(mock_counter == 4) {turning_position = 'F'; mock_counter = 0; stoper = LOW;}
    if(mock_counter == 3) {turning_position = 'B'; mock_counter = 4;}
    if(mock_counter == 2) {turning_position = 'R'; mock_counter = 3;}
    if(mock_counter == 1) {turning_position = 'F'; mock_counter = 2;}
    if(mock_counter == 0) {turning_position = 'L'; mock_counter = 1;}
    }
    mock_last_state = mock_state;
    
    turning_position = 'L';*/
    read_processing_String = "LRRFLFFLRFFLB";       //if all the program is okey change it or delete it!!!!
    char read_processing_char[read_processing_String.length()];
   
    for(byte i = 0; i <= read_processing_String.length() - 1; i ++){
      read_processing_char[i] = read_processing_String.charAt(i);
    }
  

    if(mock_loop_controler == HIGH){
    if(turning_position == 'N') mock_interrup = HIGH;
    else mock_interrup = LOW;
    
    mock_state = mock_interrup;
    if(mock_last_state != mock_state && mock_state == HIGH){

        turning_position = read_processing_char[mock_counter];

        if(mock_counter <= read_processing_String.length() - 1) mock_counter ++;
        else {mock_counter = 0; mock_loop_counter ++;}
      
    }
    mock_last_state = mock_state;
  }

    if(mock_loop_counter == 1) mock_loop_controler = LOW;
    if(mock_loop_controler == LOW) {turning_position = 'N'; mock_interrup = LOW; mock_state = LOW; mock_last_state = LOW;}
    //turning_position = 'L';
  }
  





