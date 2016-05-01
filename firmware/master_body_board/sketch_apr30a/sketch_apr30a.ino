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
					             10 - line sensor W/B calibration (auto read )
					             11 - save the calibration data to EEPROM 

    Desc:                    master_board for reiceving data from slave and the main brain of the car
                             The line following program is using PID controler technology

-----------------------------------------------------------------------------------------------------*/




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
#define range      10                             //the accepted range of error
byte motor_speed = 100;                           //seting the maximum motor_speed
float kp = 20;                                  //for the small rough
float kd = 2;                                  //for the large rough
bool stoper = LOW;                                //control motor ON/OFF      (LOW == OFF / HIGH == ON)
int output;                                       //for controling the speed of the motor to turning
#define SampleTime  10                            //for the "control_motors" to caculate the kd value
byte wire_read_front_board[2];                    //use a value to contain the i2c receive data
unsigned long timer1, timer2, timer3;             //the timer for replacing "delay()"
int push_botton;
bool bottonLeft, bottonRight, bottonDown, bottonUp, bottonSure;
int input = 0;                                    //to analyse the BIN_data
int last_input;                                   //to caculate the kd in "control_motors"
String print_light_data;

int page = 11;
#define maximum_page        14                    //actually page (maximum_page + 1) If you add page please add the switch case in "page control"

void setup() {
	pinMode(A0, INPUT);                           //push_botton analogReader
	pinMode(13, OUTPUT);                          //the LED display
	pinMode(M1, OUTPUT);                          //left motor
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
	push_botton = analogRead(A0);
	control_bottons();
	if(millis() - timer2 >= 50){
		pages_control();
		timer2 = millis();
	}
	analyse_motor_position();
	if(millis() - timer3 >= SampleTime){
    control_motors();
	timer3 = millis();
    }

	if(millis() - timer1 >= 100){
		push_botton = analogRead(A0);
	Wire.requestFrom(front_sensor_board, 2);
	if(Wire.available()){
		wire_read_front_board[0] = Wire.read();

		wire_read_front_board[1] = Wire.read();
	}
	Serial.print("BIN_DATA: ");
    Serial.print(wire_read_front_board[0], BIN);
    Serial.print(" color_data: ");
    Serial.print(wire_read_front_board[1]);
    Serial.print(" stoper: ");
    Serial.print(stoper);
    Serial.print(" push_botton: ");
    Serial.println(push_botton);
	timer1 = millis();
}
 
}

void analyse_motor_position(){
	switch (wire_read_front_board[0]) {
    case B11111000:
      input = -10;
      break;
    case B11110000:
      input = -10;
      break;
    case B11100000:
      input = -10;
      break;
    case B10000000:
      input = -9;
      break;
    case B11000000:
      input = -5;
      break;
    case B01000000:
      input = -4;
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
      break;
    case B00000110:
      input = 5;
      break;
    case B00000010:
      input = 9;
      break;
    case B00001110:
      input = 10;
      break;
    case B00011110:
      input = 10;
      break;
    case B00111110:
      input = 10;
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

    if (stoper == HIGH) {
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

    

void control_bottons(){
	if (analogRead(A0) >= 145 - range && analogRead(A0) <= 145 + range) {
    bottonLeft = HIGH;                                                           //control page--
  }
  else bottonLeft = LOW;

  if (analogRead(A0) >= 501 - range && analogRead(A0) <= 501 + range) {
    bottonRight = HIGH;                                                          //control page++
  }
  else bottonRight = LOW;

  if (analogRead(A0) >= 325 - range && analogRead(A0) <= 325 + range) {
    bottonDown = HIGH;                                                          //control the motor
  }
  else bottonDown = LOW;

  if (analogRead(A0) >= 2 - range && analogRead(A0) <= 2 + range) {
    bottonUp = HIGH;                                                          //sure botton
  }
  else bottonUp = LOW;

  if (analogRead(A0) >= 739 - range && analogRead(A0) <= 739 + range) {
    bottonSure = HIGH;                                                          //sure botton
  }
  else bottonSure = LOW;
}

void pages_control(){
	if(bottonLeft == HIGH) page --;
	if(bottonRight == HIGH) page ++;
	if(page > maximum_page) page = 0;
	if(page < 0 ) page = maximum_page;

	if(bottonSure == HIGH){
		if(page == 0) {remote_calibration_white(); lcd.clear();}                     //calibrtion_color_white
		if(page == 1) {remote_calibration_black(); lcd.clear();}                     //calibration_color_black
		if(page == 2) {remote_calibration_color_1(); lcd.clear();}                   //calibration_color_1
		if(page == 3) {remote_calibration_color_2(); lcd.clear();}                   //calibration_color_2
		if(page == 4) {remote_calibration_color_3(); lcd.clear();}                   //calibration_color_2
		if(page == 5) {remote_calibration_color_4(); lcd.clear();}                   //calibration_color_2
		if(page == 6) {remote_calibration_color_5(); lcd.clear();}                   //calibration_color_2
		if(page == 7) {remote_calibration_color_6(); lcd.clear();}                   //calibration_color_2
		if(page == 8) {remote_calibration_color_7(); lcd.clear();}                   //calibration_color_2

		if(page == 9) {remote_calibration_WB_line(); lcd.clear();}                   //calibration_light_sensor
		if(page == 10) {remote_slave_save_data(); lcd.clear();}                      //save all data to EEPROM including slave and master
		if(page == 11) {control_motor_stoper(); lcd.clear();}                        //control the motor ON/OFF
   }

   if(bottonUp == HIGH){
   	if(page == 12) kp += 0.1;
   	if(page == 13) kd += 0.1;
   	if(page == 14) motor_speed ++;
   }

   if(bottonDown == HIGH){
   	if(page == 12) kp -= 0.1;
   	if(page == 13) kd -= 0.1;
   	if(page == 14) motor_speed --;
   }

   if(bottonLeft == HIGH) lcd.clear();
   if(bottonRight == HIGH) lcd.clear();
   switch(page){
   	case 0:
   	  lcd.setCursor(0,0);
      lcd.print(" Calibration_White 0");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("    on the white    ");
    break;
    case 1:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_Black 1");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("    on the black    ");
    break;
    case 2:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_1 2");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_1   ");
    break;
    case 3:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_2 3");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_2   ");
    break;
    case 4:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_3 4");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_3   ");
    break;
    case 5:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_4 5");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_4   ");
    break;
    case 6:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_5 6");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_5   ");
    break;
    case 7:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_6 7");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_6   ");
    break;
    case 8:
      lcd.setCursor(0,0);
      lcd.print(" Calibration_col_7 8");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("   on the color_7   ");
    break;
    case 9:
      lcd.setCursor(0,0);
      lcd.print("  Calibration_W/B  9");
      lcd.setCursor(0,1);
      lcd.print("   Press to start   ");
      lcd.setCursor(0,2);
      lcd.print(" Please put the car ");
      lcd.setCursor(0,3);
      lcd.print("     on the W/B     ");
    break;
    case 10:
      lcd.setCursor(0,0);
      lcd.print("  Saving the data 10");
      lcd.setCursor(0,1);
      lcd.print("                    ");
      lcd.setCursor(0,2);
      lcd.print("    Press to save   ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
    break;
    case 11:
       switch (input) {
    case -10:
      print_light_data = "0000111";
      break;
    case -9:
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
    case 9:
      print_light_data = "1000000";
      break;
    case 10:
      print_light_data = "1110000";
      break;
  }
      lcd.setCursor(0,0);
      lcd.print("   Motor_position 11");
      lcd.setCursor(0,1);
      lcd.print("       " + print_light_data + "   ");
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(15,3);
      lcd.print(wire_read_front_board[1]);
    break;
    case 12:
      lcd.setCursor(0,0);
      lcd.print("    seting kp     12");
      lcd.setCursor(0,1);
      lcd.print("       kp = ");
      lcd.print(kp);
      lcd.print("   ");
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
    break;
    case 13:
      lcd.setCursor(0,0);
      lcd.print("    seting kd     13");
      lcd.setCursor(0,1);
      lcd.print("       kd = ");
      lcd.print(kd);
      lcd.print("   ");
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
    break;
    case 14:
      lcd.setCursor(0,0);
      lcd.print("   seting speed 14  ");
      lcd.setCursor(0,1);
      lcd.print("      speed = ");
      lcd.print(motor_speed);
      lcd.print("      ");
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
    break;

}
}

//--------------------------------------------------------------------------remote the front_sensor_board BEGIN
void remote_calibration_white(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(1);                                              //color_white
      Wire.endTransmission();    
      digitalWrite(13,HIGH);
}

void remote_calibration_black(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(2);                                              //color_black
      Wire.endTransmission();    
      digitalWrite(13,LOW);
}

void remote_calibration_color_1(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(3);                                              //color_1
      Wire.endTransmission();    
      digitalWrite(13,HIGH);
}

void remote_calibration_color_2(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(4);                                              //color_2
      Wire.endTransmission();    
      digitalWrite(13,LOW);
}

void remote_calibration_color_3(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(5);                                              //color_3
      Wire.endTransmission();    
      digitalWrite(13,HIGH);
}

void remote_calibration_color_4(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(6);                                              //color_4
      Wire.endTransmission();    
      digitalWrite(13,LOW);
}

void remote_calibration_color_5(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(7);                                              //color_5
      Wire.endTransmission();    
      digitalWrite(13,HIGH);
}

void remote_calibration_color_6(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(8);                                              //color_6
      Wire.endTransmission();    
      digitalWrite(13,LOW);
}

void remote_calibration_color_7(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(9);                                              //color_7
      Wire.endTransmission();    
      digitalWrite(13,HIGH);
}

void remote_calibration_WB_line(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(10);                                             //light_sensor_W/B
      Wire.endTransmission();    
      digitalWrite(13,LOW);
}

void remote_slave_save_data(){
	  Wire.beginTransmission(front_sensor_board);
      Wire.write(11);                                             //save all data
      Wire.endTransmission();    
      digitalWrite(13,HIGH);

      EEPROM.put(0, kp);
      EEPROM.put(5, kd);
      EEPROM.put(10, motor_speed);
      
}


//--------------------------------------------------------------------------remote the front_sensor_board END

void control_motor_stoper(){
	if(stoper == HIGH) stoper = LOW;
	else if(stoper == LOW) stoper = HIGH;
}