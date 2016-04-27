
/*-----------------------------------------------------------------------
	Function:      	I2C sensor front data board (slave)

  I2C ADDRESS:    0x01

	INPUT:			I2C 5 byte input 				read data_calibration()
					1 - color sensor white calibration (auto read )
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
					Light arduino LED pin 13 when calibrating


	OUTPUT:      	I2C 4 bytes output 			read requestEvent()
					i2c_sending[0] = line_sensor_bin_data;
					i2c_sending[1] = distance_Serial[1];      1:White
                                                    2:Black
                                                    3:color_1
                                                    4:color_2
                                                    5:color_3
                                                    6:color_4
                                                    7:color_5
                                                    8:color_6
                                                    9:color_7

	Desc:          	using I2C to return the sensor data
	-----------------------------------------------------------------------*/

#include <TimerOne.h>
#include <EEPROM.h>
#include <Wire.h>

//pin-------------
#define S2     4
#define S3     5
//filter control of colo sensor, 00 = RED, 01 = GREEN, 10 = BLUE, 11 = no filter
#define color_sensor_output_right    3
#define color_sensor_output_left    2
#define multiplexer_counter_S2   10                  //control the ic
#define multiplexer_counter_S1   11                  //control the ic
#define multiplexer_counter_S0   12                  //control the ic
//S2      S1      S0        multiplexer
// 0       0       0        Y0 --> Z
// 0       0       1        Y1 --> Z
// 0       1       0        Y2 --> Z
// 0       1       1        Y3 --> Z
// 1       0       0        Y4 --> Z
// 1       0       1        Y5 --> Z
// 1       1       0        Y6 --> Z
#define line_sensor_pin          A0                  //light_data_analogInput
#define LED 13                                       //HIGH when it is calibrating


//configuration-------------
#define i2c_address_of_this_board 0x01
#define multiplexer_time_step  10                    //the analyse light data delaytime
#define debug_line_sensor_data 1
#define calibration_counter_time 500




int   calibration_color_1_left[3] = {43, 77, 55};
int   calibration_color_1_right[3] = {39, 81, 30};
int   calibration_color_2_left[3] = {43, 77, 55};
int   calibration_color_2_right[3] = {39, 81, 30};
int   calibration_color_3_left[3] = {43, 77, 55};
int   calibration_color_3_right[3] = {39, 81, 30};
int   calibration_color_4_left[3] = {43, 77, 55};
int   calibration_color_4_right[3] = {39, 81, 30};
int   calibration_color_5_left[3] = {43, 77, 55};
int   calibration_color_5_right[3] = {39, 81, 30};
int   calibration_color_6_left[3] = {43, 77, 55};
int   calibration_color_6_right[3] = {39, 81, 30};
int   calibration_color_7_left[3] = {43, 77, 55};
int   calibration_color_7_right[3] = {39, 81, 30};

int   calibration_white_left[3] = {27, 27, 27};
int   calibration_white_right[3] = {30, 30, 30};
int   calibration_black_left[3] = {3, 3, 3};
int   calibration_black_right[3] = {4, 4, 4};
int   white_D[7] = {59, 61, 61, 71, 59, 62, 86};
int   black_D[7] = {110, 108, 102, 120, 93, 104, 121};
long  cumulative_calibration_data[18][3];
long  calibration_counter;
long W_distance[2], B_distance[2], color_1_distance[2], color_2_distance[2], color_3_distance[2], color_4_distance[2], color_5_distance[2], color_6_distance[2], color_7_distance[2];
String print_text;



bool  calibrating;
byte  BIN_DATA, line_sensor_bin_data, Wire_Read;
//byte  analyse_defferent_light_digit;
byte  distance_Serial[2];
unsigned long timer[3];
int   RGB_value_left[3];
int   g_count_left = 0;    // count the frequecy
int   g_array_left[3];     // store the RGB value
int   g_flag = 0;     // filter of RGB queue
int   RGB_value_right[3];
int   g_array_temporarily[3][2];
int   g_count_right = 0;    // count the frequecy
int   g_array_right[3];     // store the RGB value


//light_sensor Begin
bool L0, L1, L2;
byte counter = 0;

int   raw_light_sensor_data[7];
bool light_digital[7] ;


////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //i2c_setup
  Wire.begin(i2c_address_of_this_board);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);         // register event

  //color_sensor_setup
  Timer1.initialize(10000);             // 5000us = 5ms
  Timer1.attachInterrupt(TSC_Callback);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(color_sensor_output_right, INPUT);
  pinMode(color_sensor_output_left, INPUT);
  attachInterrupt(digitalPinToInterrupt(color_sensor_output_right), TSC_Count2, RISING);
  attachInterrupt(digitalPinToInterrupt(color_sensor_output_left), TSC_Count, RISING);

  //read data from EEPROM
    for (byte i = 0; i <= 12 ; i += 2)    EEPROM.get(i, white_D[i / 2]);
    for (byte i = 15; i <= 27 ; i += 2)   EEPROM.get(i, black_D[(i - 15) / 2]);
    for (byte i = 30; i <= 34 ; i += 2)   EEPROM.get(i, calibration_white_left[(i - 30) / 2]);
    for (byte i = 37; i <= 41 ; i += 2)   EEPROM.get(i, calibration_black_left[(i - 37) / 2]);
    for (byte i = 44; i <= 48 ; i += 2)   EEPROM.get(i, calibration_color_1_left[(i - 44) / 2]);

    for (byte i = 51; i <= 55 ; i += 2)   EEPROM.get(i, calibration_white_right[(i - 51) / 2]);
    for (byte i = 58; i <= 62 ; i += 2)   EEPROM.get(i, calibration_black_right[(i - 58) / 2]);
    for (byte i = 65; i <= 69 ; i += 2)   EEPROM.get(i, calibration_color_1_right[(i - 65) / 2]);
  pinMode(multiplexer_counter_S2, OUTPUT);
  pinMode(multiplexer_counter_S1, OUTPUT);
  pinMode(multiplexer_counter_S0, OUTPUT);
  pinMode(line_sensor_pin, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED,calibrating);
  // multiplexer of the light digital sensor
  if (millis() - timer[1] >= multiplexer_time_step) {

    //Period of line sensor = multiplexer_time_step * 7 = 70ms
    raw_light_sensor_data[counter] = analogRead(line_sensor_pin);

    if (debug_line_sensor_data == HIGH) {
      //print_text += String(raw_light_sensor_data[counter]) + "  ";
      print_text += String(raw_light_sensor_data[counter]) + "  ";
    }

    // without calibrating
    if (calibrating == LOW) {
      if ( raw_light_sensor_data[counter] >= (white_D[counter] + black_D[counter]) / 2)  light_digital[counter] = HIGH;
      else  light_digital[counter] = LOW;

      BIN_DATA = BIN_DATA | light_digital[counter];
      BIN_DATA = BIN_DATA << 1;
    }
    //calibrating
    else if (calibrating == HIGH && Wire_Read == 10) {
      if (calibration_counter > 0) {
        if (raw_light_sensor_data[counter] < black_D[counter])  black_D[counter] = raw_light_sensor_data[counter];
        if (raw_light_sensor_data[counter] > white_D[counter])  white_D[counter] = raw_light_sensor_data[counter];
        calibration_counter--;
      }
      else {
        calibrating = LOW;
        Wire_Read = 0;
      }
    }

    if (counter < 6)    counter++;
    else  {
      counter = 0;
      line_sensor_bin_data = BIN_DATA;
  //    analyse_defferent_light_digit = analyse_defferent_light_data();  //   Wrong is 1 | True is 0
      if (debug_line_sensor_data == HIGH) {
        Serial.print(print_text);
        Serial.print("   led:");
        Serial.print(calibrating);
        Serial.print("   Wire_Read   ");
        Serial.print(Wire_Read);
        Serial.print("  calibration_counter:  ");
        Serial.print(calibration_counter);
        Serial.print("  left ");
        Serial.print(g_array_left[0]);
        Serial.print(",");
        Serial.print(g_array_left[1]);
        Serial.print(",");
        Serial.print(g_array_left[2]);
        Serial.print("  right ");
        Serial.print(g_array_right[0]);
        Serial.print(",");
        Serial.print(g_array_right[1]);
        Serial.print(",");
        Serial.print(g_array_right[2]);
        Serial.print("    distance_Serial_left ");
        Serial.print(distance_Serial[0]);
        Serial.print("  left_distance:  ");
        Serial.print(W_distance[0]);
        Serial.print(",");
        Serial.print(color_1_distance[0]);
        Serial.print(",");
        Serial.print(B_distance[0]);
        Serial.print("  right_distance:  ");
        Serial.print(W_distance[1]);
        Serial.print(",");
        Serial.print(color_1_distance[1]);
        Serial.print(",");
        Serial.print(B_distance[1]);
        Serial.print("   ");
        Serial.print(calibration_color_1_left[0]);
        Serial.print(" ");
        Serial.print(calibration_color_1_left[1]);
        Serial.print(" ");
        Serial.print(calibration_color_1_left[2]);
        Serial.print("   ");
        Serial.print(calibration_color_1_right[0]);
        Serial.print(" ");
        Serial.print(calibration_color_1_right[1]);
        Serial.print(" ");
        Serial.println(calibration_color_1_right[2]);
        print_text = "";
      }
    }
    L2 = bitRead(counter, 2);
    L1 = bitRead(counter, 1);
    L0 = bitRead(counter, 0);

    digitalWrite(multiplexer_counter_S2, L2);
    digitalWrite(multiplexer_counter_S1, L1);
    digitalWrite(multiplexer_counter_S0, L0);
    timer[1] = millis();
  }
}



void TSC_Callback() {
  if (g_flag == 0) {                       //Filter without Red
    g_count_left = 0;
    g_count_right = 0;
    g_flag ++;
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    Timer1.setPeriod(10000);
  }

  else if (g_flag == 1) {                     //Filter without Green
    g_array_temporarily[0][0] = g_count_left;
    g_array_temporarily[0][1] = g_count_right;
    g_count_left = 0;
    g_count_right = 0;
    g_flag ++;
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    Timer1.setPeriod(10000);
  }

  else if (g_flag == 2) {                       //Filter without Blue
    g_array_temporarily[1][0] = g_count_left;
    g_array_temporarily[1][1] = g_count_right;
    g_count_left = 0;
    g_count_right = 0;
    g_flag ++;
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    Timer1.setPeriod(10000);
  }

  else if (g_flag == 3) {
    g_array_temporarily[2][0] = g_count_left;
    g_array_temporarily[2][1] = g_count_right;
    g_count_left = 0;
    g_count_right = 0;
    for (int i = 0; i <= 2; i++) {
      g_array_left[i] = g_array_temporarily[i][0];
      g_array_right[i] = g_array_temporarily[i][1];
    }

    // without calibrating
    if (calibrating == LOW) {
      get_color_senor_value();
      analyse_color_distances();

    }

    // calibrating
    else if (calibrating == HIGH && Wire_Read > 0 && Wire_Read <= 3) {
      color_data_calibration();
  
    }
    g_flag = 0;

  }
}




void get_color_senor_value() {
  for (int i = 0; i <= 2; i++) {
    RGB_value_left[i] = map(g_array_left[i], calibration_black_left[i], calibration_white_left[i], 0, 255);
    RGB_value_right[i] = map(g_array_right[i], calibration_black_right[i], calibration_white_right[i], 0, 255);
  }
}

void analyse_color_distances() {

  W_distance[0] = (g_array_left[0] - calibration_white_left[0]) * (g_array_left[0] - calibration_white_left[0]) + (g_array_left[1] - calibration_white_left[1]) * (g_array_left[1] - calibration_white_left[1]) + (g_array_left[2] - calibration_white_left[2]) * (g_array_left[2] - calibration_white_left[2]);
  B_distance[0] = (g_array_left[0] - calibration_black_left[0]) * (g_array_left[0] - calibration_black_left[0]) + (g_array_left[1] - calibration_black_left[1]) * (g_array_left[1] - calibration_black_left[1]) + (g_array_left[2] - calibration_black_left[2]) * (g_array_left[2] - calibration_black_left[2]);
  color_1_distance[0] = (g_array_left[0] - calibration_color_1_left[0]) * (g_array_left[0] - calibration_color_1_left[0]) + (g_array_left[1] - calibration_color_1_left[1]) * (g_array_left[1] - calibration_color_1_left[1]) + (g_array_left[2] - calibration_color_1_left[2]) * (g_array_left[2] - calibration_color_1_left[2]);

  if ( W_distance[0] <= 200 && W_distance[0] >= -200) {
    distance_Serial[0] = 1;
  }
  else if ( color_1_distance[0] <= 30 && color_1_distance[0] >= -30) {
    distance_Serial[0] = 2;
  }
  else if ( B_distance[0] <= 200  && B_distance[0] >= -200) {
    distance_Serial[0] = 3;
  }



  W_distance[1] = (g_array_right[0] - calibration_white_right[0]) * (g_array_right[0] - calibration_white_right[0]) + (g_array_right[1] - calibration_white_right[1]) * (g_array_right[1] - calibration_white_right[1]) + (g_array_right[2] - calibration_white_right[2]) * (g_array_right[2] - calibration_white_right[2]);
  B_distance[1] = (g_array_right[0] - calibration_black_right[0]) * (g_array_right[0] - calibration_black_right[0]) + (g_array_right[1] - calibration_black_right[1]) * (g_array_right[1] - calibration_black_right[1]) + (g_array_right[2] - calibration_black_right[2]) * (g_array_right[2] - calibration_black_right[2]);
  color_1_distance[1] = (g_array_right[0] - calibration_color_1_right[0]) * (g_array_right[0] - calibration_color_1_right[0]) + (g_array_right[1] - calibration_color_1_right[1]) * (g_array_right[1] - calibration_color_1_right[1]) + (g_array_right[2] - calibration_color_1_right[2]) * (g_array_right[2] - calibration_color_1_right[2]);
  color_2_distance[1] = (g_array_right[0] - calibration_color_2_right[0]) * (g_array_right[0] - calibration_color_2_right[0]) + (g_array_right[1] - calibration_color_2_right[1]) * (g_array_right[1] - calibration_color_2_right[1]) + (g_array_right[2] - calibration_color_2_right[2]) * (g_array_right[2] - calibration_color_2_right[2]);
  color_3_distance[1] = (g_array_right[0] - calibration_color_3_right[0]) * (g_array_right[0] - calibration_color_3_right[0]) + (g_array_right[1] - calibration_color_3_right[1]) * (g_array_right[1] - calibration_color_3_right[1]) + (g_array_right[2] - calibration_color_3_right[2]) * (g_array_right[2] - calibration_color_3_right[2]);
  color_4_distance[1] = (g_array_right[0] - calibration_color_4_right[0]) * (g_array_right[0] - calibration_color_4_right[0]) + (g_array_right[1] - calibration_color_4_right[1]) * (g_array_right[1] - calibration_color_4_right[1]) + (g_array_right[2] - calibration_color_4_right[2]) * (g_array_right[2] - calibration_color_4_right[2]);
  color_5_distance[1] = (g_array_right[0] - calibration_color_5_right[0]) * (g_array_right[0] - calibration_color_5_right[0]) + (g_array_right[1] - calibration_color_5_right[1]) * (g_array_right[1] - calibration_color_5_right[1]) + (g_array_right[2] - calibration_color_5_right[2]) * (g_array_right[2] - calibration_color_5_right[2]);
  color_6_distance[1] = (g_array_right[0] - calibration_color_6_right[0]) * (g_array_right[0] - calibration_color_6_right[0]) + (g_array_right[1] - calibration_color_6_right[1]) * (g_array_right[1] - calibration_color_6_right[1]) + (g_array_right[2] - calibration_color_6_right[2]) * (g_array_right[2] - calibration_color_6_right[2]);
  color_7_distance[1] = (g_array_right[0] - calibration_color_7_right[0]) * (g_array_right[0] - calibration_color_7_right[0]) + (g_array_right[1] - calibration_color_7_right[1]) * (g_array_right[1] - calibration_color_7_right[1]) + (g_array_right[2] - calibration_color_7_right[2]) * (g_array_right[2] - calibration_color_7_right[2]);

  if ( W_distance[1] <= 200 && W_distance[1] >= -200) {
    distance_Serial[1] = 1;
  }
  else if ( B_distance[1] <= 200  && B_distance[1] >= -200) {
    distance_Serial[1] = 2;
  }
  else if ( color_1_distance[1] <= 30 && color_1_distance[1] >= -30) {
    distance_Serial[1] = 3;
  }
  else if ( color_2_distance[1] <= 30 && color_2_distance[1] >= -30) {
    distance_Serial[1] = 4;
  }
  else if ( color_3_distance[1] <= 30 && color_3_distance[1] >= -30) {
    distance_Serial[1] = 5;
  }
  else if ( color_4_distance[1] <= 30 && color_4_distance[1] >= -30) {
    distance_Serial[1] = 6;
  }
  else if ( color_5_distance[1] <= 30 && color_5_distance[1] >= -30) {
    distance_Serial[1] = 7;
  }
  else if ( color_6_distance[1] <= 30 && color_6_distance[1] >= -30) {
    distance_Serial[1] = 8;
  }
  else if ( color_7_distance[1] <= 30 && color_7_distance[1] >= -30) {
    distance_Serial[1] = 9;
  }

}


void TSC_Count() {
  g_count_left ++ ;
}
void TSC_Count2() {
  g_count_right ++ ;
}


void requestEvent() { 
  byte i2c_sending[2];
  i2c_sending[0] = line_sensor_bin_data;
 // i2c_sending[1] = distance_Serial[0];
  i2c_sending[1] = distance_Serial[1];
  //i2c_sending[3] = analyse_defferent_light_digit;
  Wire.write(i2c_sending, 2);
}



void receiveEvent(int howMany) {
  Wire_Read = Wire.read();
  calibrating = HIGH;
  calibration_counter = calibration_counter_time;
  if (Wire_Read == 4) {
    for (int i = 0; i <= 6 ; i++)	black_D[i] = 1023;
    for (int i = 0; i <= 6 ; i++)	white_D[i] = 0;
  }
  if (Wire_Read == 11) {

    for (byte i = 0; i <= 12 ; i += 2) 		EEPROM.put(i, white_D[i / 2]);
    for (byte i = 15; i <= 27 ; i += 2) 	EEPROM.put(i, black_D[(i - 15) / 2]);
    for (byte i = 30; i <= 34 ; i += 2)		EEPROM.put(i, calibration_white_left[(i - 30) / 2]);
    for (byte i = 37; i <= 41 ; i += 2)		EEPROM.put(i, calibration_black_left[(i - 37) / 2]);
    for (byte i = 44; i <= 48 ; i += 2)   EEPROM.put(i, calibration_color_1_left[(i - 44) / 2]);

    for (byte i = 51; i <= 55 ; i += 2)   EEPROM.put(i, calibration_white_right[(i - 51) / 2]);
    for (byte i = 58; i <= 62 ; i += 2)   EEPROM.put(i, calibration_black_right[(i - 58) / 2]);
    for (byte i = 65; i <= 69 ; i += 2)   EEPROM.put(i, calibration_color_1_right[(i - 65) / 2]);


    calibrating = LOW;
    Wire_Read = 0;
    calibration_counter == 0;
  }


}



void color_data_calibration() {
  if (Wire_Read == 1) {
    if (calibration_counter > 0) {                              //White calabration
      for (int i = 0; i <= 2; i++) {
        cumulative_calibration_data[0][i] += g_array_left[i];
        cumulative_calibration_data[1][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_white_left[i] = cumulative_calibration_data[0][i] / calibration_counter_time;
        calibration_white_right[i] = cumulative_calibration_data[1][i] / calibration_counter_time;
        cumulative_calibration_data[0][i] = 0;
        cumulative_calibration_data[1][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
  }

  else if (Wire_Read == 2) {                                    //Black calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        cumulative_calibration_data[2][i] += g_array_left[i];
        cumulative_calibration_data[3][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_black_left[i] = cumulative_calibration_data[2][i] / calibration_counter_time;
        calibration_black_right[i] = cumulative_calibration_data[3][i] / calibration_counter_time;
        cumulative_calibration_data[2][i] = 0;
        cumulative_calibration_data[3][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
  }

  else if (Wire_Read == 3) {                                    //color_1 calabration              
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[4][i] += g_array_left[i];
        cumulative_calibration_data[5][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_1_left[i] = cumulative_calibration_data[4][i] / calibration_counter_time;
        calibration_color_1_right[i] = cumulative_calibration_data[5][i] / calibration_counter_time;
        cumulative_calibration_data[4][i] = 0;
        cumulative_calibration_data[5][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
  }

  else if(Wire_Read == 4) {                                     //color_2 calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[6][i] += g_array_left[i];
        cumulative_calibration_data[7][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_2_left[i] = cumulative_calibration_data[6][i] / calibration_counter_time;
        calibration_color_2_right[i] = cumulative_calibration_data[7][i] / calibration_counter_time;
        cumulative_calibration_data[6][i] = 0;
        cumulative_calibration_data[7][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
    }

  else if(Wire_Read == 5) {                                     //color_3 calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[8][i] += g_array_left[i];
        cumulative_calibration_data[9][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_3_left[i] = cumulative_calibration_data[8][i] / calibration_counter_time;
        calibration_color_3_right[i] = cumulative_calibration_data[9][i] / calibration_counter_time;
        cumulative_calibration_data[8][i] = 0;
        cumulative_calibration_data[9][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
    }

  else if(Wire_Read == 6) {                                     //color_4 calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[10][i] += g_array_left[i];
        cumulative_calibration_data[11][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_4_left[i] = cumulative_calibration_data[10][i] / calibration_counter_time;
        calibration_color_4_right[i] = cumulative_calibration_data[11][i] / calibration_counter_time;
        cumulative_calibration_data[10][i] = 0;
        cumulative_calibration_data[11][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
    }

  else if(Wire_Read == 7) {                                     //color_5 calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[12][i] += g_array_left[i];
        cumulative_calibration_data[13][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_5_left[i] = cumulative_calibration_data[12][i] / calibration_counter_time;
        calibration_color_5_right[i] = cumulative_calibration_data[13][i] / calibration_counter_time;
        cumulative_calibration_data[12][i] = 0;
        cumulative_calibration_data[13][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
    }

  else if(Wire_Read == 8) {                                     //color_6 calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[14][i] += g_array_left[i];
        cumulative_calibration_data[15][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_6_left[i] = cumulative_calibration_data[14][i] / calibration_counter_time;
        calibration_color_6_right[i] = cumulative_calibration_data[15][i] / calibration_counter_time;
        cumulative_calibration_data[14][i] = 0;
        cumulative_calibration_data[15][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
    }

  else if(Wire_Read == 9) {                                     //color_7 calabration
    if (calibration_counter > 0) {
      for (int i = 0; i <= 2; i++) {
        //get_color_senor_value();
        cumulative_calibration_data[16][i] += g_array_left[i];
        cumulative_calibration_data[17][i] += g_array_right[i];
      }
      calibration_counter--;
    }
    else {
      for (int i = 0; i <= 2; i++) {
        calibration_color_7_left[i] = cumulative_calibration_data[16][i] / calibration_counter_time;
        calibration_color_7_right[i] = cumulative_calibration_data[17][i] / calibration_counter_time;
        cumulative_calibration_data[16][i] = 0;
        cumulative_calibration_data[17][i] = 0;
      }
      calibrating = LOW;
      Wire_Read = 0;
    }
    }
  }

  






/*bool analyse_defferent_light_data() {
  bool _analyse_defferent_light_digit;
  if (line_sensor_bin_data == B00000000 ) {
    _analyse_defferent_light_digit = 1;
  }
  else _analyse_defferent_light_digit = 0;

  return _analyse_defferent_light_digit;
}
*/
