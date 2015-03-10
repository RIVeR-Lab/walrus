
#ifndef CONSTANTS_H
#define CONSTANTS_H

//Status LED (pin sources LED current)
#define P_LED_STATUS 18 //PE6

//Buttons (normally open, grounded when closed)
#define P_BUTT_1 5 //PD5
#define P_BUTT_2 8 //PE0
#define P_BUTT_3 6 //PD6
#define P_BUTT_4 7 //PD7
#define P_BUTT_5 9 //PE1

//LEDs (pins source LED current)
#define P_LED_1 27 //PB7
#define P_LED_2 14 //PC4
#define P_LED_3 26 //PB6
#define P_LED_4 15 //PC5
#define P_LED_5 16 //PC6

//Physical button mapping
#define P_BUTT_UP P_BUTT_3
#define P_BUTT_RIGHT P_BUTT_1
#define P_BUTT_DOWN P_BUTT_5	
#define P_BUTT_LEFT P_BUTT_4	
#define P_BUTT_CENT P_BUTT_2	

//Physical LED mapping
#define P_LED_UP P_LED_3	
#define P_LED_RIGHT P_LED_1	
#define P_LED_DOWN P_LED_5	
#define P_LED_LEFT P_LED_4	
#define P_LED_CENT P_LED_2 

//Display pins
#define P_DISP_RS 41 //PF3
#define P_DISP_RW 42 //PF4
#define P_DISP_EN 43 //PF5
#define P_DISP_D0 28 //PA0
#define P_DISP_D1 29 //PA1
#define P_DISP_D2 30 //PA2
#define P_DISP_D3 31 //PA3
#define P_DISP_D4 32 //PA4
#define P_DISP_D5 33 //PA5
#define P_DISP_D6 34 //PA6
#define P_DISP_D7 35 //PA7

//Button debounce interval
#define DEBOUNCE_TIME 5

//LED cycle constants
#define LED_CYCLE_SPEED 100
#define LED_FIRST_DIMM 32
#define LED_SECOND_DIMM 16


//Custom Characters

uint8_t BATT_0_CHAR[8] = { 
  B01110, 
  B11111, 
  B10001,
  B10001, 
  B10001, 
  B10001, 
  B10001, 
  B11111, 
};

uint8_t BATT_20_CHAR[8] = {
  B01110, 
  B11111, 
  B10001, 
  B10001, 
  B10001, 
  B10001, 
  B11111, 
  B11111,
};

uint8_t BATT_40_CHAR[8] = {
  B01110,
  B11111,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
};

uint8_t BATT_60_CHAR[8] = {
  B01110,
  B11111,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
};

uint8_t BATT_80_CHAR[8] = {
  B01110,
  B11111,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

uint8_t BATT_100_CHAR[8] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

uint8_t UP_ARROW_CHAR[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00000,
  B00000,
};

uint8_t DOWN_ARROW_CHAR[8] = {
  B00000,
  B00000,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100,
  B00000,
};


#endif
