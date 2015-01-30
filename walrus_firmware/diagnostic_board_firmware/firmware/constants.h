
#ifndef CONSTANTS_H
#define CONSTANTS_H

//Status LED (pin sources LED current)
#define P_LED_STATUS 18 //PE6

//Button pins (normally open, grounded when closed)+
#define P_BUTT_UP 6	//PD6, Button 1
#define P_BUTT_RIGHT 5	//PD5, Button 2
#define P_BUTT_DOWN 8	//PE0, Button 3
#define P_BUTT_LEFT 7 	//PD7, Button 4
#define P_BUTT_CENT 9	//PE1, Button 5

//Button LED pins (pins source LED current)
#define P_LED_UP 26		//PB6, Button 1
#define P_LED_RIGHT 27 	//PB7, Button 2
#define P_LED_DOWN 14	//PC4, Button 3
#define P_LED_LEFT 15	//PC5, Button 4
#define P_LED_CENT 16	//PC6, Button 5

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
#define DEBOUNCE_TIME 200

//LED cycle speed
#define LED_CYCLE_SPEED 250


#endif