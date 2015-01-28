
#ifndef SCREEN_H
#define SCREEN_H

//Includes message constants for buttons (UP, RIGHT, DOWN, LEFT, ENTER)
#include <walrus_firmware_msgs/DiagnosticRXMsg.h>

//Special Character
#define UP_ARROW 5
#define DOWN_ARROW 6
#define LEFT_ARROW 127
#define RIGHT_ARROW 126
#define BATT_0 0
#define BATT_25 1
#define BATT 50 2
#define BATT 75 3
#define BATT 100 4

//Abstract class to generalize displaying and interacting with components of the diagnostic interface
class Screen
{
public:
	//Gets an array of characters to display on the screen when this component is active
	//Returns an array of exactly 80 characters in row major order (4 rows, 20 columns)
	virtual char* getDisplay() = 0;
	//Called to perfom an button press on the component (refer to constants in walrus_firmware_msgs/DiagnosticRXMsg.h)
	virtual Screen* doAction(int action) = 0;
}

#endif 
