#ifndef SCREEN_H
#define SCREEN_H

#define UP 1
#define RIGHT 2
#define DOWN 3
#define LEFT 4
#define ENTER 5

#define UP_ARROW 5
#define DOWN_ARROW 6
#define LEFT_ARROW 127
#define RIGHT_ARROW 126
#define BATT_0 0
#define BATT_25 1
#define BATT 50 2
#define BATT 75 3
#define BATT 100 4

class Screen
{
public:
	virtual char* getDisplay() = 0;
	virtual Screen* doAction(int action) = 0;
}

#endif 
