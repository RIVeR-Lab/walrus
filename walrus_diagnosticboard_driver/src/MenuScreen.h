#ifndef MENUSCREEN_H
#define MENUSCREEN_H

#include <string.h>
#include "MenuItem.h"
#include "Screen.h"
#include <vector.h>

class MenuScreen : Screen
{

private:
	vector<MenuItem> items;
	Screen* previous;
	string title;
	int selected;
	int pos;
	
public:
	MenuScreen(string title, Screen* previous);
	void addMenuItem(MenuItem item);
	void reset();
	virtual char* getDisplay();
	virtual Screen* doAction(int action);
	
}

#endif 
