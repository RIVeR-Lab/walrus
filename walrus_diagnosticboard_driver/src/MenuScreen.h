
#ifndef MENUSCREEN_H
#define MENUSCREEN_H

#include <string.h>
#include "MenuItem.h"
#include "Screen.h"
#include <vector.h>

//Class for displaying and interacting with a menu
class MenuScreen : Screen
{

private:
	//Vector of items in the menu
	vector<MenuItem> items;
	//Previous screen component that led to this menu (for going back)
	Screen* previous;
	//Title of this menu (will be displayed in brackets on the top line)
	string title;
	//Index of the currently selected menu item
	int selected;
	//Line of the currently selected menu item (1, 2 or 3 as 0 is for the title)
	int pos;
	
public:
	//Constructor
	MenuScreen(string title, Screen* previous);
	//Add an item to this menu
	void addMenuItem(MenuItem item);
	//Reset the menu so the first item is on line 1 and selected
	void reset();
	//Get the character array to display the current state of this menu
	//Overrides method in Screen class
	virtual char* getDisplay();
	//Perform a button press
	//Overrides method in Screen class
	virtual Screen* doAction(int action);
	
}

#endif 
