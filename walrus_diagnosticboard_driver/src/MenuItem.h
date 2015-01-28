
#ifndef MENUITEM_H
#define MENUITEM_H

#include <string.h>
#include "Screen.h"

//Represents a item in a MenuScreen menu
class MenuItem
{
protected:
	//Name of the item (must by 18 characters or less
	string text;
	//Screen to jump to when selected
	Screen* whenSelected;

public:
	//Constructor
	MenuItem(string text, Screen* whenSelected)
	//Return the menu item text
	virtual string getText();
	//Return the screen to jump to when selected
	virtual Screen* whenSelected();
}


#endif
