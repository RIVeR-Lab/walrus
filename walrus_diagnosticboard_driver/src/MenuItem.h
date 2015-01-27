
#ifndef MENUITEM_H
#define MENUITEM_H

#include <string.h>
#include "Screen.h"

class MenuItem
{
private:
	string text;
	Screen whenSelected;

public:
	MenuItem(string text, Screen whenSelected)
	string getText();
	Screen whenSelected();
}


#endif
