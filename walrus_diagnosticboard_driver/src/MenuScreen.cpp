#include "MenuScreen.h"
#include "ros/ros.h"
#include "stdlib.h"

MenuScreen::items = vector<MenuItem>();

//Constructor
MenuScreen::MenuScreen(string title, Screen* previous)
{
	//If the title is greater than 18 characters long, trim it down to 18 characters (18 + 2 for brackets ([]) = 20 character display line)
	if (title.length() > 18)
	{
		ROS_WARN("Diagnostics menu title'" + title + "' is too long for the display");
		this->title = title.substr(18)
	}
	else
		this->title = title;
	
	this->previous = previous;
	
	//Set the first item to be selected
	reset();
}

//Reset the menu so the first item is selected
void MenuScreen::reset()
{
	this->selected = 0;
	this->pos = 1;
}

//Add an item to this menu
void MenuScreen::addMenuItem(MenuItem item)
{
	items.push_back(item);
}

//Create a display for the curren state of the menu
char* MenuScreen::getDisplay()
{
	char display[80];
	memset(display, ' ', 80);

	//Write first line
	display[0] = '[';
	strcpy(display+1, title.c_str(), title.length());
	display[title.length+1] = ']';

	if (items.size() > 0)
	{
		//Write other lines
		int itemstart = selected-pos;
		for (int l = 1; l <= 3; l++)
		{
			if (l < items.size())
				strcpy(display+((20*l)+1), item.getText().c_str(), item.getText().length());
		}
	
		//Up scroll arrow
		if (selected > 0)
			display[39] = UP_ARROW;
		
		//Down scroll arrow
		if (selected < items.size()-1)
			display[79] = DOWN_ARROW;
		
		//Select arrow
		display[20*pos] = RIGHT_ARROW;
	}
	else
		strcpy(display+20, "No menu items", 13);
	
	return display;
}

//Perform and action on the current state of the menu and r
Screen* MenuScreen::doAction(int action)
{
	if (action == LEFT)		//Return to previous screen
		return previous;
	else if (action == ENTER) //Go to the screen indicated by the selected item
		return items.at(selected).whenSelected();
	else if (action == UP && selected > 0)
	{ 
		selected--;ss
		if (pos > 1)
			pos--;
		return this;
	}
	else if (action == DOWN && selected < items.size())
	{
		selected++;
		if (pos < 3)
			pos++;
		return this;
	}
}
