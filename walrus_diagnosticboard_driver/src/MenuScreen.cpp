#i
nclude "MenuScreen.h"
#include "ros/ros.h"
#include "stdlib.h"
#include "walrus_firmware_msgs/DiagnosticTXMsg.h"

//Initialize empty item vector
MenuScreen::items = vector<MenuItem>();

//Constructor
MenuScreen::MenuScreen(string title, Screen* previous)
{
	//If the title is greater than 18 characters long, trim it down to 18 characters (18 + 2 for brackets ([]) = 20 character display line)
	if (title.length() > 18)
	{
		ROS_WARN("Diagnostics menu title'" + title + "' is too long for the display");
		this->title = title.substr(18);
	}
	else
		this->title = title;
	
	//Assure the provided screen is not null
	if (!previous)
		ROS_FATAL("Diagnostics menu with title '" + title + "' was provided with a NULL previous screen");
	else
		this->previous = previous;
	
	//Set the first item to be selected
	reset();
}

//Reset the menu so the first item is selected and on line 1
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
	//Create a display frame and set it to all spaces
	//Display frame is 80 characters long and stored in row major order
	char display[80];
	memset(display, ' ', 80);

	//Write the title and brackets to the line 0 of the display frame
	display[0] = '[';
	strcpy(display+1, title.c_str(), title.length());
	display[title.length+1] = ']';

	//Make sure there is at least one item in the menu
	if (items.size() > 0)
	{
		//Write lines 1-3 as long as there are items to fill them
		int itemstart = selected-pos; //Find the item that will go on line 1
		for (int l = 1; l <= 3; l++)
		{
			if (l < items.size())
				strcpy(display+((20*l)+1), item.getText().c_str(), item.getText().length());
		}
	
		//Put the scroll up arrow at screen position (1, 19) if scrolling up is permitted
		if (selected > 0)
			display[39] = UP_ARROW;
		
		//Put the scroll down arrow at screen position (3, 19) if scrolling down is permitted
		if (selected < items.size()-1)
			display[79] = DOWN_ARROW;
		
		//Put the select arrow in front of the selected item's line
		display[20*pos] = RIGHT_ARROW;
	}
	else
		strcpy(display+20, "No menu items", 13);
		
	//Return the display frame 
	return display;
}

//Perform and action on the current state of the menu and r
Screen* MenuScreen::doAction(int action)
{
	//Return to the previous screen
	if (action == walrus_firmware_msgs::DiagnosticTXMsg::LEFT)	
		return previous;
	//Go the screen indicated by the selected menu item
	else if (action == walrus_firmware_msgs::DiagnosticTXMsg::ENTER)
		return items.at(selected).whenSelected();
	//Move up in the menu
	else if (action == walrus_firmware_msgs::DiagnosticTXMsg::UP && selected > 0)
	{ 
		selected--;ss
		if (pos > 1)
			pos--;
	}
	//Move down in the menu
	else if (action == walrus_firmware_msgs::DiagnosticTXMsg::DOWN && selected < items.size()-1)
	{
		selected++;
		if (pos < 3)
			pos++;
	}
	//Keep displaying this menu
	return this;
}


