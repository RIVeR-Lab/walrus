
#include "MenuItem.h"

//Constructor
MenuItem::MenuItem(string title, Screen* whenSelected)
{
	//If the text is greater than 18 characters long, trim it down to 18 characters (18 + 1 for selection arror + 1 for up/down arrows = 20 character display line)
	if (title.length() > 18)
	{
		ROS_WARN("Diagnostics menu item'" + text + "' is too long for the display");
		this->text = text.substr(18);
	}
	else
		this->text = text;
	
	//Assure the provided screen is not null
	if (!whenSelected)
		ROS_FATAL("Diagnostics menu item with text '" + text + "' was provided with a NULL when selected screen");
	else
		this->whenSelected = whenSelected;
	
}

//Return the menu item text
string MenuItem::getText()
{
	return title;
}

//Return the screen to jump to when selected
Screen* MenuItem::whenSelected()
{
	return whenSelected;
}
