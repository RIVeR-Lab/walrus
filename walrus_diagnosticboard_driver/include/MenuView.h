#ifndef MENUVIEW_H
#define MENUVIEW_H

#include "View.h"
#include "ScrollView.h"
#include "TextView.h"
#include "ScrollIconView.h"

class MenuView : public ViewContainer
{
protected:
    ScrollView item_list_view;
    TextView header_view;
    ScrollIconView up_down_icon_view;

public:
    MenuView(int height, int width, int row, int column) 
    : ViewContainer(int height, int width, int row, int column) 
    {
        addChild(&item_list 
    }

}


#endif
