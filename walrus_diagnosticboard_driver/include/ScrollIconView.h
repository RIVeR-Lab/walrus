#ifndef SCROLLICON_VIEW
#define SCROLLICON_VIEW

#include "View.h"

class ScrollIconView : public View
{
protected:
    bool show_up;
    bool show_down;

public:
    ScrollIconView(int height, int row, int column) : View(height, 1, row, column), show_up(false), show_down(true)
    {
    }
    
    void showUp(bool show)
    {
        show_up = show;
    }
    void showDown(bool show)
    {
        show_down = show;
    }
    
    virtual void render(char* characters)
    {
        for (int l = 0; l < height; l++)
        {
            if (l == 0 && show_up)
                characters[l] = walrus_firmware_msgs::DiagnosticScreen::UP_ARROW;
            else if (l == height-1 && show_down)
                characters[l] = walrus_firmware_msgs::DiagnosticScreen::DOWN_ARROW;
            else
                characters[l] = ' ';
        }
    }

};


#endif
