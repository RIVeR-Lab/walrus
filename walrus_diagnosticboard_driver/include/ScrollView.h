#ifndef SCROLL_VIEW
#define SCROLL_VIEW

#include "View.h"
#include <stdio.h>
#include <vector>
#include <string>

class ScrollView : public View
{
protected:
    int linePtr;
    View* scrolled_view;
    int top() {return 0;}
    int bottom() {return scrolled_view->getHeight() - height;}
    int clampLinePtr()
    {
        if (linePtr < top())
            linePtr = top();
        else if (linePtr > bottom())
            linePtr = bottom();
    }
     
public:
    ScrollView(int height, int width, int row, int column, View* scrolled_view) 
    : View(height, width, row, column), scrolled_view(scrolled_view), linePtr(0)
    {
    }   
    void upOne()
    {
        up(1);
    }
    void up(int num_lines)
    {
        offsetLinePtr(num_lines*-1);
    }
    void downOne()
    {
        down(1);
    }
    void down(int num_lines)
    {
        offsetLinePtr(num_lines);
    }
    void gotoTop()
    {
        setLinePtr(top());
    }
    void gotoBottom()
    {
        setLinePtr(bottom());
    }
    void offsetLinePtr(int offset)
    {
        setLinePtr(linePtr + offset);
    }
    void setLinePtr(int line)
    {
        linePtr = line;
        clampLinePtr();
    }
    bool atTop()
    {
        return linePtr == top();
    }
    bool atBottom()
    {
        return linePtr == bottom();
    }
    int getLinePtr()
    {
        clampLinePtr();
        return linePtr;
    }
    virtual void render(char* characters)
    {
        int sv_width = scrolled_view->getWidth();
        int sv_height = scrolled_view->getHeight();
        char* sv_buff = new char[sv_width*sv_height];
        scrolled_view->render(sv_buff);
        clampLinePtr();
        int copy_width, space_width;
        if (scrolled_view->getWidth() > width)
        {
            copy_width = width;
            space_width = 0;
        }
        else
        {
            copy_width = scrolled_view->getWidth();
            space_width = width-copy_width;
        }    
        for (int l = 0; l < height; l++)
        {
            memcpy(characters+(l*width),sv_buff+((linePtr+l)*sv_width),copy_width);
            memset(characters+(l*width)+copy_width, ' ', space_width);
        }
        delete[] sv_buff;
    }
};


#endif
