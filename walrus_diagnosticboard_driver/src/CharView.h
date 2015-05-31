#ifndef CHARVIEW_H
#define CHARVIEW_H

#include "View.h"

class CharView : public View
{
protected:
    char character;
    
public:
    CharView(int row, int column, int character) 
    : View(1, 1, row, column), character(character) {}
    Charview(int row, int column)
    : View(1, 1, row, column), character(' ') {}
    void setChar(int new_char)
    {
        character = new_char;
    }
    virtual void render(char* characters)
    {
        characters[0] = character;
    }
}

#endif
