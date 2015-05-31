#ifndef VIEW_H
#define VIEW_H

#include <walrus_firmware_msgs/DiagnosticButton.h>
#include <walrus_firmware_msgs/DiagnosticScreen.h>

class View
{
protected:
    int height;
    int width;
    int row;
    int column;

public:
    View(int height, int width, int row, int column) 
    : height(height), width(width), row(row), column(column) {}
    View(int row, int column)
    : height(0), width(0), row(row), column(column) {}
    View() 
    : height(0), width(0), row(0), column(0) {}
    virtual void render(char* characters) = 0;
    virtual void performAction(int action) {}
    virtual int getHeight() {return height; }
    virtual int getWidth() {return width; }
    virtual int getRow() {return row; }
    virtual int getColumn() {return column; }
    virtual int getRenderSize() {return height*width; }
};

#endif
