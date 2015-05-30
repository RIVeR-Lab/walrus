
#ifndef VIEW_BASE_H
#define VIEW_BASE_H

#include <stdlib>

namespace diagnostic_peripheral_base
{
  /*
    A base class for developing peripheral character display views
  */
  class ViewBase
  {
  public:
    ViewBase(int height, int width, int row, int column)
      : height(height), width(width), row(row), column(column) {}

    virtual void render(char* frame)
    {
      memset(frame, ' ', width*height);
      frame[0] = 'X';
      frame[width-1] = 'X';
      frame[width*(height-1)] = 'X';
      frame[(width*height)-1] = 'X';
    }

    virtual int getHeight() { return height; }
    virtual int getWidth() { return width; }
    virtual int getRow() { return row; }
    virtual int getColumn() { return column; }

  protected:
    int height;
    int width;
    int row;
    int column;

  };
}

#endif
