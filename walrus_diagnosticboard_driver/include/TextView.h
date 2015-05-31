#ifndef TEXTVIEW_H
#define TEXTVIEW_H

#include "View.h"
#include <string>
#include <sstream>

using namespace std;

enum Justify {LEFT, CENTER, RIGHT};

class TextView : public View
{
protected:
    string prefix;
    string suffix;
    string text;
    bool allow_split_words;
    Justify justification;
    int max_height;

    int required_height()
    {
        int tl = total_length();
        if (tl % width == 0)
            return tl/width;
        else
            return (tl/width)+1;
    }
    
    int total_length()
    {
        return prefix.length() + text.length() + suffix.length();
    }    
    
public:
    TextView(int height, int width, int row, int column)
    : View(height, width, row, column), max_height(0), prefix(""), text(""), suffix("")
    {
    }
    TextView(int width, int row, int column)
    : View(0, width, row, column), max_height(0), prefix(""), text(""), suffix("")
    {
    }
    TextView(int height, int width, int row, int column, string prefix, string text, string suffix)
    : View(height, width, row, column), max_height(0), prefix(prefix), text(text), suffix(suffix)
    {
    }
    TextView(int width, int row, int column, string prefix, string text, string suffix)
    : View(0, width, row, column), max_height(0), prefix(prefix), text(text), suffix(suffix)
    {
    }
    TextView(int height, int width, int row, int column, string prefix, string text)
    : View(height, width, row, column), max_height(0), prefix(prefix), text(text), suffix("")
    {
    }
    TextView(int width, int row, int column, string prefix, string text)
    : View(0, width, row, column), max_height(0), prefix(prefix), text(text), suffix("")
    {
    }
    TextView(int height, int width, int row, int column, string text)
    : View(height, width, row, column), max_height(0), prefix(""), text(text), suffix("")
    {
    }
    TextView(int width, int row, int column, string text)
    : View(0, width, row, column), max_height(0), prefix(""), text(text), suffix("")
    {
    }
    
    void setFixedHeight(int h)
    {
        height = h;
    }
    
    void setMaxHeight(int h)
    {
        height = 0;
        max_height = h;
    }

    void setPrefix(string txt)
    {
        prefix = txt;
    }
    
    void setSuffix(string txt)
    {
        suffix = txt;
    }
    
    void setText(string txt)
    {
        text = txt;
    }
    
    void setText(double d, int precision = 0)
    {
        stringstream ss;
        ss << std::fixed << std::setprecision(precision) << d;
        text = ss.str();
    }
    
    void setText(int i)
    {
        stringstream ss;
        ss << i;
        text = ss.str();
    }
    
    //Not even close to thread safe, make sure no one tries to update text between a call to getHeight and Render or bad stuff
    virtual int getHeight()
    {
        ROS_INFO("OVERRIDE");
        if (height == 0)
        {
            if (max_height == 0 || required_height() < max_height)
                return required_height();
            else   
                return max_height;
        }
        else
            return height;
    }
    
    virtual void render(char* characters)
    {
        int actual_height = getHeight();
        int len = actual_height * width;
        int text_len;
        int text_len_max = len-prefix.length()-suffix.length();
        if (text.length() > text_len_max)
            text_len = text_len_max;
        else
            text_len = text.length();
        int ptr = 0;
        
        for (int l = 0; l < prefix.length() && ptr < len; l++)
            characters[ptr++] = prefix[l];
        
        for (int l = 0; l < text_len; l++)
            characters[ptr++] = text[l];

        for (int l = 0; l < suffix.length() && ptr < len; l++)
            characters[ptr++] = suffix[l];
       
        while (ptr < len)
            characters[ptr++] = ' ';      
    }
    
    


};

#endif
