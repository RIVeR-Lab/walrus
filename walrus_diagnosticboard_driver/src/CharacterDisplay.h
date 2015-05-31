#ifndef CHARACTERDISPLAY_H
#define CHARACTERDISPLAY_H

#include <string>
#include <vector>

enum Justification {LEFT, CENTER, RIGHT};


class CharacterDisplay
{
private:
    int width;
    int height;
    vector<char*> lines;
    char* newLine()
    {
        char* line = new char[width]; 
        lines.push_back(line);
        return line;
    }

public:
    CharacterDisplay(int width, int height)
    : width(width), height(height)
    { 
        lines = new vector<char*>();
    }
    ~CharacterDisplay()
    {
        for (int l = 0; l < lines.size(); l++)
            delete[] lines[l];
    }
    
    static string truncThenConcat(string start, string middle, string end="")
    {
        int total_len = middle.length() + start.length() + end.length();
        if (total_len > width)
        {
            string new_middle = middle.substr(0, width-(start.length()+end.length()));
            return start + new_middle + end;
        }
        else
            return start + middle + end;
    }
    
    void writeBlock(string s, Justification justify = LEFT)
    {
    }
    
    void writeLine(string s, Justification justify = LEFT)
    {
        string trunc_s = substr(0, width);
        
        char* line = newLine();
        
    }
     

}


#endif
