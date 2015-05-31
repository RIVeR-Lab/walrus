#include "StatusScreen.h"


char* initNewLine()
{
    char* new_line = new char[SCREEN_WIDTH];
    for (int l = 0; l < SCREEN_WIDTH; l++)
        new_line[l] = ' ';
    return new_line;
}

bool getNextWordLength(string str, int pos, int* len)
{
    int count = 0;
    while (true)
    {
        if (str[pos] == string::npos)
        {
            (*len) = count;
            return true;
        }
        else if (str[pos] == ' ')
        {
            (*len) = count;
            return false;
         }
         count++;
         pos++;
    }
}


void StatusScreen::renderStatusData()
{
    int old_num_lines = lines->size();
    for (int l = 0; l < linex->size(); l++)
        delete[] lines[l];
    delete lines;
    
    lines = new vector<char*>();
    char* currentline;
    int word_len;
    bool last;
    int line_ptr;
    int str_ptr;
    
    //Print Name
    currentline = initNewLine();
    line_ptr = str_ptr = 0;
    currentline[char_ptr++] = '[';    
    while (true) //Loop forever
    {
        if (name[str_ptr] == ' ') //If the next character is a space, check if the next word will fit on the current line
        {
            last = getNextWordLength(name, str_ptr, &word_len); //Get the length of the next word and wheter it is the last word
            if (last) //If its the last word make sure to leave room for the closing ']], it would look dumb if thats all that was on a line
            {
                if (word_len > SCREEN_WIDTH-line_ptr && word_len <= 19) //If its larger than the line length, there's no point it putting in on a new line
                {
                    lines->push_back(currentline);
                    currentline = initNewLine();
                    line_ptr = 0;
                    str_ptr++;
                }
            }
            else
            {
                if (word_len > SCREEN_WIDTH-line_ptr+1 && word_len <= 20) //If it's larger than the line length, there's no point it putting in on a new line
                {
                    lines->push_back(currentline);
                    currentline = initNewLine();
                    line_ptr = 0;
                    str_ptr++;
                }
            }
        }
        else if (name[str_ptr] == '\0') //End of string detected
        {
            currentline[line_ptr] = ']';
            lines->push_back(currentline);
            break;
        }
        currentline[line_ptr++] = name[str_ptr++];
        if (line_ptr == SCREEN_WIDTH)  //Hit the end of the line, this should only happen if there are words longer than SCREEN_WIDTH
        {         
            lines->push_back(currentline);
            if (name[str_ptr] == '\0') //If we just wrote the last string character, exit (this will result in no ']' at the end and should only happen if the first and only word is exactly 19 characters, which will look dumb with a ']' no mater how it's done
                break;
            currentline = initNewLine();
            line_ptr = 0;
        }
    }
    
    //Print Status
    currentline = initNewLine();
    memcpy(currentline, "Status:", 7);
    if (level == OK)
        memcpy(currentline+SCREEN_WIDTH-2, "OK", 2);
    else if (level == WARN)
        memcpy(currentline+SCREEN_WIDTH-4, "WARN", 4);
    else if (level == ERROR)
        memcpy(currentline+SCREEN_WIDTH-5, "ERROR", 5);
    else if (level == STALE)
        memcpy(currentline+SCREEN_WIDTH-5, "STALE", 5);
    else
        currentline[SCREEN_WIDTH-1] = '?';
    lines->push_back(currentline);
        
    
    //Print Message
    currentline = initNewLine();  
    while (true) //Loop forever
    {
        if (name[str_ptr] == ' ') //If the next character is a space, check if the next word will fit on the current line
        {
            getNextWordLength(name, str_ptr, &word_len); //Get the length of the next word and wheter it is the last word
            if (word_len > SCREEN_WIDTH-line_ptr+1 && word_len <= 20) //If it's larger than the line length, there's no point it putting in on a new line
            {
                lines->push_back(currentline);
                currentline = initNewLine();
                line_ptr = 0;
                str_ptr++;
            }
        }
        else if (name[str_ptr] == '\0') //End of string detected
        {
            lines->push_back(currentline);
            break;
        }
        currentline[line_ptr++] = name[str_ptr++];
        if (line_ptr == SCREEN_WIDTH) 
        {         
            lines->push_back(currentline);
            if (name[str_ptr] == '\0')
                break;
            currentline = initNewLine();
            line_ptr = 0;
        }
    }
    
    
    //Print data
    for (std::map<string, string>::iterator it=mymap.begin(); it!=mymap.end(); ++it)
    {
        
    }
    
}

