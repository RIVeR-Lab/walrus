#ifndef VIEWCONTAINER_H
#define VIEWCONTAINER_H

#include "View.h"

class ViewContainer : public View
{
protected:
    vector<View*> children;

public:
    ViewContainer(int height, int width, int row, int column) 
    : View(height, width, row, column), children()
    {
    }
    ViewContainer(int height, int width, int row, int column, View** views, int num_views)
    : View(height, width, row, column), children()
    {
        for (int l = 0; l < num_views; l++)
            addChild(views[l]);
    }

    void addChild(View* child)
    {
        children.push_back(child);
    }
    
    virtual void render(char* characters)
    {
        memset(characters, ' ', width*height);
        for (int l = 0; l < children.size(); l++)
        {
            if (children[l]->getRow()+children[l]->getHeight() <= height && children[l]->getColumn()+children[l]->getWidth() <= width)
            {
                int size = children[l]->getWidth()*children[l]->getHeight();
                char* child_area = new char[size];
                children[l]->render(child_area);
                
                for (int m = 0; m < children[l]->getHeight(); m++)             
                        memcpy(characters+((children[l]->getRow()+m)*width)+children[l]->getColumn(), child_area+(m*children[l]->getWidth()), children[l]->getWidth());               
                
                delete[] child_area;
            }
        }
    }

};

#endif
