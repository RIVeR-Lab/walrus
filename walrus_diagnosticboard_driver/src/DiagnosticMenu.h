
#ifndef DIAGNOSTICMENU_H
#define DIAGNOSTICMENU_H

#include <string.h>
#include "MenuScreen.h"
#include "MenuItem.h"
#include "Screen.h"
#include <vector.h>


class DiagnosticMenu : public MenuScreen
{
private:
    DiagnosticArray data;
    string path;

public:
    DiagnosticMenu(DiagnosticArray data, string path, Screen* previous);
    

}

#endif

