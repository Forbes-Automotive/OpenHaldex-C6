#pragma once

#include <OpenHaldexC6_defs.h>

void setupAPI();
// Function declarations only (implementations in .cpp)
void setupWebServer();
extern AsyncWebServer webServer;