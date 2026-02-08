#ifndef OPENHALDEXC6_IO_H
#define OPENHALDEXC6_IO_H

#include <OpenHaldexC6_defs.h>

void setupIO();
void setupTasks();
void updateTriggers(void *arg);
void showHaldexState(void *arg);
void frames10(void *arg);
void frames20(void *arg);
void frames25(void *arg);
void frames100(void *arg);
void frames200(void *arg);
void frames1000(void *arg);

#endif // OPENHALDEXC6_IO_H
