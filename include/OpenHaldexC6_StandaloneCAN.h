#pragma once

#include <OpenHaldexC6_defs.h>

// Periodic frame tasks (dispatcher tasks per generation)
void frames10(void *arg);
void frames20(void *arg);
void frames25(void *arg);
void frames100(void *arg);
void frames200(void *arg);
void frames1000(void *arg);

// Gen1 standalone frames
void Gen1_frames10();
void Gen1_frames20();
void Gen1_frames25();
void Gen1_frames100();
void Gen1_frames200();
void Gen1_frames1000();

// Gen2 standalone frames
void Gen2_frames10();
void Gen2_frames20();
void Gen2_frames25();
void Gen2_frames100();
void Gen2_frames200();
void Gen2_frames1000();

// Gen4 standalone frames
void Gen4_frames10();
void Gen4_frames20();
void Gen4_frames25();
void Gen4_frames100();
void Gen4_frames200();
void Gen4_frames1000();

// Gen5 standalone frames
void Gen5_frames10();
void Gen5_frames20();
void Gen5_frames25();
void Gen5_frames100();
void Gen5_frames200();
void Gen5_frames1000();