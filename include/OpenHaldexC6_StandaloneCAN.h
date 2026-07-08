#pragma once

#include <OpenHaldexC6_defs.h>

// Periodic frame tasks (dispatcher tasks per generation)
void frames10(void *arg);
void frames13(void *arg);
void frames20(void *arg);
void frames25(void *arg);
void frames50(void *arg);
void frames100(void *arg);
void frames200(void *arg);
void frames250(void *arg);
void frames1000(void *arg);
void gen41DualBusRatesTask(void *arg);

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

// Gen4.1 (GM/SAAB) standalone frames
void Gen41_frames10();
void Gen41_frames13();
void Gen41_frames20();
void Gen41_frames25();
void Gen41_frames50();
void Gen41_frames100();
void Gen41_frames200();
void Gen41_frames250();
void Gen41_frames1000();

// Gen5 (0CQ) standalone frames - MQB Gen5 controller
void Gen5_0CQ_frames10();
void Gen5_0CQ_frames20();
void Gen5_0CQ_frames25();
void Gen5_0CQ_frames100();
void Gen5_0CQ_frames200();
void Gen5_0CQ_frames1000();

// Gen5 (0AY) standalone frames - 0AY controller (starter copy of Gen4 standalone)
void Gen5_0AY_frames10();
void Gen5_0AY_frames20();
void Gen5_0AY_frames25();
void Gen5_0AY_frames100();
void Gen5_0AY_frames200();
void Gen5_0AY_frames1000();

// Gen42 (Ford) standalone frames
void Gen42_frames10();
void Gen42_frames20();
void Gen42_frames100();
void Gen42_frames200();
void Gen42_frames1000();