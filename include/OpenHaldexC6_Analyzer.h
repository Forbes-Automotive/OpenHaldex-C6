#pragma once

#include <OpenHaldexC6_defs.h>

void setupAnalyzer();
void setAnalyzerMode(bool enable);
void setAnalyzerSerialMode(bool enable);
void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus);