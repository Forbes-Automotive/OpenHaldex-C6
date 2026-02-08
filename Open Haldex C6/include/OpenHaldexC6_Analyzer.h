#ifndef OPENHALDEXC6_ANALYZER_H
#define OPENHALDEXC6_ANALYZER_H

#include <OpenHaldexC6_defs.h>

void setupAnalyzer();
void setAnalyzerMode(bool enable);
void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus);

#endif // OPENHALDEXC6_ANALYZER_H
