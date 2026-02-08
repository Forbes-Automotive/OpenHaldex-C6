#include <OpenHaldexC6_Analyzer.h>

void setupAnalyzer() {
  // Stub implementation
}

void setAnalyzerMode(bool enable) {
  analyzerMode = enable;
  if (analyzerMode) {
    if (!disableController) {
      disableController = true;
    }
  }
}

void analyzerQueueFrame(const twai_message_t &frame, uint8_t bus) {
  // Stub implementation
}
