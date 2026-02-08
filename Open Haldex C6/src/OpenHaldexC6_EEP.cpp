#include <OpenHaldexC6_EEP.h>

void readEEP() {
#if detailedDebugEEP
  DEBUG("EEPROM initialising!");
#endif

  // use ESP32's 'Preferences' to remember settings
  pref.begin("broadcastOpen", false);
  pref.begin("isStandalone", false);
  pref.begin("disableControl", false);
  pref.begin("followBrake", false);
  pref.begin("followHandbrake", false);
  pref.begin("invertBrake", false);
  pref.begin("invertHandbrake", false);
  pref.begin("haldexGen", false);
  pref.begin("lastMode", false);
  pref.begin("disableThrottle", false);
  pref.begin("disableSpeed", false);
  pref.begin("otaUpdate", false);
  pref.begin("customSpeed", false);
  pref.begin("customThrottle", false);
  pref.begin("throttleArray", false);
  pref.begin("speedArray", false);
  pref.begin("lockArray", false);

  // first run comes with EEP value of 255, so write actual values
  if (pref.getUInt("haldexGeneration") == 255) {
#if detailedDebugEEP
    DEBUG("First run...");
#endif
    pref.putBool("broadcastOpen", broadcastOpenHaldexOverCAN);
    pref.putBool("isStandalone", isStandalone);
    pref.putBool("disableControl", disableController);
    pref.putBool("followBrake", followBrake);
    pref.putBool("followHandbrake", followHandbrake);
    pref.putBool("invertBrake", invertBrake);
    pref.putBool("invertHandbrake", invertHandbrake);
    pref.putBool("otaUpdate", otaUpdate);
    pref.putBool("customSpeed", customSpeed);
    pref.putBool("customThrottle", customThrottle);
    pref.putUChar("haldexGen", haldexGeneration);
    pref.putUChar("lastMode", lastMode);
    pref.putUChar("disableThrottle", disableThrottle);
    pref.putUShort("disableSpeed", disableSpeed);
    pref.putBytes("speedArray", (byte *)(&speedArray), sizeof(speedArray));
    pref.putBytes("throttleArray", (byte *)(&throttleArray), sizeof(throttleArray));
    pref.putBytes("lockArray", (byte *)(&lockArray), sizeof(lockArray));
  } else {
    broadcastOpenHaldexOverCAN = pref.getBool("broadcastOpen", false);
    isStandalone = pref.getBool("isStandalone", false);
    disableController = pref.getBool("disableControl", false);
    followBrake = pref.getBool("followBrake", false);
    followHandbrake = pref.getBool("followHandbrake", false);
    invertBrake = pref.getBool("invertBrake", false);
    invertHandbrake = pref.getBool("invertHandbrake", false);
    otaUpdate = pref.getBool("otaUpdate", false);
    customSpeed = pref.getBool("customSpeed", true);
    customThrottle = pref.getBool("customThrottle", false);
    haldexGeneration = pref.getUChar("haldexGen", 1);
    lastMode = pref.getUChar("lastMode", 0);
    disableThrottle = pref.getUChar("disableThrottle", 0);
    state.pedal_threshold = disableThrottle;
    disableSpeed = pref.getUShort("disableSpeed", 0);
    pref.getBytes("speedArray", &speedArray, sizeof(speedArray));
    pref.getBytes("throttleArray", &throttleArray, sizeof(throttleArray));
    pref.getBytes("lockArray", &lockArray, sizeof(lockArray));

    switch (lastMode) {
      case 0:
        state.mode = MODE_STOCK;
        break;
      case 1:
        state.mode = MODE_FWD;
        break;
      case 2:
        state.mode = MODE_5050;
        break;
      case 3:
        state.mode = MODE_6040;
        break;
      case 4:
        state.mode = MODE_7525;
        break;
      case 5:
        state.mode = MODE_CUSTOM;
        break;
      default:
        state.mode = MODE_FWD;
        break;
    }
  }

#if detailedDebugEEP
  DEBUG("EEPROM initialised with...");
  DEBUG("    Broadcast OpenHaldex over CAN: %s", broadcastOpenHaldexOverCAN ? "true" : "false");
  DEBUG("    Standalone mode: %s", isStandalone ? "true" : "false");
  DEBUG("    Follow handbrake: %s", followHandbrake ? "true" : "false");
  DEBUG("    Follow brake: %s", followBrake ? "true" : "false");
  DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");
  DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");
  DEBUG("    Haldex Generation: %d", haldexGeneration);
  DEBUG("    Last Mode: %d", lastMode);
  DEBUG("    Disable Below Throttle: %d", disableThrottle);
  DEBUG("    Disable Above Speed: %d", disableSpeed);
  DEBUG("    System Update on Reboot: %d", otaUpdate);
#endif
}

void writeEEP(void *arg) {
  while (1) {
    stackwriteEEP = uxTaskGetStackHighWaterMark(NULL);

#if detailedDebugEEP
    DEBUG("Writing EEPROM...");
#endif

    // update EEP only if changes have been made
    pref.putBool("broadcastOpen", broadcastOpenHaldexOverCAN);
    pref.putBool("isStandalone", isStandalone);
    pref.putBool("disableControl", disableController);
    pref.putBool("followBrake", followBrake);
    pref.putBool("followHandbrake", followHandbrake);
    pref.putBool("customSpeed", customSpeed);
    pref.putBool("customThrottle", customThrottle);
    pref.putBool("invertBrake", invertBrake);
    pref.putBool("invertHandbrake", invertHandbrake);
    pref.putUChar("haldexGen", haldexGeneration);
    pref.putUChar("lastMode", lastMode);
    pref.putUChar("disableThrottle", disableThrottle);
    pref.putUShort("disableSpeed", disableSpeed);
    pref.putBytes("speedArray", (byte *)(&speedArray), sizeof(speedArray));
    pref.putBytes("throttleArray", (byte *)(&throttleArray), sizeof(throttleArray));
    pref.putBytes("lockArray", (byte *)(&lockArray), sizeof(throttleArray));

#if detailedDebugEEP
    DEBUG("Written EEPROM with data:");
    DEBUG("    Broadcast OpenHaldex over CAN: %s", broadcastOpenHaldexOverCAN ? "true" : "false");
    DEBUG("    Standalone mode: %s", isStandalone ? "true" : "false");
    DEBUG("    Follow handbrake: %s", followHandbrake ? "true" : "false");
    DEBUG("    Follow brake: %s", followBrake ? "true" : "false");
    DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");
    DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");
    DEBUG("    Haldex Generation: %d", haldexGeneration);
    DEBUG("    Last Mode: %d", lastMode);
    DEBUG("    Disable Below Throttle: %d", disableThrottle);
    DEBUG("    Disable Above Speed: %d", disableSpeed);
#endif

    vTaskDelay(eepRefresh / portTICK_PERIOD_MS);
  }
}
