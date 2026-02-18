#include <OpenHaldexC6_EEP.h>

void readEEP()
{
#if detailedDebugEEP
  DEBUG("EEPROM initialising!");
#endif

  // use ESP32's 'Preferences' to remember settings
  pref.begin("broadcastOpen", false);   // bool - broadcast over CAN
  pref.begin("isStandalone", false);    // bool - is standalone mode
  pref.begin("disableControl", false);  // bool - disable controller
  pref.begin("followBrake", false);     // bool - follow brake signal
  pref.begin("followHandbrake", false); // bool - follow handbrake signal
  pref.begin("invertBrake", false);     // bool - invert brake output
  pref.begin("invertHandbrake", false); // bool - invert handbrake output
  pref.begin("tcForce5050", false);     // bool - use tc signal from can to force 5050
  pref.begin("extBtn5050", false);      // bool - use external button to force 5050

  pref.begin("haldexGen", false);       // int - haldex generation
  pref.begin("lastMode", false);        // int - last setting
  pref.begin("disableThrottle", false); // int - disable throttle
  pref.begin("disengageUSpeed", false); // int - disable under speed
  pref.begin("disengageASpeed", false); // int - disable above speed
  pref.begin("otaUpdate", false);       // has OTA update

  pref.begin("throttleArray", false); // bytes for throttle array
  pref.begin("speedArray", false);    // bytes for speed array
  pref.begin("lockArray", false);     // bytes for lock array

  // first run comes with EEP value of 255, so write actual values
  if (pref.getUInt("haldexGeneration") == 255)
  {
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
    pref.putBool("tcForce5050", tcForce5050);       // bool - use tc signal from can to force 5050
    pref.putBool("extBtn5050", extButtonForce5050); // bool - use external button to force 5050

    pref.putBool("otaUpdate", otaUpdate);
    pref.putUChar("haldexGen", haldexGeneration);
    pref.putUChar("lastMode", lastMode);
    pref.putUChar("disableThrottle", disableThrottle);
    pref.putUShort("disengageUSpeed", disengageUnderSpeed);
    pref.putUShort("disengageASpeed", disengageAboveSpeed);
    pref.putBytes("speedArray", (byte *)(&speedArray), sizeof(speedArray));
    pref.putBytes("throttleArray", (byte *)(&throttleArray), sizeof(throttleArray));
    pref.putBytes("lockArray", (byte *)(&lockArray), sizeof(lockArray));
  }
  else
  {
    broadcastOpenHaldexOverCAN = pref.getBool("broadcastOpen", false);
    isStandalone = pref.getBool("isStandalone", false);
    disableController = pref.getBool("disableControl", false);
    followBrake = pref.getBool("followBrake", false);
    followHandbrake = pref.getBool("followHandbrake", false);
    invertBrake = pref.getBool("invertBrake", false);
    invertHandbrake = pref.getBool("invertHandbrake", false);
    tcForce5050 = pref.getBool("tcForce5050", false);       // bool - use tc signal from can to force 5050
    extButtonForce5050 = pref.getBool("extBtn5050", false); // bool - use external button to force 5050

    otaUpdate = pref.getBool("otaUpdate", false);
    haldexGeneration = pref.getUChar("haldexGen", 1);
    lastMode = pref.getUChar("lastMode", 0);
    disableThrottle = pref.getUChar("disableThrottle", 0);
    state.pedal_threshold = disableThrottle;
    disengageUnderSpeed = pref.getUShort("disengageUSpeed", 0);
    disengageAboveSpeed = pref.getUShort("disengageASpeed", 0);
    pref.getBytes("speedArray", &speedArray, sizeof(speedArray));
    pref.getBytes("throttleArray", &throttleArray, sizeof(throttleArray));
    pref.getBytes("lockArray", &lockArray, sizeof(lockArray));

    switch (lastMode)
    {
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
      state.mode = MODE_EXPERT;
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
  DEBUG("    TC5050: %s", tcForce5050 ? "true" : "false");
  DEBUG("    extBtn5050: %s", extButtonForce5050 ? "true" : "false");

  DEBUG("    Haldex Generation: %d", haldexGeneration);
  DEBUG("    Last Mode: %d", lastMode);
  DEBUG("    Disable Under Speed: %d", disengageUnderSpeed);
  DEBUG("    Disable Above Speed: %d", disengageAboveSpeed);
  DEBUG("    System Update on Reboot: %d", otaUpdate);
#endif
}

void writeEEP(void *arg)
{
  while (1)
  {
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
    pref.putBool("invertBrake", invertBrake);
    pref.putBool("invertHandbrake", invertHandbrake);
    pref.putBool("tcForce5050", tcForce5050);       // bool - use tc signal from can to force 5050
    pref.putBool("extBtn5050", extButtonForce5050); // bool - use external button to force 5050

    pref.putUChar("haldexGen", haldexGeneration);
    pref.putUChar("lastMode", lastMode);
    pref.putUChar("disableThrottle", disableThrottle);
    pref.putUShort("disengageUSpeed", disengageUnderSpeed);
    pref.putUShort("disengageASpeed", disengageAboveSpeed);
    pref.putBytes("speedArray", (byte *)(&speedArray), sizeof(speedArray));
    pref.putBytes("throttleArray", (byte *)(&throttleArray), sizeof(throttleArray));
    pref.putBytes("lockArray", (byte *)(&lockArray), sizeof(lockArray));

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
    DEBUG("    Disable Under Speed: %d", disengageUnderSpeed);
    DEBUG("    Disable Above Speed: %d", disengageAboveSpeed);
#endif

    vTaskDelay(eepRefresh / portTICK_PERIOD_MS);
  }
}
