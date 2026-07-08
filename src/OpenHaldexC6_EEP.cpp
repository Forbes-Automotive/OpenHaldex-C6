#include <OpenHaldexC6_EEP.h>          // include the header for EEPROM/preference functions
#include <OpenHaldexC6_Calculations.h> // for haldexLearnTable and learn globals

void readEEP() // function to read stored preferences into runtime variables
{              // start readEEP
#if detailedDebugEEP
  DEBUG("EEPROM initialising!"); // debug: EEPROM init start
#endif

  // use ESP32's 'Preferences' to remember settings
  pref.begin("broadcastOpen", false);   // open CAN broadcast preference
  pref.begin("isStandalone", false);    // standalone mode preference
  pref.begin("useCANifAvail", false);   // use CAN if available preference
  pref.begin("disableControl", false);  // controller disable preference
  pref.begin("followBrake", false);     // follow brake input preference
  pref.begin("followHandbrake", false); // follow handbrake input preference
  pref.begin("invertBrake", false);     // invert brake output preference
  pref.begin("invertHandbrake", false); // invert handbrake output preference
  pref.begin("tcForceMode", false);     // TC force mode preference
  pref.begin("extBtnForceMode", false); // external button force mode
  pref.begin("hazardForceMode", false); // hazard lights force mode
  pref.begin("dsbOnboardBtn", false);   // disable onboard button preference
  pref.begin("dsbExtBtn", false);       // disable external button preference
  pref.begin("canSleepEn", false);      // CAN-wake light sleep enable preference
  pref.begin("ledBrightness", false);   // LED brightness preference

  pref.begin("haldexGen", false);       // stored haldex generation
  pref.begin("forceModeValue", false);  // stored force mode value
  pref.begin("lastMode", false);        // stored last mode
  pref.begin("disableThrottle", false); // stored throttle disable value
  pref.begin("disengageUSpeed", false); // stored disengage under speed
  pref.begin("disengageASpeed", false); // stored disengage above speed
  pref.begin("otaUpdate", false);       // stored OTA update flag

  pref.begin("throttleArray", false); // stored throttle curve bytes
  pref.begin("speedArray", false);    // stored speed curve bytes
  pref.begin("lockArray", false);     // stored lock curve bytes
  pref.begin("learnTable", false);    // stored haldex learn table
  pref.begin("wifiPwd", false);       // stored WiFi AP password
  pref.begin("udsMQBEn", false);      // UDS MQB polling enable preference

  // first run comes with EEP value of 255, so write actual values
  if (pref.getUInt("haldexGeneration") == 255) // detect first run
  {                                            // if first run
#if detailedDebugEEPF
    DEBUG("First run..."); // debug: first run detected
#endif
    pref.putBool("broadcastOpen", broadcastOpenHaldexOverCAN); // save broadcast setting
    pref.putBool("isStandalone", isStandalone);                // save standalone setting
    pref.putBool("useCANifAvail", useCANifAvailable);          // save use CAN if available setting
    pref.putBool("disableControl", disableController);         // save controller disable
    pref.putBool("followBrake", followBrake);                  // save follow brake
    pref.putBool("followHandbrake", followHandbrake);          // save follow handbrake
    pref.putBool("invertBrake", invertBrake);                  // save invert brake
    pref.putBool("invertHandbrake", invertHandbrake);          // save invert handbrake
    pref.putBool("tcForceMode", tcForceMode);                  // save tc force mode
    pref.putBool("extBtnForceMode", extBtnForceMode);          // save ext button force mode
    pref.putBool("hazardForceMode", hazardForceMode);          // save hazard force mode
    pref.putBool("dsbOnboardBtn", disableOnboardButton);       // save disable onboard button
    pref.putBool("dsbExtBtn", disableExternalButton);          // save disable external button
    pref.putBool("fixHunting", fixHunting);                    // save Motor_11 BPK-mode toggle
    pref.putBool("canSleepEn", canSleepEnabled);               // save CAN-wake light sleep enable
    pref.putBool("canSleepAggr", canSleepAggressive);          // save aggressive CAN sleep enable
    pref.putUShort("lpWakeFps", lpWakeThresholdFps);           // save LP wake threshold (fps)
    pref.putUChar("ledBrightness", ledBrightness);             // save LED brightness

    pref.putBool("otaUpdate", otaUpdate);                                            // save OTA update flag
    pref.putUChar("haldexGen", haldexGeneration);                                    // save haldex generation
    pref.putUChar("tcFMV", tcForceModeValue);                                        // save TC force mode value
    pref.putUChar("hazFMV", hazardForceModeValue);                                   // save Hazard force mode value
    pref.putUChar("extFMV", extBtnForceModeValue);                                   // save ExtBtn force mode value
    pref.putUChar("lastMode", lastMode);                                             // save last used mode
    pref.putUChar("disableThrottle", disableThrottle);                               // save throttle disable
    pref.putUShort("disengageUSpeed", disengageUnderSpeed);                          // save disengage under speed
    pref.putUShort("disengageASpeed", disengageAboveSpeed);                          // save disengage above speed
    pref.putBytes("speedArray", (byte *)(&speedArray), sizeof(speedArray));          // save speed array bytes
    pref.putBytes("throttleArray", (byte *)(&throttleArray), sizeof(throttleArray)); // save throttle array bytes
    pref.putBytes("lockArray", (byte *)(&lockArray), sizeof(lockArray));             // save lock array bytes
    pref.putBool("learnOK", false);                                                  // learn table not valid on first run
    pref.putString("wifiPwd", wifiPassword);                                         // save WiFi password (empty = open network)
    pref.putString("wifiSsid", wifiSsid);                                            // save WiFi SSID (factory default on first run)
    pref.putBool("udsMQBEn", udsMQBEnabled);                                         // save UDS MQB polling enable
    pref.putUChar("forceModesPrio", forceModesPriority);                             // save force-modes priority order
    pref.putFloat("lockReleaseRate", lockReleaseRatePerSec);                         // save lock release rate (%/s)
    pref.putBool("lockReleaseEn", lockReleaseEnabled);                               // save lock release enable
  } // end if first run
  else // normal run: load stored values
  {
    broadcastOpenHaldexOverCAN = pref.getBool("broadcastOpen", false);      // load broadcast setting
    isStandalone = pref.getBool("isStandalone", false);                     // load standalone setting
    useCANifAvailable = pref.getBool("useCANifAvail", false);               // load use CAN if available setting
    disableController = pref.getBool("disableControl", false);              // load controller disable
    followBrake = pref.getBool("followBrake", false);                       // load follow brake
    followHandbrake = pref.getBool("followHandbrake", false);               // load follow handbrake
    invertBrake = pref.getBool("invertBrake", false);                       // load invert brake
    invertHandbrake = pref.getBool("invertHandbrake", false);               // load invert handbrake
    tcForceMode = pref.getBool("tcForceMode", false);                       // load tc force mode
    extBtnForceMode = pref.getBool("extBtnForceMode", false);               // load ext button force mode
    hazardForceMode = pref.getBool("hazardForceMode", false);               // load hazard force mode
    disableOnboardButton = pref.getBool("dsbOnboardBtn", false);            // load disable onboard button
    disableExternalButton = pref.getBool("dsbExtBtn", false);               // load disable external button
    fixHunting = pref.getBool("fixHunting", false);                         // load Motor_11 BPK-mode toggle
    canSleepEnabled = pref.getBool("canSleepEn", true);                     // load CAN-wake light sleep enable
    canSleepAggressive = pref.getBool("canSleepAggr", false);               // load aggressive CAN sleep enable
    lpWakeThresholdFps = pref.getUShort("lpWakeFps", 1100);                  // load LP wake threshold (fps)
    ledBrightness = pref.getUChar("ledBrightness", led_brightness_default); // load LED brightness

    otaUpdate = pref.getBool("otaUpdate", false);                          // load OTA update flag
    haldexGeneration = pref.getUChar("haldexGen", 1);                      // load haldex generation with default
    if (haldexGeneration == 5) haldexGeneration = 50;                      // migrate legacy Gen5 -> Gen5 (0CQ)
    tcForceModeValue = pref.getUChar("tcFMV", 2);                          // load TC force mode value (default 50:50)
    hazardForceModeValue = pref.getUChar("hazFMV", 2);                     // load Hazard force mode value
    extBtnForceModeValue = pref.getUChar("extFMV", 2);                     // load ExtBtn force mode value
    lastMode = pref.getUChar("lastMode", 0);                               // load last mode with default
    disableThrottle = pref.getUChar("disableThrottle", 0);                 // load throttle disable with default
    state.pedal_threshold = disableThrottle;                               // apply throttle disable to state
    disengageUnderSpeed = pref.getUShort("disengageUSpeed", 0);            // load disengage under speed
    disengageAboveSpeed = pref.getUShort("disengageASpeed", 0);            // load disengage above speed
    pref.getBytes("speedArray", &speedArray, sizeof(speedArray));          // read speed array bytes
    pref.getBytes("throttleArray", &throttleArray, sizeof(throttleArray)); // read throttle array bytes
    pref.getBytes("lockArray", &lockArray, sizeof(lockArray));             // read lock array bytes
    haldexLearnTableValid = pref.getBool("learnOK", false);                // read learn table valid flag
    if (haldexLearnTableValid)
    {
      pref.getBytes("learnTbl", haldexLearnTable, sizeof(haldexLearnTable)); // read learn table bytes
    }
    pref.getString("wifiPwd", wifiPassword, sizeof(wifiPassword)); // load WiFi AP password
    pref.getString("wifiSsid", wifiSsid, sizeof(wifiSsid));        // load WiFi AP SSID
    if (wifiSsid[0] == '\0')                                       // legacy installs: missing key
    {
      strncpy(wifiSsid, wifiHostNameDefault, sizeof(wifiSsid) - 1); // restore factory default
    }
    udsMQBEnabled = pref.getBool("udsMQBEn", false);               // load UDS MQB polling enable
    forceModesPriority = pref.getUChar("forceModesPrio", 0);       // load force-modes priority (0=TC>Haz>Ext)
    lockReleaseRatePerSec = pref.getFloat("lockReleaseRate", 120.0f); // load lock release rate (%/s)
    lockReleaseEnabled = pref.getBool("lockReleaseEn", true);        // load lock release enable

    switch (lastMode) // map stored lastMode to runtime enum
    {
    case 0:
      state.mode = MODE_STOCK;  // set MODE_STOCK
      break;
    case 1:
      state.mode = MODE_FWD;    // set MODE_FWD
      break;
    case 2:
      state.mode = MODE_5050;   // set MODE_5050
      break;
    case 3:
      state.mode = MODE_6040;   // set MODE_6040
      break;
    case 4:
      state.mode = MODE_7525;   // set MODE_7525
      break;
    case 5:
      state.mode = MODE_EXPERT; // set MODE_EXPERT
      break;
    default:                    // unrecognized value
      state.mode = MODE_FWD;    // default to MODE_FWD
      break;
    } 
  }

#if detailedDebugEEP
  DEBUG("EEPROM initialised with...");                                                           // debug: print loaded prefs
  DEBUG("    Broadcast OpenHaldex over CAN: %s", broadcastOpenHaldexOverCAN ? "true" : "false"); // debug broadcast
  DEBUG("    Standalone mode: %s", isStandalone ? "true" : "false");                             // debug standalone
  DEBUG("    Follow handbrake: %s", followHandbrake ? "true" : "false");                         // debug handbrake
  DEBUG("    Follow brake: %s", followBrake ? "true" : "false");                                 // debug brake
  DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");                         // debug invert handbrake
  DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");                                 // debug invert brake
  DEBUG("    tcForceMode: %s", tcForceMode ? "true" : "false");                                  // debug tc force mode
  DEBUG("    extBtnForceMode: %s", extBtnForceMode ? "true" : "false");                          // debug ext btn force

  DEBUG("    Haldex Generation: %d", haldexGeneration); // debug haldex gen
  DEBUG("    Force Mode TC/Haz/Ext: %d/%d/%d", tcForceModeValue, hazardForceModeValue, extBtnForceModeValue);
  DEBUG("    Last Mode: %d", lastMode);                      // debug last mode
  DEBUG("    Disable Under Speed: %d", disengageUnderSpeed); // debug disengage under speed
  DEBUG("    Disable Above Speed: %d", disengageAboveSpeed); // debug disengage above speed
  DEBUG("    System Update on Reboot: %d", otaUpdate);       // debug ota update flag
#endif
} 

void writeEEP(void *arg) // task function to periodically write preferences
{                        // start writeEEP
  while (1)              // loop forever in task
  {
    stackwriteEEP = uxTaskGetStackHighWaterMark(NULL); // record stack high watermark

#if detailedDebugEEP
    DEBUG("Writing EEPROM..."); // debug: writing prefs
#endif

    // update EEP only if changes have been made
    pref.putBool("broadcastOpen", broadcastOpenHaldexOverCAN); // write broadcast setting
    pref.putBool("isStandalone", isStandalone);                // write standalone setting
    pref.putBool("useCANifAvail", useCANifAvailable);          // write use CAN if available setting
    pref.putBool("disableControl", disableController);         // write controller disable
    pref.putBool("followBrake", followBrake);                  // write follow brake
    pref.putBool("followHandbrake", followHandbrake);          // write follow handbrake
    pref.putBool("invertBrake", invertBrake);                  // write invert brake
    pref.putBool("invertHandbrake", invertHandbrake);          // write invert handbrake
    pref.putBool("tcForceMode", tcForceMode);                  // write tc force mode
    pref.putBool("extBtnForceMode", extBtnForceMode);          // write ext button force mode
    pref.putBool("hazardForceMode", hazardForceMode);          // write hazard force mode
    pref.putBool("dsbOnboardBtn", disableOnboardButton);       // write disable onboard button
    pref.putBool("dsbExtBtn", disableExternalButton);          // write disable external button
    pref.putBool("fixHunting", fixHunting);                    // write Motor_11 BPK-mode toggle
    pref.putBool("canSleepEn", canSleepEnabled);               // write CAN-wake light sleep enable
    pref.putBool("canSleepAggr", canSleepAggressive);          // write aggressive CAN sleep enable
    pref.putUShort("lpWakeFps", lpWakeThresholdFps);           // write LP wake threshold (fps)
    pref.putUChar("ledBrightness", ledBrightness);             // write LED brightness

    pref.putUChar("haldexGen", haldexGeneration);                                    // write haldex generation
    pref.putUChar("tcFMV", tcForceModeValue);                                        // write TC force mode value
    pref.putUChar("hazFMV", hazardForceModeValue);                                   // write Hazard force mode value
    pref.putUChar("extFMV", extBtnForceModeValue);                                   // write ExtBtn force mode value
    pref.putUChar("lastMode", lastMode);                                             // write last mode
    pref.putUChar("disableThrottle", disableThrottle);                               // write throttle disable
    pref.putUShort("disengageUSpeed", disengageUnderSpeed);                          // write disengage under speed
    pref.putUShort("disengageASpeed", disengageAboveSpeed);                          // write disengage above speed
    pref.putBytes("speedArray", (byte *)(&speedArray), sizeof(speedArray));          // write speed array
    pref.putBytes("throttleArray", (byte *)(&throttleArray), sizeof(throttleArray)); // write throttle array
    pref.putBytes("lockArray", (byte *)(&lockArray), sizeof(lockArray));             // write lock array
    pref.putBool("learnOK", haldexLearnTableValid);                                  // write learn valid flag
    if (haldexLearnTableValid)
    {
      pref.putBytes("learnTbl", haldexLearnTable, sizeof(haldexLearnTable)); // write learn table bytes
    }
    pref.putString("wifiSsid", wifiSsid);    // write WiFi AP SSID
    pref.putString("wifiPwd", wifiPassword); // write WiFi AP password
    pref.putBool("udsMQBEn", udsMQBEnabled); // write UDS MQB polling enable
    pref.putUChar("forceModesPrio", forceModesPriority);      // write force-modes priority order
    pref.putFloat("lockReleaseRate", lockReleaseRatePerSec);  // write lock release rate (%/s)
    pref.putBool("lockReleaseEn", lockReleaseEnabled);        // write lock release enable

#if detailedDebugEEP
    DEBUG("Written EEPROM with data:");                                                            // debug: print written prefs
    DEBUG("    Broadcast OpenHaldex over CAN: %s", broadcastOpenHaldexOverCAN ? "true" : "false"); // debug broadcast
    DEBUG("    Standalone mode: %s", isStandalone ? "true" : "false");                             // debug standalone
    DEBUG("    Follow handbrake: %s", followHandbrake ? "true" : "false");                         // debug handbrake
    DEBUG("    Follow brake: %s", followBrake ? "true" : "false");                                 // debug brake
    DEBUG("    Invert handbrake: %s", invertHandbrake ? "true" : "false");                         // debug invert handbrake
    DEBUG("    Invert brake: %s", invertBrake ? "true" : "false");                                 // debug invert brake
    DEBUG("    Haldex Generation: %d", haldexGeneration);                                          // debug haldex gen
    DEBUG("    tcForceMode: %s", tcForceMode ? "true" : "false");                                  // debug tc force mode
    DEBUG("    extBtnForceMode: %s", extBtnForceMode ? "true" : "false");                          // debug ext btn force
    DEBUG("    Force Mode TC/Haz/Ext: %d/%d/%d", tcForceModeValue, hazardForceModeValue, extBtnForceModeValue);
    DEBUG("    Last Mode: %d", lastMode);                      // debug last mode
    DEBUG("    Disable Below Throttle: %d", disableThrottle);  // debug disable throttle
    DEBUG("    Disable Under Speed: %d", disengageUnderSpeed); // debug disengage under speed
    DEBUG("    Disable Above Speed: %d", disengageAboveSpeed); // debug disengage above speed
#endif

    vTaskDelay(eepRefresh / portTICK_PERIOD_MS); // wait before next write
  } // end while
} // end writeEEP
