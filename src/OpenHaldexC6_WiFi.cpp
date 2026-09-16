#include <OpenHaldexC6_WiFi.h>
#include <cstring>

// Legacy WiFi implementation - now a stub
// All WiFi functionality has moved to OpenHaldexC6_WebServer.cpp

static void softAPStart()
{
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  if (strlen(wifiPassword) >= 8)
  {
    WiFi.softAP(wifiHostName, wifiPassword); // password-protected AP
    DEBUG("WiFi AP started with password: %s", wifiHostName);
  }
  else
  {
    WiFi.softAP(wifiHostName); // open network
    DEBUG("WiFi AP started (open): %s", wifiHostName);
  }
}

// Bridge mode: join a home/garage network as a station, in addition to the AP.
// Non-blocking - WiFi.begin() returns immediately, actual connection is picked up
// later by pollWifiSta() from the main loop. AP always keeps running regardless
// of whether the STA side connects, so nothing changes for anyone who doesn't
// set a home network SSID.
static void staStart()
{
  wifiStaConnected = false;
  wifiStaIP[0] = '\0';

  if (wifiStaSsid[0] == '\0') // bridge mode disabled
  {
    WiFi.mode(WIFI_AP);
    DEBUG("WiFi bridge mode disabled - AP only");
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false); // we manage credentials ourselves via EEP; don't also wear the WiFi lib's own NVS blob
  if (strlen(wifiStaPassword) >= 8)
  {
    WiFi.begin(wifiStaSsid, wifiStaPassword);
  }
  else
  {
    WiFi.begin(wifiStaSsid);
  }
  DEBUG("WiFi bridge mode: connecting to home network \"%s\"...", wifiStaSsid);
}

// Call periodically from the main loop to track STA connection state and pick up
// its IP once associated. AP-only installs (wifiStaSsid empty) skip this entirely.
void pollWifiSta()
{
  if (wifiStaSsid[0] == '\0')
    return;

  bool nowConnected = (WiFi.status() == WL_CONNECTED);
  if (nowConnected && !wifiStaConnected)
  {
    strncpy(wifiStaIP, WiFi.localIP().toString().c_str(), sizeof(wifiStaIP) - 1);
    wifiStaIP[sizeof(wifiStaIP) - 1] = '\0';
    DEBUG("WiFi bridge mode: connected to \"%s\", IP %s", wifiStaSsid, wifiStaIP);
  }
  else if (!nowConnected && wifiStaConnected)
  {
    wifiStaIP[0] = '\0';
    DEBUG("WiFi bridge mode: lost connection to \"%s\" - auto-reconnecting", wifiStaSsid);
  }
  wifiStaConnected = nowConnected;
}

// (Re)applies AP (+ STA if a home-network SSID is configured) using whatever is
// currently in wifiSsid/wifiPassword/wifiStaSsid/wifiStaPassword, and restarts
// mDNS so openhaldex.local resolves on both interfaces. Shared by first boot and
// by the rebootWiFi restart path, so both stay in sync as this grows.
void applyWifiMode()
{
  WiFi.hostname(wifiHostName);
  staStart();     // sets WIFI_AP or WIFI_AP_STA as appropriate, begins STA connect if configured
  softAPStart();  // AP always runs regardless of STA state
  WiFi.setSleep(false);
  if (canSleepAggressive)
  {
    WiFi.setTxPower(WIFI_POWER_8_5dBm); // trim AP TX power to reduce active-WiFi current
  }
  DEBUG("AP IP address: 192.168.1.1");

  // Only tear down mDNS if it was actually running - calling end() on first
  // boot (before it's ever been started) confuses the underlying ESP-IDF mdns
  // component into rejecting the addService() that follows ("Service already
  // exists"), which silently breaks openhaldex.local. Matches the original
  // code's behavior, which never called end() on the first-boot path either.
  static bool mdnsStarted = false;
  if (mdnsStarted)
  {
    MDNS.end();
  }
  MDNS.begin("openhaldex");           // openhaldex.local
  MDNS.addService("http", "tcp", 80); // advertise HTTP
  mdnsStarted = true;
}

void setupWiFi()
{
  applyWifiMode();
}

void disconnectWifi()
{
  rebootWiFi = true;
}

void resetWifiPassword()
{
  memset(wifiPassword, 0, sizeof(wifiPassword)); // clear password -> open network
  rebootWiFi = true;                             // trigger AP restart
  DEBUG("WiFi password cleared - restarting as open AP");
}

void resetWifi()
{
  memset(wifiPassword, 0, sizeof(wifiPassword));                 // clear password -> open network
  memset(wifiSsid, 0, sizeof(wifiSsid));                         // clear SSID
  strncpy(wifiSsid, wifiHostNameDefault, sizeof(wifiSsid) - 1); // restore factory default SSID
  rebootWiFi = true;                                             // trigger AP restart
  DEBUG("WiFi reset to defaults - SSID: %s, open network", wifiSsid);
}

void resetWifiSsid()
{
  memset(wifiSsid, 0, sizeof(wifiSsid));                       // clear
  strncpy(wifiSsid, wifiHostNameDefault, sizeof(wifiSsid) - 1); // restore factory default
  rebootWiFi = true;                                            // trigger AP restart
  DEBUG("WiFi SSID reset to default - restarting AP: %s", wifiSsid);
}

void resetWifiSta()
{
  memset(wifiStaSsid, 0, sizeof(wifiStaSsid));         // clear home network SSID -> bridge mode disabled
  memset(wifiStaPassword, 0, sizeof(wifiStaPassword)); // clear home network password
  wifiStaConnected = false;
  wifiStaIP[0] = '\0';
  rebootWiFi = true; // trigger AP(/STA) restart
  DEBUG("WiFi bridge mode disabled - AP only");
}