#include <OpenHaldexC6_WiFi.h>
#include <cstring>
#include <esp_netif.h>
#include <dhcpserver/dhcpserver.h>

// Legacy WiFi implementation - now a stub
// All WiFi functionality has moved to OpenHaldexC6_WebServer.cpp
// ---------------------------------------------------------------------------
static void softAPLocalOnly()
{
  esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (ap == NULL)
    return;
  esp_netif_dhcps_stop(ap);
  dhcps_offer_t offer = 0; // clear OFFER_ROUTER: no gateway in DHCP offers
  esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &offer, sizeof(offer));
  esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offer, sizeof(offer)); // no DNS either
  esp_netif_dhcps_start(ap);
}

// ---------------------------------------------------------------------------
// Bridge mode (PR #39): optionally join a home/garage network as a station as
// well. One radio is shared between AP and STA, and every STA connect attempt
// scans all channels, pulling the AP off its own channel for a second or two.
// That is fine when the network is there (one attempt, done), but with the
// home SSID saved and the car parked anywhere else the core's auto-reconnect
// would re-scan forever and the AP would stutter for whoever is in the car.
// So: try hard for a short window after (re)start, then back off to one
// attempt every few minutes.
// ---------------------------------------------------------------------------
static const uint32_t STA_INITIAL_WINDOW_MS = 20000;      // keep the core's auto-reconnect for this long after start
static const uint32_t STA_RETRY_INTERVAL_MS = 5UL * 60000; // then one fresh attempt this often
static uint32_t staStartedMs = 0;   // when the current connect window opened
static uint32_t staLastAttemptMs = 0;
static bool staBackedOff = false;   // auto-reconnect disabled, we're on the slow retry

static void staBegin()
{
  if (strlen(wifiStaPassword) >= 8)
    WiFi.begin(wifiStaSsid, wifiStaPassword);
  else
    WiFi.begin(wifiStaSsid); // open network
  staLastAttemptMs = millis();
}

static void staStart()
{
  wifiStaConnected = false;
  wifiStaIP[0] = '\0';
  staBackedOff = false;
  if (wifiStaSsid[0] == '\0')
    return; // bridge mode off - startSoftAP() stays in plain WIFI_AP
  WiFi.mode(WIFI_AP_STA);
  WiFi.persistent(false); // credentials live in our EEP, not the core's NVS blob
  WiFi.setAutoReconnect(true);
  WiFi.setMinSecurity(strlen(wifiStaPassword) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN); // core default refuses open networks
  staStartedMs = millis();
  staBegin();
  DEBUG("WiFi bridge mode: joining \"%s\"...", wifiStaSsid);
}

// Called from loop(): tracks the STA connection, picks up its IP, and runs the
// back-off described above. No-op when bridge mode is off.
void pollWifiSta()
{
  if (wifiStaSsid[0] == '\0')
    return;
  const uint32_t now = millis();
  const bool nowConnected = (WiFi.status() == WL_CONNECTED);
  if (nowConnected && !wifiStaConnected)
  {
    strncpy(wifiStaIP, WiFi.localIP().toString().c_str(), sizeof(wifiStaIP) - 1);
    wifiStaIP[sizeof(wifiStaIP) - 1] = '\0';
    DEBUG("WiFi bridge mode: connected to \"%s\" as %s", wifiStaSsid, wifiStaIP);
    // Connected: let the core handle brief drop-outs again.
    if (staBackedOff)
    {
      WiFi.setAutoReconnect(true);
      staBackedOff = false;
    }
    staStartedMs = now; // a later drop gets a fresh fast window
  }
  else if (!nowConnected && wifiStaConnected)
  {
    wifiStaIP[0] = '\0';
    DEBUG("WiFi bridge mode: lost \"%s\"", wifiStaSsid);
  }
  wifiStaConnected = nowConnected;

  if (!nowConnected)
  {
    if (!staBackedOff && (now - staStartedMs) > STA_INITIAL_WINDOW_MS)
    {
      // Not there. Stop the core's continuous re-scan so the AP is left alone.
      WiFi.setAutoReconnect(false);
      WiFi.disconnect(false, false);
      staBackedOff = true;
      DEBUG("WiFi bridge mode: \"%s\" not found - retrying every %lu min", wifiStaSsid, (unsigned long)(STA_RETRY_INTERVAL_MS / 60000UL));
    }
    else if (staBackedOff && (now - staLastAttemptMs) > STA_RETRY_INTERVAL_MS)
    {
      staBegin(); // one shot; the core gives up on its own without auto-reconnect
    }
  }
}

// Bring the AP up (or back up) with the current SSID/password, plus the STA
// side if a home network is configured. Shared by the boot path and the
// rebootWiFi restart in loop() so the DHCP behaviour stays identical in both.
void startSoftAP()
{
  WiFi.mode(WIFI_AP);
  staStart(); // switches to WIFI_AP_STA and begins connecting when bridge mode is on
  // gateway 0.0.0.0 = local-only network (see softAPLocalOnly). Checked against
  // NetworkInterface::config() in the 3.x core: a gateway outside the AP subnet
  // simply skips the "gateway inside the DHCP range" test, so this is accepted,
  // and the lease pool defaults to <AP IP>+1 .. +11 (192.168.1.2-.12). The
  // result is an offer with an address and netmask and no router or
  // DNS option at all, which is exactly what a local-only network should look
  // like. Offering router 0.0.0.0 instead would be malformed - hence clearing
  // the option rather than relying on the zero gateway.
  bool cfg = WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0));
  if (!cfg)
    DEBUG("softAPConfig REJECTED - AP will fall back to the default 192.168.4.1");
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
  softAPLocalOnly();
  WiFi.setSleep(false);
  // Aggressive sleep: trim AP TX power to reduce active-WiFi current.
  if (canSleepAggressive)
  {
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
  }
}

void setupWiFi()
{
  // WiFi setup is now in main.cpp
  WiFi.hostname(wifiHostName);
  DEBUG("Creating Access Point...");
  startSoftAP();
  // Print what the AP actually came up on - the old hardcoded "192.168.1.1"
  // would have hidden a rejected softAPConfig behind the address we wanted.
  DEBUG("IP address: %s", WiFi.softAPIP().toString().c_str());

  MDNS.begin("openhaldex");           // openhaldex.local
  MDNS.addService("http", "tcp", 80); // advertise HTTP
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
  memset(wifiStaSsid, 0, sizeof(wifiStaSsid));         // empty SSID = bridge mode off
  memset(wifiStaPassword, 0, sizeof(wifiStaPassword));
  wifiStaConnected = false;
  wifiStaIP[0] = '\0';
  rebootWiFi = true; // restart as plain AP
  DEBUG("WiFi bridge mode disabled - AP only");
}
