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

// Bring the AP up (or back up) with the current SSID/password. Shared by the
// boot path and the rebootWiFi restart in loop() so the DHCP behaviour stays
// identical in both.
void startSoftAP()
{
  WiFi.mode(WIFI_AP);
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
