#!/usr/bin/env python3
"""
Export/import OpenHaldex-C6 configuration (Expert tune table, general settings,
and WiFi AP identity) over the device's HTTP API, so a firmware update never
has to mean re-typing the Expert grid by hand again.

Talks to the endpoints in src/OpenHaldexC6_API.cpp (as of origin/main):
  GET  /api/settings        -> full settings blob incl. throttleArray/speedArray/lockArray
  POST /api/settings        -> write back the general settings
  POST /api/tune            -> write back throttleArray/speedArray/lockArray
  GET  /api/wifi/ssid       -> {ssid, default}
  POST /api/wifi/ssid       -> {ssid}
  GET  /api/wifi            -> {passwordSet}   (password itself is write-only, by design)
  POST /api/wifi            -> {password}      (8-64 chars, or "" to clear -> open network)
  GET  /api/wifi/sta        -> {ssid, passwordSet, connected, ip}   (bridge mode: optional home-network client)
  POST /api/wifi/sta        -> {ssid, password}  (empty ssid disables bridge mode)

Usage:
  # while your Mac is joined to the OpenHaldex-C6 WiFi AP:
  python3 openhaldex_config.py export  backup.json
  python3 openhaldex_config.py import  backup.json
  python3 openhaldex_config.py import  backup.json --tune-only   # just the Expert grid

The AP password is never read from or written to the backup file in plaintext
handling from chat/scripts; on import, if the backup says a password was set,
you'll be prompted for it live in the terminal (getpass, not echoed) rather
than it ever being stored. If you want it remembered for future imports,
answer 'y' when asked and it's saved to backup.json with mode 0600 -- keep
that file private (it's already covered by tools/.gitignore).
"""
import argparse
import getpass
import json
import sys
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "192.168.1.1"  # OpenHaldex-C6 softAP gateway IP; use --host openhaldex.local if on mDNS


def api(host, path, method="GET", payload=None, timeout=5):
    url = f"http://{host}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read()
            return json.loads(body) if body else {}
    except urllib.error.URLError as e:
        sys.exit(f"Couldn't reach {url}: {e}\n"
                 f"Is this Mac joined to the OpenHaldex-C6 WiFi network (or on the same LAN)?")


def cmd_export(args):
    settings = api(args.host, "/api/settings")
    wifi_ssid = api(args.host, "/api/wifi/ssid")
    wifi_pw_state = api(args.host, "/api/wifi")
    wifi_sta = api(args.host, "/api/wifi/sta")

    backup = {
        "_exported_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "_fw_version": settings.get("FW_VERSION"),
        "settings": settings,
        "wifi": {
            "ssid": wifi_ssid.get("ssid"),
            "passwordSet": wifi_pw_state.get("passwordSet", False),
            "password": None,  # never populated automatically -- API is write-only for the password
        },
        "wifiSta": {
            "ssid": wifi_sta.get("ssid") or None,
            "passwordSet": wifi_sta.get("passwordSet", False),
            "password": None,  # never populated automatically -- API is write-only for the password
        },
    }

    with open(args.file, "w") as f:
        json.dump(backup, f, indent=2)

    print(f"Exported to {args.file}")
    print(f"  Firmware: {backup['_fw_version']}")
    print(f"  AP SSID: {backup['wifi']['ssid']}  (password set: {backup['wifi']['passwordSet']})")
    if backup["wifiSta"]["ssid"]:
        print(f"  Bridge mode: joined to \"{backup['wifiSta']['ssid']}\" (password set: {backup['wifiSta']['passwordSet']})")
    else:
        print("  Bridge mode: disabled (AP only)")
    if backup["wifi"]["passwordSet"] or backup["wifiSta"]["passwordSet"]:
        print("  NOTE: passwords can't be read back from the device (by design, write-only).")
        print("        Run this script's 'import' after your next update and it'll ask")
        print("        you to re-enter them once, then remember them for next time if you want.")


def cmd_import(args):
    with open(args.file) as f:
        backup = json.load(f)

    settings = backup["settings"]

    # --- Expert tune table (throttleArray / speedArray / lockArray) ---
    tune_payload = {
        "throttleArray": settings["throttleArray"],
        "speedArray": settings["speedArray"],
        "lockArray": settings["lockArray"],
    }
    resp = api(args.host, "/api/tune", "POST", tune_payload)
    print(f"Tune table restored: {resp}")

    if args.tune_only:
        return

    # --- general settings ---
    general_keys = [
        "haldexGeneration", "forceModeValue", "disengageUnderSpeed", "disengageAboveSpeed",
        "disableThrottle", "disableController", "isStandalone", "tcForceMode",
        "extButtonForceMode", "disableOnboardButton", "disableExternalButton",
        "followBrake", "invertBrake", "followHandbrake", "invertHandbrake",
        "broadcastOpenHaldexOverCAN",
    ]
    general_payload = {k: settings[k] for k in general_keys if k in settings}
    resp = api(args.host, "/api/settings", "POST", general_payload)
    print(f"General settings restored: {resp}")

    # --- WiFi identity ---
    wifi = backup.get("wifi", {})
    ssid = wifi.get("ssid")
    if ssid:
        resp = api(args.host, "/api/wifi/ssid", "POST", {"ssid": ssid})
        print(f"WiFi SSID restored ({ssid}): {resp}")

    if wifi.get("passwordSet"):
        pw = wifi.get("password")
        if not pw:
            pw = getpass.getpass("Backup says a WiFi AP password was set but it isn't stored. "
                                  "Enter it now to restore (blank to skip): ")
        if pw:
            resp = api(args.host, "/api/wifi", "POST", {"password": pw})
            print(f"WiFi password restored: {resp}")
            remember = input("Save this password into the backup file for next time? [y/N] ").strip().lower()
            if remember == "y":
                backup["wifi"]["password"] = pw
                with open(args.file, "w") as f:
                    json.dump(backup, f, indent=2)
                import os
                os.chmod(args.file, 0o600)
                print(f"Saved (file permissions set to 0600). Keep {args.file} private.")

    # --- home WiFi (bridge mode) ---
    wifi_sta = backup.get("wifiSta", {})
    sta_ssid = wifi_sta.get("ssid")
    if sta_ssid:
        sta_pw = wifi_sta.get("password") or ""
        if wifi_sta.get("passwordSet") and not sta_pw:
            sta_pw = getpass.getpass(f"Backup says a password was set for home network \"{sta_ssid}\" "
                                      "but it isn't stored. Enter it now to restore (blank to skip): ")
        resp = api(args.host, "/api/wifi/sta", "POST", {"ssid": sta_ssid, "password": sta_pw})
        print(f"Home WiFi (bridge mode) restored ({sta_ssid}): {resp}")
        if sta_pw:
            remember = input("Save this home WiFi password into the backup file for next time? [y/N] ").strip().lower()
            if remember == "y":
                backup["wifiSta"]["password"] = sta_pw
                with open(args.file, "w") as f:
                    json.dump(backup, f, indent=2)
                import os
                os.chmod(args.file, 0o600)
                print(f"Saved (file permissions set to 0600). Keep {args.file} private.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=DEFAULT_HOST, help=f"device address (default {DEFAULT_HOST}, or try openhaldex.local)")
    sub = p.add_subparsers(dest="cmd", required=True)

    pe = sub.add_parser("export", help="pull current device config into a JSON file")
    pe.add_argument("file")
    pe.set_defaults(func=cmd_export)

    pi = sub.add_parser("import", help="push a JSON config file back to the device")
    pi.add_argument("file")
    pi.add_argument("--tune-only", action="store_true", help="only restore the Expert lock table, nothing else")
    pi.set_defaults(func=cmd_import)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
