#!/usr/bin/env python3
"""
Back up and restore an OpenHaldex-C6 over its HTTP API - the command-line twin of
the "Backup & Restore" card in the web UI (Settings tab). Files are interchangeable:
a backup exported here imports in the web UI and vice versa.

What is saved: the Expert tune (lock table + steering scale), general settings,
per-frame edit switches, and the WiFi names. WiFi *passwords* are never saved -
the device only lets them be written, never read - so import asks for them once
(not echoed). Nothing secret is written to the backup file unless you answer 'y'
to the "remember it" question, in which case that file is chmod 600.

Usage (the computer must reach the controller: join its WiFi AP, or be on the same
network when bridge mode is on - then use --host openhaldex.local or its IP):

  python3 tools/openhaldex_config.py export  my-backup.json
  python3 tools/openhaldex_config.py import  my-backup.json
  python3 tools/openhaldex_config.py import  my-backup.json --tune-only
  python3 tools/openhaldex_config.py --host openhaldex.local export my-backup.json

Do an export BEFORE re-flashing: a full USB flash / filesystem erase wipes the saved
settings, and this is how you get them back.

Endpoints used (src/OpenHaldexC6_API.cpp): GET/POST /api/settings, POST /api/tune,
GET/POST /api/wifi/ssid, /api/wifi and /api/wifi/sta, plus GET /ota/health.
"""
import argparse
import getpass
import http.client
import json
import os
import sys
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "192.168.1.1"  # the controller's own access point

# Keep in sync with BACKUP_GENERAL_KEYS in data/app.js (the web UI's restore list).
BACKUP_GENERAL_KEYS = [
    "haldexGeneration", "isStandalone", "useCANifAvailable", "broadcastOpenHaldexOverCAN", "disableController",
    "disengageUnderSpeed", "disengageAboveSpeed", "disableThrottle",
    "tcForceMode", "tcForceModeValue", "hazardForceMode", "hazardForceModeValue",
    "extButtonForceMode", "extBtnForceModeValue", "disableOnboardButton", "disableExternalButton",
    "followBrake", "invertBrake", "followHandbrake", "invertHandbrake",
    "fixHunting", "dangerZoneEnabled", "esp14MinFloorPct", "bpkCeilingNm",
    "steeringScaleEnabled", "liveDiagEnabled", "ledBrightness",
    "canSleepEnabled", "canSleepAggressive", "benchMode", "lpWakeThresholdFps",
    "longLearnNotes",
]

# Connection-level failures worth retrying: the controller drops its WiFi for a couple
# of seconds every time an SSID/password is changed, and now and then a request is reset.
# (HTTPError is a URLError subclass, so it is caught separately, first.)
TRANSIENT = (urllib.error.URLError, http.client.RemoteDisconnected, ConnectionError, TimeoutError)


def api(host, path, method="GET", payload=None, timeout=5, retry_seconds=15):
    url = f"http://{host}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    deadline = time.time() + retry_seconds
    while True:
        req = urllib.request.Request(url, data=data, method=method)
        if data is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                body = resp.read()
                return json.loads(body) if body else {}
        except urllib.error.HTTPError as e:
            # Something answered, but with an error. A 406 HTML page means this address
            # is a different device (e.g. your router), not the controller.
            sys.exit(f"{method} {url} -> HTTP {e.code}. "
                     f"Is {host} really the OpenHaldex-C6? (a router answering there gives 406)")
        except TRANSIENT as e:
            if time.time() >= deadline:
                sys.exit(f"Couldn't reach {url}: {e}\n"
                         "Is this computer on the controller's WiFi (or on the same network in bridge mode)?")
            time.sleep(1)


def wait_until_up(host, seconds=30):
    """After a WiFi change the controller restarts its radio - wait for it to come back."""
    time.sleep(2)
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"http://{host}/ota/health", timeout=3):
                return True
        except TRANSIENT:
            time.sleep(1)
    return False


def write_private(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
    os.chmod(path, 0o600)


def cmd_export(args):
    settings = api(args.host, "/api/settings")
    ap_ssid = api(args.host, "/api/wifi/ssid")
    ap_pw = api(args.host, "/api/wifi")
    sta = api(args.host, "/api/wifi/sta")

    backup = {
        "_exportedAt": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "_fwVersion": settings.get("FW_VERSION"),
        "settings": settings,
        "wifi": {"ssid": ap_ssid.get("ssid"), "passwordSet": bool(ap_pw.get("passwordSet"))},
        "wifiSta": {"ssid": sta.get("ssid") or None, "passwordSet": bool(sta.get("passwordSet"))},
    }
    with open(args.file, "w") as f:
        json.dump(backup, f, indent=2)

    print(f"Exported to {args.file}")
    print(f"  Firmware:    {backup['_fwVersion']}")
    print(f"  AP name:     {backup['wifi']['ssid']}  (password set: {backup['wifi']['passwordSet']})")
    if backup["wifiSta"]["ssid"]:
        print(f"  Bridge mode: \"{backup['wifiSta']['ssid']}\"  (password set: {backup['wifiSta']['passwordSet']})")
    else:
        print("  Bridge mode: off")
    if backup["wifi"]["passwordSet"] or backup["wifiSta"]["passwordSet"]:
        print("  Note: WiFi passwords can't be read back from the controller, so they are not in")
        print("        this file - import will ask for them again.")


def password_for(section, what):
    """Password from the backup file if it was remembered there, else ask (not echoed)."""
    if not section.get("passwordSet"):
        return ""
    return section.get("password") or getpass.getpass(
        f"Backup says {what} had a password. Enter it to restore (blank = skip): ")


def cmd_import(args):
    with open(args.file) as f:
        backup = json.load(f)
    s = backup.get("settings") or {}
    if not all(isinstance(s.get(k), list) for k in ("throttleArray", "speedArray", "lockArray")):
        sys.exit("This file has no Expert tune table - is it a backup made by this tool or the web UI?")

    host = args.host
    print(f"Restoring to {host}  (backup from firmware {backup.get('_fwVersion') or backup.get('_fw_version') or 'unknown'})")

    # 1. Generation first, so the per-frame switches below land on the right table.
    if not args.tune_only and isinstance(s.get("haldexGeneration"), int):
        api(host, "/api/settings", "POST", {"haldexGeneration": s["haldexGeneration"]})

    # 2. Expert tune (+ steering scale when the backup has it).
    tune = {k: s[k] for k in ("throttleArray", "speedArray", "lockArray")}
    if isinstance(s.get("steeringArray"), list) and isinstance(s.get("steeringLockScaleArray"), list):
        tune["steeringArray"] = s["steeringArray"]
        tune["steeringLockScaleArray"] = s["steeringLockScaleArray"]
    print(f"  Expert tune restored: {api(host, '/api/tune', 'POST', tune)}")
    if args.tune_only:
        return

    # 3. General settings.
    general = {k: s[k] for k in BACKUP_GENERAL_KEYS if k in s}
    print(f"  General settings restored: {api(host, '/api/settings', 'POST', general)}")

    # 4. Per-frame edit switches, one bit at a time (that is how the API takes them).
    frames = [fb for fb in s.get("frameBlocks", [])
              if isinstance(fb.get("bit"), int) and isinstance(fb.get("enabled"), bool)]
    for fb in frames:
        api(host, "/api/settings", "POST", {"frameEditBit": fb["bit"], "frameEditOn": fb["enabled"]})
    if frames:
        print(f"  Frame edits restored: {len(frames)} switches")

    # 5. WiFi last: every change restarts the radio, and if you're connected through the
    #    AP itself you lose the link once its name/password change - that is expected.
    #    Passwords are collected first so each network is written only once.
    wifi, sta = backup.get("wifi") or {}, backup.get("wifiSta") or {}
    sta_pw = password_for(sta, f'the home network "{sta.get("ssid")}"') if sta.get("ssid") else ""
    ap_pw = password_for(wifi, "the controller's own access point")

    if sta.get("ssid"):
        r = api(host, "/api/wifi/sta", "POST", {"ssid": sta["ssid"], "password": sta_pw})
        print(f"  Home WiFi (bridge mode) restored: {r}")
        wait_until_up(host)
    if wifi.get("ssid"):
        r = api(host, "/api/wifi/ssid", "POST", {"ssid": wifi["ssid"]})
        print(f"  AP name restored: {r}")
        wait_until_up(host)
    if ap_pw:
        r = api(host, "/api/wifi", "POST", {"password": ap_pw})
        print(f"  AP password restored: {r}")

    newly_typed = (sta_pw and not sta.get("password")) or (ap_pw and not wifi.get("password"))
    if newly_typed and input("Remember the password(s) in the backup file for next time? [y/N] ").strip().lower() == "y":
        if sta_pw:
            backup["wifiSta"]["password"] = sta_pw
        if ap_pw:
            backup["wifi"]["password"] = ap_pw
        write_private(args.file, backup)
        print(f"  Saved (chmod 600). Keep {args.file} private and out of git.")


def main():
    p = argparse.ArgumentParser(description="Back up / restore an OpenHaldex-C6 (see the top of this file for details).")
    p.add_argument("--host", default=DEFAULT_HOST,
                   help=f"controller address (default {DEFAULT_HOST}; or openhaldex.local / its home-network IP)")
    sub = p.add_subparsers(dest="cmd", required=True)

    pe = sub.add_parser("export", help="save the controller's config to a JSON file")
    pe.add_argument("file")
    pe.set_defaults(func=cmd_export)

    pi = sub.add_parser("import", help="restore a JSON backup onto the controller")
    pi.add_argument("file")
    pi.add_argument("--tune-only", action="store_true", help="restore only the Expert tune, nothing else")
    pi.set_defaults(func=cmd_import)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
