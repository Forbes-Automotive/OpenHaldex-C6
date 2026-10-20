#!/usr/bin/env python3
"""
Record the car's CAN traffic through an OpenHaldex-C6 and compare phases.

The controller's analyzer mode streams every frame it sees - bus 0 is the car
side, bus 1 the Haldex side - over GVRET on TCP port 23, while still forwarding
everything untouched. This tool drives that from a computer, so no SavvyCAN is
needed, and records each step of a test as a labelled phase. `analyze` then
lists what is on each bus, names every ID it can from the PQ DBC and the
firmware's own ID list, and flags anything nobody accounts for.

Useful for "is an aftermarket box on the bus, and which side of the controller
is it on?", for "what keeps the car's bus awake after locking?", and for any
"what changed between A and B" CAN question. A PQ DBC such as opendbc's
vw_pq.dbc names the stock IDs; the firmware's own OpenHaldexC6_canID.h is read
automatically.

  python3 tools/can_capture.py --host 10.0.0.73 capture armed   --seconds 30
  python3 tools/can_capture.py --host 10.0.0.73 capture pin     --seconds 30
  python3 tools/can_capture.py --host 10.0.0.73 off             # analyzer off
  python3 tools/can_capture.py analyze --dbc vw_pq.dbc

While analyzer mode is on the controller does NOT edit frames - the Haldex runs
stock. `capture` turns it off again when it finishes unless --keep is given;
`off` is there for when a run is interrupted. Do not drive with it left on and
expect OpenHaldex control.
"""
import argparse
import csv
import glob
import json
import os
import re
import socket
import struct
import sys
import time
import urllib.request
from collections import defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CANID_H = os.path.join(REPO, "include", "OpenHaldexC6_canID.h")
BUS_NAME = {0: "car", 1: "haldex"}


def api_post(host, payload):
    req = urllib.request.Request(f"http://{host}/api/settings", data=json.dumps(payload).encode(),
                                 method="POST", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=8) as r:
        return r.read().decode()


def api_get(host):
    with urllib.request.urlopen(f"http://{host}/api/settings", timeout=8) as r:
        return json.load(r)


def set_analyzer(host, on):
    api_post(host, {"analyzerMode": bool(on)})
    time.sleep(0.5)
    return bool(api_get(host).get("analyzerMode"))


def read_frames(sock, seconds):
    """Parse GVRET binary frames: F1 00 ts(4) id(4) bus<<4|dlc data[dlc]."""
    buf = b""
    end = time.time() + seconds
    frames = []
    sock.settimeout(0.5)
    while time.time() < end:
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
        while True:
            i = buf.find(b"\xF1\x00")
            if i < 0:
                buf = buf[-1:]
                break
            if len(buf) < i + 11:
                buf = buf[i:]
                break
            dlc = buf[i + 10] & 0x0F
            if dlc > 8:  # not a frame header - resync one byte on
                buf = buf[i + 1:]
                continue
            if len(buf) < i + 11 + dlc:
                buf = buf[i:]
                break
            ts, raw_id = struct.unpack_from("<II", buf, i + 2)
            bus = buf[i + 10] >> 4
            data = buf[i + 11:i + 11 + dlc]
            frames.append((time.time(), ts, raw_id & 0x1FFFFFFF, bool(raw_id & 0x80000000), bus, data.hex()))
            buf = buf[i + 11 + dlc:]
    return frames


def cmd_capture(a):
    os.makedirs(a.out, exist_ok=True)
    print(f"analyzer on: {set_analyzer(a.host, True)}")
    try:
        with socket.create_connection((a.host, 23), timeout=8) as s:
            s.sendall(b"\xE7\xE7")  # enter GVRET binary mode
            print(f"recording '{a.label}' for {a.seconds}s ...", flush=True)
            frames = read_frames(s, a.seconds)
    finally:
        if not a.keep:
            print(f"analyzer on after turning off: {set_analyzer(a.host, False)}")
    path = os.path.join(a.out, f"{a.label}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_time", "dev_us", "id", "extended", "bus", "data"])
        w.writerows(frames)
    per_bus = defaultdict(int)
    for fr in frames:
        per_bus[fr[4]] += 1
    print(f"saved {len(frames)} frames to {path}  " +
          "  ".join(f"{BUS_NAME.get(b, b)}={n}" for b, n in sorted(per_bus.items())))


def cmd_off(a):
    print(f"analyzer on: {set_analyzer(a.host, False)}")


def load_names(dbc):
    names = {}
    if dbc and os.path.exists(dbc):
        for line in open(dbc, encoding="latin-1"):
            m = re.match(r"BO_ (\d+) (\w+):", line)
            if m:
                names[int(m.group(1)) & 0x1FFFFFFF] = m.group(2)
    if os.path.exists(CANID_H):
        for line in open(CANID_H, encoding="latin-1"):
            m = re.match(r"\s*#define\s+(\w+)\s+(0x[0-9A-Fa-f]+)\b", line)
            if m:
                names.setdefault(int(m.group(2), 16), m.group(1) + " (fw)")
    return names


def summarise(path):
    rows = list(csv.DictReader(open(path)))
    span = (float(rows[-1]["host_time"]) - float(rows[0]["host_time"])) if len(rows) > 1 else 1.0
    stat = {}
    for r in rows:
        key = (int(r["bus"]), int(r["id"]))
        s = stat.setdefault(key, {"n": 0, "payloads": set(), "last": ""})
        s["n"] += 1
        s["payloads"].add(r["data"])
        s["last"] = r["data"]
    return stat, max(span, 1e-3), len(rows)


def cmd_analyze(a):
    names = load_names(a.dbc)
    phases = {}
    for p in sorted(glob.glob(os.path.join(a.out, "*.csv"))):
        phases[os.path.splitext(os.path.basename(p))[0]] = summarise(p)
    if not phases:
        sys.exit(f"no captures in {a.out}")

    for label, (stat, span, total) in phases.items():
        print(f"\n=== {label}: {total} frames over {span:.1f}s ===")
        for (bus, cid), s in sorted(stat.items()):
            name = names.get(cid)
            flag = "" if name else "   <-- UNKNOWN (not stock PQ, not in firmware)"
            print(f"  {BUS_NAME.get(bus, bus):6s} 0x{cid:03X} {s['n']/span:7.1f}/s "
                  f"{len(s['payloads']):4d} distinct  last={s['last']:<16} {name or '?'}{flag}")

    labels = list(phases)
    if len(labels) > 1:
        print("\n=== differences between phases ===")
        keys = set().union(*(set(v[0]) for v in phases.values()))
        for key in sorted(keys):
            present = [l for l in labels if key in phases[l][0]]
            bus, cid = key
            tag = f"{BUS_NAME.get(bus, bus):6s} 0x{cid:03X} {names.get(cid, 'UNKNOWN')}"
            if len(present) != len(labels):
                print(f"  {tag}: only in {', '.join(present)}")
            else:
                sets = [phases[l][0][key]["payloads"] for l in labels]
                if any(s != sets[0] for s in sets[1:]) and max(len(s) for s in sets) <= 4:
                    # Low-variety frames that changed are status/state bits - worth a look.
                    print(f"  {tag}: payload changed " +
                          "  ".join(f"{l}={sorted(phases[l][0][key]['payloads'])}" for l in labels))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="openhaldex.local")
    p.add_argument("--out", default="can_captures")
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="record one labelled phase")
    c.add_argument("label")
    c.add_argument("--seconds", type=float, default=30)
    c.add_argument("--keep", action="store_true", help="leave analyzer mode on afterwards")
    c.set_defaults(func=cmd_capture)
    o = sub.add_parser("off", help="turn analyzer mode off")
    o.set_defaults(func=cmd_off)
    z = sub.add_parser("analyze", help="summarise and diff the captured phases")
    z.add_argument("--dbc", help="PQ DBC for naming IDs (e.g. opendbc vw_pq.dbc)")
    z.set_defaults(func=cmd_analyze)
    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
