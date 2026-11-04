#!/usr/bin/env python3
"""
Build the release index (Releases/releases.json) from the actual binaries, and
verify that it still matches them.

Why this exists: the web updater lists releases out of releases.json and shows
the indexed size while downloading. If a .bin is rebuilt and re-uploaded without
regenerating the index, the index silently describes a file that no longer
exists - the version string then names more than one binary and nobody can tell
which one a unit is running. It has happened more than once: V9.00.0's firmware
was re-uploaded after the index was written, V8.00.3's entry was back-filled
192 bytes short of the real file, and V9.00.6's firmware is currently 1120
bytes larger than its index entry says.

Two subcommands:

  add     stage a new release: copy the built binaries into Releases/V<version>/,
          hash what actually landed there, and rewrite releases.json.

  verify  check every entry in releases.json against the files on disk. Exits
          non-zero on any mismatch, so CI can block a release that would ship a
          stale index. Run this on every push.

Examples:

  python3 tools/make_release.py add 10.00.2 \\
      --firmware   .pio/build/esp32c6/firmware.bin \\
      --filesystem .pio/build/esp32c6/littlefs.bin \\
      --bootloader .pio/build/esp32c6/bootloader.bin \\
      --partitions .pio/build/esp32c6/partitions.bin \\
      --notes "Gen5 0AY lock fixes; own release channel."

  python3 tools/make_release.py verify
  python3 tools/make_release.py verify --strict   # also fail on untracked dirs

The hash is always taken from the file inside Releases/ after the copy, never
from the build-output path, so the index describes exactly what is committed.
"""
import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RELEASES_DIR = os.path.join(REPO_ROOT, "Releases")
INDEX_PATH = os.path.join(RELEASES_DIR, "releases.json")
PRODUCT = "OpenHaldex-C6"

# The updater only sees folders matching this (digits and dots after the V), so
# a release has to be named that way or it is invisible in the UI.
FOLDER_RE = re.compile(r"^V(\d+(?:\.\d+)*)$")

# Images the updater can actually install. bootloader/partitions are published
# for USB recovery only and are not part of an OTA, so they are not indexed.
INDEXED = (("firmware", "firmware.bin"), ("filesystem", "littlefs.bin"))
EXTRA = (("bootloader", "bootloader.bin"), ("partitions", "partitions.bin"))


def sha256_and_size(path):
    h = hashlib.sha256()
    n = 0
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
            n += len(chunk)
    return h.hexdigest(), n


def version_key(v):
    """Sort 10.00.0 above 9.00.7 - numeric per field, short versions first."""
    parts = []
    for field in str(v).split("."):
        parts.append(int(field) if field.isdigit() else -1)
    return parts


def load_index():
    if not os.path.exists(INDEX_PATH):
        return {"product": PRODUCT, "latest": "", "releases": []}
    with open(INDEX_PATH) as f:
        return json.load(f)


def write_index(index):
    # Newest first, so the UI's "latest" is just releases[0] for a given channel.
    index["releases"].sort(key=lambda r: version_key(r.get("version", "0")), reverse=True)
    stable = [r for r in index["releases"] if r.get("channel") == "stable"]
    index["latest"] = (stable or index["releases"] or [{}])[0].get("version", "")
    index.setdefault("product", PRODUCT)
    # Key order kept stable so diffs stay readable.
    ordered = {"product": index["product"], "latest": index["latest"], "releases": index["releases"]}
    with open(INDEX_PATH, "w") as f:
        json.dump(ordered, f, indent=2)
        f.write("\n")


VER_HEADER = os.path.join(REPO_ROOT, "include", "OpenHaldexC6_ver.h")


def notes_from_changelog(version):
    """Pull this version's entry out of the changelog in OpenHaldexC6_ver.h.

    The changelog is the one place release notes are already written, so taking
    them from there beats maintaining a second copy - and beats interpolating a
    commit message into a shell argument in CI. Returns "" if there is no entry.
    """
    try:
        text = open(VER_HEADER).read()
    except OSError:
        return ""
    # Entry starts at "V<version> - " and runs until the next V<n>.<n> heading
    # or the end of the comment block.
    start = re.search(r"^V" + re.escape(version) + r"\s*-\s*", text, re.M)
    if not start:
        return ""
    rest = text[start.end():]
    end = re.search(r"^(?:V\d|\*/|---)", rest, re.M)
    body = rest[: end.start()] if end else rest
    # Flatten: the changelog is hard-wrapped and bullet-indented.
    lines = [ln.strip() for ln in body.splitlines()]
    out, cur = [], []
    for ln in lines:
        if ln.startswith("- "):
            if cur:
                out.append(" ".join(cur))
            cur = [ln[2:].strip()]
        elif ln:
            cur.append(ln)
    if cur:
        out.append(" ".join(cur))
    joined = "; ".join(p.rstrip(". ") for p in out if p)
    return re.sub(r"\s+", " ", joined).strip()


def cmd_add(args):
    if not FOLDER_RE.match("V" + args.version):
        sys.exit(f"Version {args.version!r} must be digits and dots only (e.g. 10.00.2) "
                 "or the updater will not list it.")

    dest_dir = os.path.join(RELEASES_DIR, "V" + args.version)
    os.makedirs(dest_dir, exist_ok=True)

    sources = {"firmware": args.firmware, "filesystem": args.filesystem,
               "bootloader": args.bootloader, "partitions": args.partitions}
    for kind, name in INDEXED:
        if not sources[kind]:
            sys.exit(f"--{kind} is required (an OTA needs both firmware and filesystem).")

    notes = args.notes
    if args.notes_file:
        notes = open(args.notes_file).read().strip()
    elif not notes:
        notes = notes_from_changelog(args.version)
        if notes:
            print(f"  notes taken from the {args.version} changelog entry")
        else:
            print(f"  WARNING: no changelog entry for V{args.version} - notes will be empty. "
                  f"Add one to include/OpenHaldexC6_ver.h.")

    entry = {"version": args.version, "channel": args.channel,
             "date": args.date or datetime.date.today().isoformat(),
             "ota": not args.no_ota, "notes": notes}

    for kind, name in INDEXED + EXTRA:
        src = sources.get(kind)
        if not src:
            continue
        if not os.path.exists(src):
            sys.exit(f"{kind}: {src} does not exist - build first.")
        dest = os.path.join(dest_dir, name)
        if os.path.abspath(src) != os.path.abspath(dest):
            shutil.copy2(src, dest)
        # Hash the copy, not the source: the index must describe what ships.
        digest, size = sha256_and_size(dest)
        print(f"  {name:16s} {size:>9} bytes  {digest[:16]}...")
        if kind in dict(INDEXED):
            entry[kind] = {"path": f"V{args.version}/{name}", "size": size, "sha256": digest}

    index = load_index()
    index["releases"] = [r for r in index["releases"] if r.get("version") != args.version]
    index["releases"].append(entry)
    write_index(index)
    print(f"\nIndexed {args.version} ({entry['channel']}, {entry['date']}). "
          f"latest is now {load_index()['latest']}.")
    print("Commit Releases/ and tag the release; then run `verify` to confirm.")


def cmd_reindex(args):
    """Recompute size/sha256 for existing entries from the files on disk.

    For adopting a repository whose index was written by hand or back-filled:
    it corrects the hashes without touching version, channel, date, notes or
    ota, so the index starts describing the binaries that are actually here.
    It cannot recover which binary an old entry *meant* to describe - if a file
    was replaced, the old hash is simply gone.
    """
    index = load_index()
    changed = []
    for rel in index.get("releases", []):
        version = rel.get("version", "?")
        for kind, _ in INDEXED:
            meta = rel.get(kind)
            if not meta:
                continue
            path = os.path.join(RELEASES_DIR, meta.get("path", ""))
            if not os.path.exists(path):
                print(f"  skip {version} {kind}: missing {meta.get('path')}")
                continue
            digest, size = sha256_and_size(path)
            if meta.get("size") != size or (meta.get("sha256") or "").lower() != digest:
                changed.append(f"{version} {kind}")
                meta["size"], meta["sha256"] = size, digest
    if not changed:
        print("Index already matches the binaries; nothing to do.")
        return 0
    if args.dry_run:
        print("Would correct:", ", ".join(changed))
        return 0
    write_index(index)
    print("Corrected:", ", ".join(changed))
    print("Note: this records what the files ARE, which is not proof of what they "
          "were meant to be. Re-verify after committing.")
    return 0


def cmd_verify(args):
    index = load_index()
    problems, checked = [], 0

    for rel in index.get("releases", []):
        version = rel.get("version", "?")
        for kind, _ in INDEXED:
            meta = rel.get(kind)
            if not meta:
                problems.append(f"{version}: no {kind} entry")
                continue
            path = os.path.join(RELEASES_DIR, meta.get("path", ""))
            if not os.path.exists(path):
                problems.append(f"{version} {kind}: missing file {meta.get('path')}")
                continue
            digest, size = sha256_and_size(path)
            checked += 1
            if meta.get("size") != size:
                problems.append(f"{version} {kind}: size indexed {meta.get('size')} "
                                f"but file is {size} ({size - meta.get('size'):+d} bytes)")
            if (meta.get("sha256") or "").lower() != digest:
                problems.append(f"{version} {kind}: sha256 mismatch "
                                f"(indexed {str(meta.get('sha256'))[:16]}..., "
                                f"actual {digest[:16]}...)")

    # A release folder the updater would show but the index does not describe.
    if args.strict:
        indexed = {r.get("version") for r in index.get("releases", [])}
        for name in sorted(os.listdir(RELEASES_DIR)):
            m = FOLDER_RE.match(name)
            if m and m.group(1) not in indexed:
                problems.append(f"{name}: folder present but not in releases.json")

    if problems:
        print(f"releases.json does NOT match the binaries ({len(problems)} problem(s)):")
        for p in problems:
            print("  -", p)
        print("\nRegenerate with `make_release.py add <version> ...` rather than "
              "editing the index by hand.")
        return 1

    print(f"releases.json matches all {checked} indexed image(s). latest = {index.get('latest')}")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("add", help="stage and index a new release")
    a.add_argument("version", help="e.g. 10.00.2 (digits and dots only)")
    a.add_argument("--firmware", required=True)
    a.add_argument("--filesystem", required=True)
    a.add_argument("--bootloader")
    a.add_argument("--partitions")
    a.add_argument("--notes", default="",
                   help="release notes; omit to take them from this version's changelog entry")
    a.add_argument("--notes-file", help="read notes from this file instead")
    a.add_argument("--channel", default="stable", choices=["stable", "beta"])
    a.add_argument("--date", help="YYYY-MM-DD (default: today)")
    a.add_argument("--no-ota", action="store_true",
                   help="publish for USB recovery only; the updater will not offer it")
    a.set_defaults(func=cmd_add)

    r = sub.add_parser("reindex", help="correct existing entries' size/sha256 from the files on disk")
    r.add_argument("--dry-run", action="store_true", help="report what would change, write nothing")
    r.set_defaults(func=cmd_reindex)

    v = sub.add_parser("verify", help="check releases.json against the files on disk")
    v.add_argument("--strict", action="store_true",
                   help="also fail on release folders missing from the index")
    v.set_defaults(func=cmd_verify)

    args = p.parse_args()
    sys.exit(args.func(args) or 0)


if __name__ == "__main__":
    main()
