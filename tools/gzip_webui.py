# PlatformIO extra script (pre): build the LittleFS image from a gzipped copy
# of data/ instead of data/ itself.
#
# ESPAsyncWebServer serves "<file>.gz" with Content-Encoding: gzip whenever
# "<file>" is missing (both serveStatic and beginResponse(fs, path)), so the
# web UI works unchanged while taking far less flash and Wi-Fi airtime. The
# files are compressed byte-for-byte - no minifying, so nothing in the
# JavaScript can break - and data/ itself is never touched.
#
# version.json stays plain: the firmware reads it directly (readFsVersion).
# Images are already compressed, so they are copied as they are.
import gzip
import os
import shutil

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

COMPRESS = (".html", ".js", ".css", ".json", ".svg", ".webmanifest", ".txt")
KEEP_PLAIN = {"version.json"}

src = env.subst("$PROJECT_DATA_DIR")  # noqa: F821
out = os.path.join(env.subst("$BUILD_DIR"), "data_gz")  # noqa: F821

if os.path.isdir(src):
    shutil.rmtree(out, ignore_errors=True)
    before = after = 0
    for root, _dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        dest_dir = os.path.normpath(os.path.join(out, rel))
        os.makedirs(dest_dir, exist_ok=True)
        for name in files:
            path = os.path.join(root, name)
            data = open(path, "rb").read()
            before += len(data)
            if name.lower().endswith(COMPRESS) and name not in KEEP_PLAIN:
                # mtime=0 keeps the image reproducible from the same sources
                packed = gzip.compress(data, compresslevel=9, mtime=0)
                open(os.path.join(dest_dir, name + ".gz"), "wb").write(packed)
                after += len(packed)
            else:
                shutil.copyfile(path, os.path.join(dest_dir, name))
                after += len(data)
    env.Replace(PROJECT_DATA_DIR=out)  # noqa: F821
    print("gzip_webui: web UI %d -> %d bytes (%s)" % (before, after, out))
