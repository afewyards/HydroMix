#!/usr/bin/env python3
"""Wrap firmware/build/valvecontroller.bin in a ZCL OTA image (.ota).

Zigbee2MQTT (and any OTA server) needs the ZCL OTA file header + an upgrade-image
sub-element around the raw ESP-IDF app binary. The file version is derived from
firmware/version.txt so it matches what the running image advertises via
ota_file_version() in main/zigbee.c -- the server only offers an image whose
version is strictly greater than the device's current one.

Usage:
    python3 tools/make_ota.py [--out DIR]
"""
import argparse
import pathlib
import struct
import sys

OTA_FILE_IDENTIFIER = 0x0BEEF11E
HEADER_VERSION = 0x0100
HEADER_LENGTH = 56
FIELD_CONTROL = 0x0000
STACK_VERSION = 0x0002          # ZigBee Pro
TAG_UPGRADE_IMAGE = 0x0000

# Must match VALVECTL_MFR_CODE / image type in firmware/main/zigbee.h + zigbee.c
MANUFACTURER_CODE = 0x1234
IMAGE_TYPE = 0x0001

ROOT = pathlib.Path(__file__).resolve().parent.parent


def encode_version(text):
    """'1.0.1' -> 0x01000100, matching ota_file_version() in zigbee.c."""
    parts = [0, 0, 0]
    for i, chunk in enumerate(text.strip().split(".")[:3]):
        digits = ""
        for ch in chunk:
            if ch.isdigit():
                digits += ch
            else:
                break
        parts[i] = int(digits) if digits else 0
    return ((parts[0] & 0xFF) << 24) | ((parts[1] & 0xFF) << 16) | ((parts[2] & 0xFF) << 8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "firmware" / "build"))
    args = ap.parse_args()

    binary = ROOT / "firmware" / "build" / "valvecontroller.bin"
    version_file = ROOT / "firmware" / "version.txt"
    if not binary.exists():
        sys.exit(f"missing {binary} -- run 'idf.py build' first")
    if not version_file.exists():
        sys.exit(f"missing {version_file}")

    version_text = version_file.read_text().strip()
    file_version = encode_version(version_text)
    if file_version == 0:
        sys.exit(f"version.txt ({version_text!r}) does not encode to a usable file version")

    image = binary.read_bytes()
    total_size = HEADER_LENGTH + 6 + len(image)
    header_string = f"ValveController {version_text}".encode()[:32].ljust(32, b"\0")

    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_FILE_IDENTIFIER, HEADER_VERSION, HEADER_LENGTH, FIELD_CONTROL,
        MANUFACTURER_CODE, IMAGE_TYPE, file_version, STACK_VERSION,
        header_string, total_size,
    )
    assert len(header) == HEADER_LENGTH, len(header)

    sub_element = struct.pack("<HI", TAG_UPGRADE_IMAGE, len(image))

    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"valvecontroller-{version_text}.ota"
    out.write_bytes(header + sub_element + image)

    print(f"version      : {version_text} -> 0x{file_version:08X}")
    print(f"manufacturer : 0x{MANUFACTURER_CODE:04X}   image type: 0x{IMAGE_TYPE:04X}")
    print(f"image bytes  : {len(image)}")
    print(f"total size   : {total_size}")
    print(f"wrote        : {out}")


if __name__ == "__main__":
    main()
