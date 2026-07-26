#!/bin/bash
#
# create-img-macos.sh
#
# macOS version of create-img.sh: builds an SD-card flashable USBODE image
# without Linux-only tools (losetup/mkfs), using hdiutil + newfs_msdos +
# newfs_exfat + diskutil. No sudo required.
#
# Layout (matches scripts/create-img.sh):
#   MBR, partition 1: FAT32 (LBA), 200 MB, label BOOTFS, bootable
#        partition 2: exFAT, remainder (~4 MB), label IMGSTORE
#

set -e

IMG_SIZE_MB=205
BOOT_SIZE_MB=200
IMG_NAME="boot.img"
SOURCE_DIR=""
OUTPUT_DIR="."

usage() {
    echo "Usage: $0 -s SOURCE_DIR [-o OUTPUT_DIR] [-n IMG_NAME]"
    echo "  -s SOURCE_DIR   Directory to copy files from (required)"
    echo "  -o OUTPUT_DIR   Output directory (default: current directory)"
    echo "  -n IMG_NAME     Image name (default: boot.img)"
    exit 1
}

while getopts "s:o:n:h" opt; do
    case $opt in
        s) SOURCE_DIR="$OPTARG" ;;
        o) OUTPUT_DIR="$OPTARG" ;;
        n) IMG_NAME="$OPTARG" ;;
        *) usage ;;
    esac
done

[ -n "$SOURCE_DIR" ] || usage
[ -d "$SOURCE_DIR" ] || { echo "Error: source directory '$SOURCE_DIR' not found"; exit 1; }

mkdir -p "$OUTPUT_DIR"
IMG_PATH="$OUTPUT_DIR/$IMG_NAME"

DEVICE=""
cleanup() {
    if [ -n "$DEVICE" ]; then
        hdiutil detach "$DEVICE" -force >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

echo "Creating ${IMG_SIZE_MB}MB disk image..."
dd if=/dev/zero of="$IMG_PATH" bs=1m count="$IMG_SIZE_MB" 2>/dev/null

echo "Writing MBR partition table..."
python3 - "$IMG_PATH" "$IMG_SIZE_MB" "$BOOT_SIZE_MB" <<'PYEOF'
import struct, sys

img_path = sys.argv[1]
total_mb = int(sys.argv[2])
boot_mb = int(sys.argv[3])

SECTOR = 512
p1_start = 2048
p1_count = boot_mb * 1024 * 1024 // SECTOR
p2_start = p1_start + p1_count
p2_count = total_mb * 1024 * 1024 // SECTOR - p2_start

def chs(_lba):
    # CHS values are ignored by modern loaders; use the standard LBA marker
    return bytes((0xFE, 0xFF, 0xFF))

def entry(boot, ptype, start, count):
    return (bytes((boot,)) + chs(start) + bytes((ptype,)) + chs(start + count - 1)
            + struct.pack('<II', start, count))

mbr = bytearray(SECTOR)
mbr[446:462] = entry(0x80, 0x0C, p1_start, p1_count)   # FAT32 LBA, bootable
mbr[462:478] = entry(0x00, 0x07, p2_start, p2_count)   # exFAT
mbr[510] = 0x55
mbr[511] = 0xAA

with open(img_path, 'r+b') as f:
    f.write(mbr)
PYEOF

echo "Attaching image..."
DEVICE=$(hdiutil attach -imagekey diskimage-class=CRawDiskImage -nomount "$IMG_PATH" \
         | head -1 | awk '{print $1}')
echo "Attached as $DEVICE"

echo "Creating FAT32 filesystem on partition 1..."
newfs_msdos -F 32 -v BOOTFS "${DEVICE/disk/rdisk}s1" >/dev/null

echo "Creating exFAT filesystem on partition 2..."
newfs_exfat -v IMGSTORE "${DEVICE/disk/rdisk}s2" >/dev/null

echo "Mounting partitions..."
diskutil mount "${DEVICE}s1" >/dev/null
diskutil mount "${DEVICE}s2" >/dev/null

BOOT_MOUNT=$(diskutil info "${DEVICE}s1" | awk -F': *' '/Mount Point/ {print $2}')
STORE_MOUNT=$(diskutil info "${DEVICE}s2" | awk -F': *' '/Mount Point/ {print $2}')
[ -n "$BOOT_MOUNT" ] || { echo "Error: boot partition not mounted"; exit 1; }
[ -n "$STORE_MOUNT" ] || { echo "Error: image store partition not mounted"; exit 1; }

echo "Copying files from $SOURCE_DIR..."
cp -r "$SOURCE_DIR"/* "$BOOT_MOUNT"/
echo "This is the exFAT partition" > "$STORE_MOUNT/readme.txt"

# remove macOS metadata that would waste space on the SD card
find "$BOOT_MOUNT" -name '.DS_Store' -delete 2>/dev/null || true
rm -rf "$BOOT_MOUNT/.fseventsd" "$BOOT_MOUNT/.Trashes" 2>/dev/null || true

sync

echo "Detaching..."
diskutil unmountDisk "$DEVICE" >/dev/null
hdiutil detach "$DEVICE" >/dev/null
DEVICE=""

echo "Done! Created image: $IMG_PATH"
