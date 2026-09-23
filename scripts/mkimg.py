#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""Build sdcard.img (FAT32) from the repo's machine/UI files. Pure Python,
no dosfstools / pyfatfs needed, so it also runs on a stock Windows or macOS
Python. Same layout as scripts/mkimg.sh:

  /system/machine/<every machines/*.tsv|*.obj|*.stl>
  /system/machine/demo_box.obj, demo_part.stl, embedded_toolpods.tsv   (devices/)
  /system/ui/embedded_ui.tsv                                          (devices/)

Geometry mirrors what `mkfs.vfat -F 32` produced for the committed 64 MiB
image: superfloppy (no partition table), 512-byte sectors, 1 sector per
cluster, 32 reserved sectors, 2 FATs, root at cluster 2, FSInfo at sector 1,
backup boot sector at 6. Long names get VFAT LFN entries (the kernel's FAT32
reader matches on them). Timestamps are fixed, so the output is
byte-for-byte reproducible from the same inputs.

Remember: files on the card SHADOW the kernel's embedded defaults at boot, so
regenerate the image whenever devices/embedded_ui.tsv or anything under
machines/ changes — a stale card silently overrides the fresh kernel copy.

usage: python3 scripts/mkimg.py [OUT_IMG]   (default: sdcard.img at the repo root)
"""
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "sdcard.img")

BPS = 512
SPC = 1
TOTAL_SECTORS = 64 * 1024 * 1024 // BPS
RESERVED = 32
NFATS = 2
LABEL = b"MINIOSSD   "
VOL_ID = 0x4D494E49            # "MINI"
# FAT32 DOS date/time for 2025-01-01 00:00:00 (fixed for reproducibility).
FDATE = ((2025 - 1980) << 9) | (1 << 5) | 1
FTIME = 0

ATTR_RO, ATTR_HIDDEN, ATTR_SYS, ATTR_VOL, ATTR_DIR, ATTR_ARCH = 1, 2, 4, 8, 0x10, 0x20
ATTR_LFN = 0x0F
EOC = 0x0FFFFFFF


def collect():
    files = []  # (fat path, source path)
    mdir = os.path.join(ROOT, "machines")
    for name in sorted(os.listdir(mdir)):
        src = os.path.join(mdir, name)
        low = name.lower()
        if os.path.isfile(src) and not low.endswith(":zone.identifier") and \
                low.endswith((".tsv", ".obj", ".stl")):
            files.append((f"system/machine/{name}", src))
    for name in ("demo_box.obj", "demo_part.stl", "embedded_toolpods.tsv"):
        src = os.path.join(ROOT, "devices", name)
        if os.path.isfile(src):
            files.append((f"system/machine/{name}", src))
    files.append(("system/ui/embedded_ui.tsv", os.path.join(ROOT, "devices", "embedded_ui.tsv")))
    return files


class Node:
    def __init__(self, name, is_dir, src=None):
        self.name, self.is_dir, self.src = name, is_dir, src
        self.children = []          # ordered
        self.first_cluster = 0
        self.data = b""
        self.parent = None


def build_tree(files):
    root = Node("", True)
    for fat_path, src in files:
        parts = fat_path.split("/")
        cur = root
        for p in parts[:-1]:
            nxt = next((c for c in cur.children if c.is_dir and c.name == p), None)
            if not nxt:
                nxt = Node(p, True); nxt.parent = cur; cur.children.append(nxt)
            cur = nxt
        leaf = Node(parts[-1], False, src)
        leaf.parent = cur
        with open(src, "rb") as f:
            leaf.data = f.read()
        # Text inputs go on the card as LF whatever the checkout's line
        # endings, so the image is identical on Windows and Linux hosts.
        if src.lower().endswith((".tsv", ".obj")):
            leaf.data = leaf.data.replace(b"\r\n", b"\n")
        cur.children.append(leaf)
    return root


SFN_OK = set(b"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789$%'-_@~`!(){}^#&")


def short_name(name, taken):
    """8.3 alias (11 bytes) plus whether an LFN is needed."""
    base, dot, ext = name.rpartition(".")
    if not dot:
        base, ext = name, ""
    def clean(s):
        return bytes(c if c in SFN_OK else ord("_") for c in s.upper().encode("ascii", "replace")
                     if c not in b" .")
    b, e = clean(base), clean(ext)[:3]
    exact = (name.upper() == name and len(base) <= 8 and len(ext) <= 3 and
             b == base.encode() and e == ext.encode())
    if exact and (b.ljust(8) + e.ljust(3)) not in taken:
        sfn = b.ljust(8) + e.ljust(3)
        taken.add(sfn)
        return sfn, False
    for n in range(1, 1000000):
        tail = b"~" + str(n).encode()
        sfn = (b[:8 - len(tail)] + tail).ljust(8) + e.ljust(3)
        if sfn not in taken:
            taken.add(sfn)
            return sfn, True
    raise RuntimeError("out of short names")


def lfn_checksum(sfn):
    s = 0
    for c in sfn:
        s = (((s & 1) << 7) + (s >> 1) + c) & 0xFF
    return s


def lfn_entries(name, sfn):
    u = name.encode("utf-16-le")
    chars = [u[i:i + 2] for i in range(0, len(u), 2)]
    chars.append(b"\x00\x00")
    while len(chars) % 13:
        chars.append(b"\xff\xff")
    chunks = [chars[i:i + 13] for i in range(0, len(chars), 13)]
    csum = lfn_checksum(sfn)
    out = []
    for i, ch in enumerate(chunks, start=1):
        seq = i | (0x40 if i == len(chunks) else 0)
        e = bytes([seq]) + b"".join(ch[0:5]) + bytes([ATTR_LFN, 0, csum]) + \
            b"".join(ch[5:11]) + b"\x00\x00" + b"".join(ch[11:13])
        out.append(e)
    return list(reversed(out))   # highest sequence first on disk


def dirent(sfn, attr, cluster, size):
    return struct.pack("<11sBBBHHHHHHHI", sfn, attr, 0, 0, FTIME, FDATE, FDATE,
                       cluster >> 16, FTIME, FDATE, cluster & 0xFFFF, size)


def dir_entries(node, is_root):
    """Serialized directory, with child first_clusters already assigned."""
    ents = []
    if is_root:
        ents.append(dirent(LABEL, ATTR_VOL, 0, 0))
    else:
        parent_cl = 0 if node.parent.parent is None else node.parent.first_cluster
        ents.append(dirent(b".          ", ATTR_DIR, node.first_cluster, 0))
        ents.append(dirent(b"..         ", ATTR_DIR, parent_cl, 0))
    taken = set()
    for c in node.children:
        sfn, need_lfn = short_name(c.name, taken)
        if need_lfn:
            ents.extend(lfn_entries(c.name, sfn))
        if c.is_dir:
            ents.append(dirent(sfn, ATTR_DIR, c.first_cluster, 0))
        else:
            ents.append(dirent(sfn, ATTR_ARCH, c.first_cluster if c.data else 0, len(c.data)))
    return b"".join(ents)


def main():
    files = collect()
    root = build_tree(files)

    cluster_bytes = BPS * SPC
    fat_entries_needed = lambda fatsz: fatsz * BPS // 4
    # Size the FAT: clusters = (total - reserved - nfats*fatsz) / spc.
    fatsz = 1
    while True:
        data_sectors = TOTAL_SECTORS - RESERVED - NFATS * fatsz
        clusters = data_sectors // SPC
        if fat_entries_needed(fatsz) >= clusters + 2:
            break
        fatsz += 1
    data_start = RESERVED + NFATS * fatsz

    # Assign clusters: directories first (breadth-first), then file data.
    fat = [0] * (clusters + 2)
    fat[0], fat[1] = 0x0FFFFFF8, EOC
    next_cl = [2]

    def alloc(nbytes):
        n = max(1, -(-nbytes // cluster_bytes))
        first = next_cl[0]
        for i in range(n):
            cl = first + i
            fat[cl] = cl + 1 if i + 1 < n else EOC
        next_cl[0] += n
        return first, n

    dirs, queue = [], [root]
    while queue:
        d = queue.pop(0)
        dirs.append(d)
        queue.extend(c for c in d.children if c.is_dir)
    # Directory sizes don't depend on cluster numbers, so size with dummies.
    for d in dirs:
        size = len(dir_entries(d, d is root))
        d.first_cluster, _ = alloc(size)
    files_nodes = []
    for d in dirs:
        files_nodes.extend(c for c in d.children if not c.is_dir)
    for f in files_nodes:
        if f.data:
            f.first_cluster, _ = alloc(len(f.data))
    if next_cl[0] > clusters + 2:
        raise SystemExit("image too small for the input files")

    img = bytearray(TOTAL_SECTORS * BPS)

    def put_cluster_chain(first, payload):
        off = (data_start + (first - 2) * SPC) * BPS
        img[off:off + len(payload)] = payload

    for d in dirs:
        put_cluster_chain(d.first_cluster, dir_entries(d, d is root))
    for f in files_nodes:
        if f.data:
            put_cluster_chain(f.first_cluster, f.data)

    free = clusters - (next_cl[0] - 2)
    bs = bytearray(BPS)
    bs[0:3] = b"\xEB\x58\x90"
    bs[3:11] = b"mkimg.py"
    struct.pack_into("<HBHBHHBHHHII", bs, 11, BPS, SPC, RESERVED, NFATS, 0, 0, 0xF8, 0,
                     32, 8, 0, TOTAL_SECTORS)
    struct.pack_into("<IHHIHH", bs, 36, fatsz, 0, 0, 2, 1, 6)
    bs[64] = 0x80
    bs[66] = 0x29
    struct.pack_into("<I", bs, 67, VOL_ID)
    bs[71:82] = LABEL
    bs[82:90] = b"FAT32   "
    bs[510:512] = b"\x55\xAA"
    fsinfo = bytearray(BPS)
    struct.pack_into("<I", fsinfo, 0, 0x41615252)
    struct.pack_into("<III", fsinfo, 484, 0x61417272, free, next_cl[0])
    struct.pack_into("<I", fsinfo, 508, 0xAA550000)
    for base in (0, 6):
        img[(base + 0) * BPS:(base + 1) * BPS] = bs
        img[(base + 1) * BPS:(base + 2) * BPS] = fsinfo
        img[(base + 2) * BPS + 510:(base + 2) * BPS + 512] = b"\x55\xAA"
    fat_bytes = struct.pack(f"<{len(fat)}I", *fat)
    for n in range(NFATS):
        off = (RESERVED + n * fatsz) * BPS
        img[off:off + len(fat_bytes)] = fat_bytes

    with open(OUT, "wb") as f:
        f.write(img)
    for fat_path, src in files:
        print(f"  /{fat_path}  ({os.path.getsize(src)} bytes)")
    print(f"[mkimg.py] wrote {OUT}: {len(files)} files, {next_cl[0] - 2} clusters used, {free} free")


if __name__ == "__main__":
    main()
