#!/usr/bin/env python3
"""
spinand_dumper.py — SPI NAND flash reader over an STM32 running stm32-vserprog
(flashrom "serprog" protocol), using raw SPI passthrough (S_CMD_O_SPIOP).

Works around the fact that flashrom itself does not support SPI NAND chips
(they need a two-step Page Data Read + Read Cache sequence, unlike NOR flash).

Just run it with no arguments for an interactive menu (auto-detects the chip,
suggests a filename based on the chip name and size, lets you pick a custom
one, and asks before starting a dump):

    python3 spinand_dumper.py
    python3 spinand_dumper.py --port /dev/ttyACM0   # if not the default port

Usage examples (non-interactive / scriptable):
  Detect chip only:
    python3 spinand_dumper.py --port /dev/ttyACM0 --detect

  Dump whole chip (auto-sized from JEDEC ID if known):
    python3 spinand_dumper.py --port /dev/ttyACM0 --dump -o full_dump.bin

  Dump only first 50 pages (~100KB) for a quick test:
    python3 spinand_dumper.py --port /dev/ttyACM0 --dump -o test.bin --pages 50

  Dump including OOB/spare area (main+spare interleaved per page):
    python3 spinand_dumper.py --port /dev/ttyACM0 --dump -o full_with_oob.bin --with-oob

  Force page count / page size for an unknown/unlisted chip:
    python3 spinand_dumper.py --port /dev/ttyACM0 --dump -o out.bin --pages 65536 --page-size 2048
"""

import argparse
import sys
import time

import serial

ACK = 0x06
NAK = 0x15

# Known SPI NAND chips by (manufacturer_id, device_id_1, device_id_2)
# total_pages = whole-chip page count, page_size = main data area, oob_size = spare area
JEDEC_TABLE = {
    (0xEF, 0xAA, 0x21): {"name": "Winbond W25N01GVZEIG", "total_pages": 65536, "page_size": 2048, "oob_size": 64},
    (0xEF, 0xAA, 0x22): {"name": "Winbond W25N02GVZEIG", "total_pages": 131072, "page_size": 2048, "oob_size": 64},
    (0xEF, 0xAA, 0x20): {"name": "Winbond W25N512GVEIG", "total_pages": 32768, "page_size": 2048, "oob_size": 64},
}


def sync(ser):
    for _ in range(8):
        ser.write(bytes([0x10]))  # S_CMD_SYNCNOP
        ser.flush()
        time.sleep(0.05)
        resp = ser.read(64)
        if resp.endswith(bytes([NAK, ACK])):
            return True
    return False


def spiop(ser, write_bytes, read_len):
    ser.reset_input_buffer()
    slen = len(write_bytes)
    hdr = bytes([0x13]) + slen.to_bytes(3, "little") + read_len.to_bytes(3, "little")
    ser.write(hdr + bytes(write_bytes))
    ser.flush()
    ack = ser.read(1)
    if ack != bytes([ACK]):
        raise RuntimeError(f"NAK/no response from programmer, got {ack!r}")
    data = ser.read(read_len)
    if len(data) != read_len:
        raise RuntimeError(f"short read: got {len(data)} of {read_len} bytes")
    return data


def get_status(ser, addr):
    return spiop(ser, [0x0F, addr], 1)[0]


def wait_busy(ser, timeout=2.0):
    start = time.time()
    while get_status(ser, 0xC0) & 0x01:  # OIP bit
        if time.time() - start > timeout:
            raise RuntimeError("timeout waiting for chip to finish operation")
        time.sleep(0.001)


def read_jedec_id(ser):
    return spiop(ser, [0x9F, 0x00], 3)


def read_page_main(ser, page_num, page_size):
    spiop(ser, [0x13, 0x00, (page_num >> 8) & 0xFF, page_num & 0xFF], 0)
    wait_busy(ser)
    return spiop(ser, [0x03, 0x00, 0x00, 0x00], page_size)


def read_page_oob(ser, page_num, page_size, oob_size):
    """Reads ONLY the OOB/spare area, as its own single page-load + single
    cache-read cycle (never combined with a main-data read in the same
    page load -- some serprog firmwares choke on two back-to-back cache
    reads after one page load, even though each individually is fine)."""
    spiop(ser, [0x13, 0x00, (page_num >> 8) & 0xFF, page_num & 0xFF], 0)
    wait_busy(ser)
    col = page_size
    return spiop(ser, [0x03, (col >> 8) & 0xFF, col & 0xFF, 0x00], oob_size)


def identify(jid_bytes):
    key = tuple(jid_bytes)
    return JEDEC_TABLE.get(key)


def cmd_detect(ser):
    jid = read_jedec_id(ser)
    print(f"JEDEC ID: {jid.hex()}")
    info = identify(jid)
    if info:
        size_mb = info["total_pages"] * info["page_size"] / (1024 * 1024)
        print(f"Chip detected: {info['name']}  ({size_mb:.0f} MB, "
              f"{info['total_pages']} pages x {info['page_size']}B + {info['oob_size']}B OOB)")
    elif jid in (bytes([0xFF, 0xFF, 0xFF]), bytes([0x00, 0x00, 0x00])):
        print("No response — check power / wiring / clip contact.")
    else:
        print("Unrecognized JEDEC ID (chip not in built-in table).")
        print("You can still dump it manually with --pages and --page-size.")
    return jid, info


def dump_loop(ser, start_page, total_pages, read_func, item_size, out_path, label):
    """Runs a page-read loop with retry+resync, writing item_size bytes/page
    to out_path. read_func(ser, page_num) -> bytes of length item_size."""
    print(f"\n[{label}] Reading {total_pages} pages -> {out_path} "
          f"({total_pages * item_size / 1024 / 1024:.2f} MB)")
    start_time = time.time()
    with open(out_path, "wb") as f:
        for i, p in enumerate(range(start_page, start_page + total_pages)):
            retries = 3
            while retries > 0:
                try:
                    data = read_func(ser, p)
                    f.write(data)
                    break
                except Exception as e:
                    retries -= 1
                    print(f"\n[{label}] Error on page {p}: {e}, retrying ({retries} left)")
                    time.sleep(0.1)
                    if not sync(ser):
                        print(f"[{label}] Resync failed after error on page {p}")
                    if retries == 0:
                        print(f"[{label}] Giving up on page {p}, writing zero-filled placeholder")
                        f.write(bytes(item_size))

            if i % 500 == 0 or i == total_pages - 1:
                f.flush()
                elapsed = time.time() - start_time
                done = i + 1
                pct = done / total_pages * 100
                done_bytes = done * item_size
                rate = done_bytes / elapsed / 1024 if elapsed > 0 else 0
                eta = (total_pages - done) * (elapsed / done) if done > 0 else 0
                print(f"[{label}] {done}/{total_pages} ({pct:.1f}%) | "
                      f"{done_bytes / 1024 / 1024:.1f}MB | {rate:.1f} KB/s | "
                      f"ETA {eta / 60:.1f} min", end="\r")

    total_bytes = total_pages * item_size
    print(f"\n[{label}] Done. Saved {total_bytes} bytes "
          f"({total_bytes / 1024 / 1024:.2f} MB) to {out_path}")


def cmd_dump(ser, args):
    jid, info = cmd_detect(ser)

    if args.page_size:
        page_size = args.page_size
    elif info:
        page_size = info["page_size"]
    else:
        print("Error: unknown chip and no --page-size given.", file=sys.stderr)
        sys.exit(1)

    oob_size = args.oob_size if args.oob_size else (info["oob_size"] if info else 64)

    if args.pages:
        total_pages = args.pages
    elif info:
        total_pages = info["total_pages"]
    else:
        print("Error: unknown chip and no --pages given.", file=sys.stderr)
        sys.exit(1)

    start_page = args.start_page

    # Pass 1: main data -- always done, this is the part that actually
    # restores file/firmware content.
    main_out = args.output
    dump_loop(ser, start_page, total_pages, lambda s, p: read_page_main(s, p, page_size),
              page_size, main_out, "MAIN")

    if args.with_oob:
        # Pass 2: OOB/spare, as its own fully separate pass -- see
        # read_page_oob() docstring for why it's not combined with pass 1.
        if main_out.lower().endswith(".bin"):
            oob_out = main_out[:-4] + ".oob"
        else:
            oob_out = main_out + ".oob"
        dump_loop(ser, start_page, total_pages, lambda s, p: read_page_oob(s, p, page_size, oob_size),
                  oob_size, oob_out, "OOB")
        print(f"\nRestore note: '{main_out}' has the main data (2048B/page) and "
              f"'{oob_out}' has the matching OOB/spare data (64B/page), same page order. "
              f"Keep both if you want a complete original-state backup for restoring later.")


def safe_name(name):
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in name)


def suggest_filename(info, with_oob):
    if info:
        base = safe_name(info["name"])
        size_mb = int(info["total_pages"] * info["page_size"] / (1024 * 1024))
        suffix = "_with_oob" if with_oob else ""
        return f"{base}_{size_mb}MB{suffix}.bin"
    return f"nand_dump{'_with_oob' if with_oob else ''}.bin"


def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    val = input(f"{prompt}{suffix}: ").strip()
    return val if val else default


def ask_yes_no(prompt, default_yes=True):
    d = "Y/n" if default_yes else "y/N"
    val = input(f"{prompt} ({d}): ").strip().lower()
    if not val:
        return default_yes
    return val.startswith("y")


def interactive_mode(port, baud):
    ser = serial.Serial(port, baud, timeout=2)
    if not sync(ser):
        print(f"Sync failed on {port} — check the USB connection / port name.", file=sys.stderr)
        sys.exit(1)

    print("Detecting chip...")
    jid, info = cmd_detect(ser)

    while True:
        print("\nWhat would you like to do?")
        print("  1) Detect chip again")
        print("  2) Dump whole chip")
        print("  3) Dump a custom range of pages (e.g. a quick test)")
        print("  4) Exit")
        choice = ask("Choice", "2")

        if choice == "1":
            jid, info = cmd_detect(ser)
            continue

        if choice == "4":
            print("Bye.")
            return

        if choice not in ("2", "3"):
            print("Not a valid option.")
            continue

        if not info:
            print("Chip isn't in the built-in JEDEC table, so size must be entered manually.")
            page_size = int(ask("Page size in bytes", "2048"))
            oob_size = int(ask("OOB/spare size in bytes", "64"))
            default_total_pages = 65536
        else:
            page_size = info["page_size"]
            oob_size = info["oob_size"]
            default_total_pages = info["total_pages"]

        if choice == "2":
            start_page = 0
            total_pages = default_total_pages
        else:
            start_page = int(ask("Start page", "0"))
            total_pages = int(ask("Number of pages to read", "50"))

        with_oob = ask_yes_no("Include OOB/spare bytes in the dump?", default_yes=False)

        default_name = suggest_filename(info, with_oob)
        out_name = ask("Output filename", default_name)

        class Args:
            pass
        args = Args()
        args.output = out_name
        args.pages = total_pages
        args.start_page = start_page
        args.page_size = page_size
        args.oob_size = oob_size
        args.with_oob = with_oob

        confirm = ask_yes_no(
            f"About to read {total_pages} pages ({total_pages * page_size / 1024 / 1024:.2f} MB main data"
            + (f" + {total_pages * oob_size / 1024 / 1024:.2f} MB OOB in a separate file" if with_oob else "")
            + f") to '{out_name}'. Proceed?",
            default_yes=True,
        )
        if confirm:
            cmd_dump(ser, args)
        else:
            print("Cancelled.")


def main():
    ap = argparse.ArgumentParser(description="SPI NAND reader via STM32 stm32-vserprog raw SPI passthrough")
    ap.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=1500000, help="Baud rate (default: 1500000, ignored over USB-CDC)")
    ap.add_argument("--detect", action="store_true", help="Only detect/identify the chip, then exit")
    ap.add_argument("--dump", action="store_true", help="Dump the chip (or a range of pages)")
    ap.add_argument("-o", "--output", help="Output filename for --dump (required with --dump)")
    ap.add_argument("--pages", type=int, default=0, help="Number of pages to read (default: whole chip if known)")
    ap.add_argument("--start-page", type=int, default=0, help="First page number to read (default: 0)")
    ap.add_argument("--page-size", type=int, default=0, help="Override main data page size in bytes")
    ap.add_argument("--oob-size", type=int, default=0, help="Override OOB/spare size in bytes")
    ap.add_argument("--with-oob", action="store_true", help="Also read and save OOB/spare bytes per page")
    args = ap.parse_args()

    if not args.detect and not args.dump:
        # No mode flags given -> interactive menu mode
        interactive_mode(args.port, args.baud)
        return

    if args.dump and not args.output:
        ap.error("--dump requires -o/--output filename")

    ser = serial.Serial(args.port, args.baud, timeout=2)
    if not sync(ser):
        print("Sync failed — check port/USB connection.", file=sys.stderr)
        sys.exit(1)

    if args.detect and not args.dump:
        cmd_detect(ser)
    elif args.dump:
        cmd_dump(ser, args)


if __name__ == "__main__":
    main()
