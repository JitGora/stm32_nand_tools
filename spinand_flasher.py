#!/usr/bin/env python3
"""
spinand_flasher.py — SPI NAND flash WRITER (erase + program + verify) over an
STM32 running stm32-vserprog, using the same raw SPI passthrough approach as
spinand_dumper.py (flashrom's serprog protocol, S_CMD_O_SPIOP).

This is the companion tool to spinand_dumper.py: dumper reads a chip, this
tool writes a .bin dump BACK onto a chip (same one or a replacement).

*** THIS TOOL ERASES AND OVERWRITES REAL DATA. IT WILL ASK FOR CONFIRMATION
    BEFORE ANY DESTRUCTIVE OPERATION, BUT DOUBLE CHECK YOUR WIRING AND FILE
    BEFORE TYPING "yes". THERE IS NO UNDO ONCE A BLOCK IS ERASED. ***

Just run it with no arguments for an interactive menu:
    python3 spinand_flasher.py
    python3 spinand_flasher.py --port /dev/ttyACM0

Non-interactive / scriptable examples:
  Detect chip only:
    python3 spinand_flasher.py --port /dev/ttyACM0 --detect

  Erase the whole chip:
    python3 spinand_flasher.py --port /dev/ttyACM0 --erase --confirm

  Flash a .bin dump back onto the chip (erase+program+verify per block):
    python3 spinand_flasher.py --port /dev/ttyACM0 --flash -i full_dump.bin --confirm

  Flash + skip verify (faster, less safe):
    python3 spinand_flasher.py --port /dev/ttyACM0 --flash -i full_dump.bin --confirm --no-verify

  Test on a single block first (recommended before a whole-chip flash):
    python3 spinand_flasher.py --port /dev/ttyACM0 --flash -i full_dump.bin --confirm --start-page 0 --pages 64
"""

import argparse
import sys
import time

import serial

ACK = 0x06
NAK = 0x15

OIP_BIT = 0x01
WEL_BIT = 0x02
EFAIL_BIT = 0x04
PFAIL_BIT = 0x08

JEDEC_TABLE = {
    (0xEF, 0xAA, 0x21): {"name": "Winbond W25N01GVZEIG", "total_pages": 65536, "page_size": 2048,
                         "oob_size": 64, "pages_per_block": 64},
    (0xEF, 0xAA, 0x22): {"name": "Winbond W25N02GVZEIG", "total_pages": 131072, "page_size": 2048,
                         "oob_size": 64, "pages_per_block": 64},
    (0xEF, 0xAA, 0x20): {"name": "Winbond W25N512GVEIG", "total_pages": 32768, "page_size": 2048,
                         "oob_size": 64, "pages_per_block": 64},
}


# ---------------------------------------------------------------- low level

def sync(ser):
    for _ in range(8):
        ser.write(bytes([0x10]))
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


def get_status(ser, addr=0xC0):
    return spiop(ser, [0x0F, addr], 1)[0]


def wait_busy(ser, timeout=3.0):
    start = time.time()
    while get_status(ser) & OIP_BIT:
        if time.time() - start > timeout:
            raise RuntimeError("timeout waiting for chip (still busy)")
        time.sleep(0.001)
    return get_status(ser)


def read_jedec_id(ser):
    return spiop(ser, [0x9F, 0x00], 3)


def write_enable(ser):
    spiop(ser, [0x06], 0)


def read_page_main(ser, page_num, page_size):
    spiop(ser, [0x13, 0x00, (page_num >> 8) & 0xFF, page_num & 0xFF], 0)
    wait_busy(ser)
    return spiop(ser, [0x03, 0x00, 0x00, 0x00], page_size)


def read_page_oob(ser, page_num, page_size, oob_size):
    spiop(ser, [0x13, 0x00, (page_num >> 8) & 0xFF, page_num & 0xFF], 0)
    wait_busy(ser)
    col = page_size
    return spiop(ser, [0x03, (col >> 8) & 0xFF, col & 0xFF, 0x00], oob_size)


def erase_block(ser, page_in_block):
    """Erases the whole block containing page_in_block. Any page number
    within the block works -- the chip aligns down to the block boundary."""
    write_enable(ser)
    spiop(ser, [0xD8, 0x00, (page_in_block >> 8) & 0xFF, page_in_block & 0xFF], 0)
    status = wait_busy(ser)
    if status & EFAIL_BIT:
        raise RuntimeError(f"ERASE FAILED on block containing page {page_in_block} (E_FAIL bit set)")


def program_page(ser, page_num, data, page_size):
    if len(data) != page_size:
        raise ValueError(f"data must be exactly {page_size} bytes, got {len(data)}")
    write_enable(ser)
    spiop(ser, [0x02, 0x00, 0x00] + list(data), 0)  # Program Load: col addr 0x0000 + full page
    spiop(ser, [0x10, 0x00, (page_num >> 8) & 0xFF, page_num & 0xFF], 0)  # Program Execute
    status = wait_busy(ser)
    if status & PFAIL_BIT:
        raise RuntimeError(f"PROGRAM FAILED on page {page_num} (P_FAIL bit set)")


def is_bad_block_marker(oob_bytes):
    # Factory bad-block marker: first byte of spare area not 0xFF
    return oob_bytes[0] != 0xFF


# --------------------------------------------------------------- high level

def identify(jid_bytes):
    return JEDEC_TABLE.get(tuple(jid_bytes))


def cmd_detect(ser):
    jid = read_jedec_id(ser)
    print(f"JEDEC ID: {jid.hex()}")
    info = identify(jid)
    if info:
        size_mb = info["total_pages"] * info["page_size"] / (1024 * 1024)
        print(f"Chip detected: {info['name']}  ({size_mb:.0f} MB, "
              f"{info['total_pages']} pages x {info['page_size']}B + {info['oob_size']}B OOB, "
              f"{info['pages_per_block']} pages/block)")
    elif jid in (bytes([0xFF, 0xFF, 0xFF]), bytes([0x00, 0x00, 0x00])):
        print("No response — check power / wiring / clip contact.")
    else:
        print("Unrecognized JEDEC ID (chip not in built-in table).")
    return jid, info


def scan_bad_blocks(ser, num_blocks, pages_per_block, page_size, oob_size):
    print(f"Scanning {num_blocks} blocks for factory bad-block markers...")
    bad = set()
    for b in range(num_blocks):
        first_page = b * pages_per_block
        try:
            oob = read_page_oob(ser, first_page, page_size, oob_size)
            if is_bad_block_marker(oob):
                bad.add(b)
        except Exception as e:
            print(f"\nCould not check block {b} ({e}); treating as bad to be safe")
            bad.add(b)
        if b % 100 == 0:
            print(f"  scanned {b}/{num_blocks}...", end="\r")
    print(f"\nFound {len(bad)} bad block(s): {sorted(bad) if bad else 'none'}")
    return bad


def cmd_erase(ser, args, info):
    pages_per_block = info["pages_per_block"] if info else args.pages_per_block
    total_pages = args.pages if args.pages else (info["total_pages"] if info else None)
    if not total_pages or not pages_per_block:
        print("Error: need --pages and --pages-per-block for an unknown chip.", file=sys.stderr)
        sys.exit(1)

    start_page = args.start_page
    num_blocks = (total_pages + pages_per_block - 1) // pages_per_block

    print(f"\nErasing {num_blocks} block(s) starting at page {start_page} "
          f"({total_pages} pages, {pages_per_block} pages/block)")

    for i, b in enumerate(range(num_blocks)):
        page = start_page + b * pages_per_block
        retries = 3
        while retries > 0:
            try:
                erase_block(ser, page)
                break
            except Exception as e:
                retries -= 1
                print(f"\nError erasing block at page {page}: {e}, retrying ({retries} left)")
                time.sleep(0.1)
                sync(ser)
                if retries == 0:
                    print(f"FAILED to erase block at page {page} after retries — continuing")
        if i % 20 == 0 or i == num_blocks - 1:
            print(f"Erased {i + 1}/{num_blocks} blocks", end="\r")
    print(f"\nErase pass complete.")


def cmd_flash(ser, args, info):
    page_size = args.page_size if args.page_size else (info["page_size"] if info else 2048)
    pages_per_block = info["pages_per_block"] if info else args.pages_per_block
    oob_size = args.oob_size if args.oob_size else (info["oob_size"] if info else 64)

    with open(args.input, "rb") as f:
        file_data = f.read()

    if len(file_data) % page_size != 0:
        print(f"Warning: file size {len(file_data)} is not a multiple of page size {page_size}; "
              f"the last partial page will be padded with 0xFF")
    total_pages_in_file = (len(file_data) + page_size - 1) // page_size

    start_page = args.start_page
    total_pages = args.pages if args.pages else total_pages_in_file

    num_blocks = (total_pages + pages_per_block - 1) // pages_per_block
    print(f"\nFlashing '{args.input}' -> {total_pages} pages ({total_pages * page_size / 1024 / 1024:.2f} MB), "
          f"starting at page {start_page}, across {num_blocks} block(s)")

    bad_blocks = set()
    if not args.skip_bad_block_scan:
        bad_blocks = scan_bad_blocks(ser, num_blocks, pages_per_block, page_size, oob_size)

    verify_fail_pages = []
    start_time = time.time()

    for b in range(num_blocks):
        block_start_page = start_page + b * pages_per_block

        if b in bad_blocks:
            print(f"\nSkipping block {b} (page {block_start_page}) — marked bad, not writing to it")
            continue

        retries = 3
        while retries > 0:
            try:
                erase_block(ser, block_start_page)
                break
            except Exception as e:
                retries -= 1
                print(f"\nErase error on block {b}: {e}, retrying ({retries} left)")
                time.sleep(0.1)
                sync(ser)
                if retries == 0:
                    print(f"FAILED to erase block {b} — skipping this block")
                    continue

        for pp in range(pages_per_block):
            page = block_start_page + pp
            if page - start_page >= total_pages:
                break
            file_off = (page - start_page) * page_size
            chunk = file_data[file_off:file_off + page_size]
            if len(chunk) < page_size:
                chunk = chunk + b"\xff" * (page_size - len(chunk))

            retries = 3
            while retries > 0:
                try:
                    program_page(ser, page, chunk, page_size)
                    break
                except Exception as e:
                    retries -= 1
                    print(f"\nProgram error on page {page}: {e}, retrying ({retries} left)")
                    time.sleep(0.1)
                    sync(ser)
                    if retries == 0:
                        print(f"FAILED to program page {page}")

            if not args.no_verify:
                try:
                    readback = read_page_main(ser, page, page_size)
                    if readback != chunk:
                        verify_fail_pages.append(page)
                        print(f"\nVERIFY MISMATCH on page {page}")
                except Exception as e:
                    verify_fail_pages.append(page)
                    print(f"\nVERIFY ERROR on page {page}: {e}")

        elapsed = time.time() - start_time
        pct = (b + 1) / num_blocks * 100
        print(f"Block {b + 1}/{num_blocks} ({pct:.1f}%) done | elapsed {elapsed / 60:.1f} min", end="\r")

    print(f"\n\nFlash complete.")
    if not args.no_verify:
        if verify_fail_pages:
            print(f"WARNING: {len(verify_fail_pages)} page(s) failed verification: {verify_fail_pages[:20]}"
                  f"{' ...' if len(verify_fail_pages) > 20 else ''}")
        else:
            print("All pages verified OK.")
    if bad_blocks:
        print(f"{len(bad_blocks)} factory-bad block(s) were skipped: {sorted(bad_blocks)}")


# ----------------------------------------------------------------- prompts

def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    val = input(f"{prompt}{suffix}: ").strip()
    return val if val else default


def ask_yes_no(prompt, default_yes=False):
    d = "Y/n" if default_yes else "y/N"
    val = input(f"{prompt} ({d}): ").strip().lower()
    if not val:
        return default_yes
    return val.startswith("y")


def type_to_confirm(word):
    val = input(f"Type '{word}' to confirm (this is destructive): ").strip()
    return val == word


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
        print("  2) Erase whole chip")
        print("  3) Erase a single block (safe test)")
        print("  4) Flash a .bin file onto the chip (erase+program+verify)")
        print("  5) Flash only a test block from a .bin file (safe test)")
        print("  6) Exit")
        choice = ask("Choice", "6")

        if choice == "1":
            jid, info = cmd_detect(ser)
            continue
        if choice == "6":
            print("Bye.")
            return
        if choice not in ("2", "3", "4", "5"):
            print("Not a valid option.")
            continue

        if not info:
            print("Chip not in built-in table — you'll need to know its page size / pages-per-block.")

        class Args:
            pass
        args = Args()
        args.page_size = info["page_size"] if info else int(ask("Page size", "2048"))
        args.oob_size = info["oob_size"] if info else int(ask("OOB size", "64"))
        args.pages_per_block = info["pages_per_block"] if info else int(ask("Pages per block", "64"))
        args.skip_bad_block_scan = False
        args.no_verify = False

        if choice == "2":
            args.start_page = 0
            args.pages = info["total_pages"] if info else int(ask("Total pages", "65536"))
            print(f"\n*** This will ERASE THE ENTIRE CHIP ({args.pages} pages). ALL DATA WILL BE LOST. ***")
            if type_to_confirm("ERASE"):
                cmd_erase(ser, args, info)
            else:
                print("Cancelled.")

        elif choice == "3":
            block_num = int(ask("Block number to erase", "0"))
            args.start_page = block_num * args.pages_per_block
            args.pages = args.pages_per_block
            print(f"\nThis will erase block {block_num} (pages {args.start_page}..{args.start_page + args.pages_per_block - 1}).")
            if ask_yes_no("Proceed?", default_yes=False):
                cmd_erase(ser, args, info)
            else:
                print("Cancelled.")

        elif choice == "4":
            args.input = ask("Path to .bin file to flash", None)
            if not args.input:
                print("No file given, cancelled.")
                continue
            args.start_page = 0
            args.pages = 0  # sized from file
            args.no_verify = not ask_yes_no("Verify every page after writing? (recommended)", default_yes=True)
            args.skip_bad_block_scan = not ask_yes_no("Scan for factory bad blocks first? (recommended)", default_yes=True)
            print(f"\n*** This will ERASE AND OVERWRITE THE ENTIRE CHIP with the contents of "
                  f"'{args.input}'. THIS CANNOT BE UNDONE. ***")
            if type_to_confirm("FLASH"):
                cmd_flash(ser, args, info)
            else:
                print("Cancelled.")

        elif choice == "5":
            args.input = ask("Path to .bin file to flash", None)
            if not args.input:
                print("No file given, cancelled.")
                continue
            block_num = int(ask("Block number to test-flash", "0"))
            args.start_page = block_num * args.pages_per_block
            args.pages = args.pages_per_block
            args.no_verify = False
            args.skip_bad_block_scan = False
            print(f"\nThis will erase+program+verify ONLY block {block_num} "
                  f"(pages {args.start_page}..{args.start_page + args.pages_per_block - 1}) "
                  f"from the start of '{args.input}'.")
            if ask_yes_no("Proceed?", default_yes=False):
                cmd_flash(ser, args, info)
            else:
                print("Cancelled.")


# --------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="SPI NAND flasher (erase/program/verify) via STM32 stm32-vserprog")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1500000)
    ap.add_argument("--detect", action="store_true")
    ap.add_argument("--erase", action="store_true", help="Erase blocks (whole chip by default)")
    ap.add_argument("--flash", action="store_true", help="Erase+program+verify from a .bin file")
    ap.add_argument("-i", "--input", help="Input .bin file for --flash")
    ap.add_argument("--start-page", type=int, default=0)
    ap.add_argument("--pages", type=int, default=0, help="Number of pages (default: whole chip / whole file)")
    ap.add_argument("--page-size", type=int, default=0)
    ap.add_argument("--oob-size", type=int, default=0)
    ap.add_argument("--pages-per-block", type=int, default=0)
    ap.add_argument("--no-verify", action="store_true", help="Skip read-back verification after programming (faster, less safe)")
    ap.add_argument("--skip-bad-block-scan", action="store_true", help="Don't scan for factory bad blocks before flashing")
    ap.add_argument("--confirm", action="store_true", help="Required to actually run --erase/--flash non-interactively")
    args = ap.parse_args()

    if not any([args.detect, args.erase, args.flash]):
        interactive_mode(args.port, args.baud)
        return

    if (args.erase or args.flash) and not args.confirm:
        ap.error("--erase/--flash require --confirm as an explicit safety check (this is destructive)")
    if args.flash and not args.input:
        ap.error("--flash requires -i/--input")

    ser = serial.Serial(args.port, args.baud, timeout=2)
    if not sync(ser):
        print("Sync failed — check port/USB connection.", file=sys.stderr)
        sys.exit(1)

    jid, info = cmd_detect(ser)

    if args.erase:
        cmd_erase(ser, args, info)
    elif args.flash:
        cmd_flash(ser, args, info)


if __name__ == "__main__":
    main()
