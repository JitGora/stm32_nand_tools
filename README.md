# stm32_spinand_tools

Read and write SPI NAND flash chips (Winbond W25N family) using an STM32
Bluepill running [stm32-vserprog](https://github.com/dword1511/stm32-vserprog).
Tested with the W25N01GVZEIG.

flashrom doesn't support SPI NAND, so these scripts talk to the chip
directly over stm32-vserprog's raw SPI passthrough instead.

## Files

**spinand_dumper.py**
- Auto-detects chip via JEDEC ID
- Dumps main data to `.bin`
- Optional OOB/spare dump to separate `.oob` file
- Interactive menu or CLI flags

**spinand_flasher.py**
- Auto-detects chip via JEDEC ID
- Erase (whole chip or single block)
- Flash: erase + program + verify from a `.bin` file
- Bad-block scan (skips writing to factory-bad blocks)
- Verify chip against a reference `.bin` (read-only)
- Health check: ECC status + bad blocks (read-only, no file needed)
- Interactive menu or CLI flags

## Wiring (Bluepill hardware SPI1)

```
NAND (SOP8)               Stm32f103 Bluepill
  ┌──────────┐            ┌──────────────────┐
  │ CS#    1 │──────────→ │ PA4  (NSS / CS)  │
  │ DO     2 │──────────→ │ PA6  (MISO)      │
  │ WP#    3 │──────────→ │ 3.3V             │
  │ GND    4 │──────────→ │ GND              │
  │ DI     5 │──────────→ │ PA7  (MOSI)      │
  │ CLK    6 │──────────→ │ PA5  (SCK)       │
  │ HOLD#  7 │──────────→ │ 3.3V             │
  │ VCC    8 │──────────→ │ 3.3V             │
  └──────────┘            └──────────────────┘
```

## Setup
```bash
pip install pyserial
python3 spinand_dumper.py
python3 spinand_flasher.py
```

