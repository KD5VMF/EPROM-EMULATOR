# EPROM EMU NG – Firmware **4.0rc4c** (Aug‑2025)
**FW_4‑compatible reliability pack — backward compatible with the classic Python uploader**

This README documents the Arduino sketch tagged **FW 4.0rc4c** that adds the same reliability features introduced in the 2.0rc13 line, but packaged for the **FW_4** tree. It is drop‑in for your existing hardware and **does not require changes** to your Python tool (`EPROM_EMU_NG_2.0rc10.py`).

---

## What’s new in 4.0rc4c

- **Robust serial framing** for `:SBN` and `:DIR` (explicitly consume CR/LF so payload starts cleanly).
- **Gentle receive timeout** in `:DIR` to prevent indefinite hangs if the host stops sending mid‑transfer.
- **Autoload sanity check**: CRC16 of the SPI image + expected length stored in **internal EEPROM**; autoload only proceeds when they **match** the current SPI contents.
- **Unified, versioned config struct** (with CRC) in EEPROM. On first boot it **auto‑migrates** from legacy fields (`mem`/`save`/`auto`/`hw`), so no manual reset is needed.
- **`:VER`** prints concise HW/FW/CFG/SPI status.
- **`:READSPI <addr> <len>`** streams SPI bytes in hex for spot‑checks.
- Fully **backward compatible** with: `:EMUOFF`, `:EMUON`, `:ini…`, `:iniSPI0/1`, `:iniAuto0/1`, `:SPICLR`, `:help`, and the classic upload flows (`:SBN`, `:DIR`).

> Tip: These changes are **firmware‑only**. Your `.hex` / `.bin` uploads and the UI output from the Python tool remain unchanged (you’ll just see `FW: 4.0rc4c` in the banner).

---

## Quick start

1. Open **`EPROM_EMU_NG_FW_4.0rc4c.ino`** in Arduino IDE.
2. Board: **Arduino Nano (ATmega328P)**. If uploads fail, try “Old Bootloader”.
3. Select your COM port and **Upload**.
4. Open Serial Monitor (115200 baud) and send `:VER` + Enter to confirm status.

Expected output includes something like:
```
VER HW:v2.1 FW:4.0rc4c MEM:27512 SAVE:1 AUTO:1 SPI_VALID:1 SPI_LEN:65536 SPI_CRC:0xABCD
```

---

## Compatibility

- **Python uploader (`EPROM_EMU_NG_2.0rc10.py`)**: works as‑is. Both `:SBN` and `:DIR` are supported. No flags or script edits required.
- **SPI image format**: unchanged. We store CRC/length **in internal EEPROM**, not in the SPI chip, so **existing SPI images** remain valid.
- **Older configs**: on first boot, legacy bytes at EEPROM addresses 0/1/2/10 are migrated into the new config block automatically.

---

## New/updated commands

- `:VER` — print concise HW/FW/CFG/SPI status.
- `:READSPI <addr> <len>` — stream raw SPI contents in hex (16 bytes per line) starting at `<addr>` for `<len>` bytes.
- `:SBN <start> <len>` then payload — now consumes CR/LF before payload and reads **exactly** `<len>` bytes.
- `:DIR <start> <len>` then payload — consumes CR/LF and enforces a **per‑byte timeout** (default 2s) so a dead host won’t hang the device.

All existing commands remain, including `:ini16`, `:ini32`, `:ini64`, `:iniE64`, `:ini128`, `:ini256`, `:iniE256`, `:ini512`, `:iniSPI0/1`, `:iniAuto0/1`, `:SPICLR`, `:help`.

---

## Autoload & SPI metadata – how it works

- When **Save to SPI** is **ON** (`:iniSPI1`) and you upload, the firmware mirrors incoming bytes to the SPI EEPROM (same as before).
- After setting an EPROM type (`:ini…`) or when `:EMUON` is issued, the firmware:
  1. Computes **CRC16‑CCITT** over the expected range for the selected device (e.g., 64 KiB for 27512 / 28C256).
  2. Saves **CRC + length** into the unified config in internal EEPROM.
- On startup or pushbutton autoload, the firmware recomputes the CRC from SPI and **only loads to SRAM if it matches** the stored CRC/length.
- If the metadata is missing or mismatched, autoload is **skipped** with a clear status message. Use `:VER` to see why.

---

## Typical workflows

### Fresh upload with SPI mirroring
1. `:ini512` (or desired `:ini…`)  
2. `:iniSPI1` (enable mirroring)  
3. Run your PC tool to upload (`:SBN`/`:DIR`).  
4. `:EMUON` (finalizes CRC/length in EEPROM).  
5. Optional: `:VER` to confirm `SPI_VALID:1`.

### Verify what’s in the SPI EEPROM
- `:READSPI 0 64` — dump first 64 bytes in hex.
- `:VER` — check `SPI_LEN` and `SPI_CRC` quickly.

### Recover from a corrupt/partial SPI image
- `:SPICLR` — erase SPI chip (and clear `SPI_VALID`).  
- Re‑upload with `:iniSPI1` enabled and `:EMUON` to refresh metadata.

---

## Differences vs. stock FW_4 (summary)

- Safer serial handling (payload can’t be thrown off by stray CR/LF).
- Won’t hang forever if a host dies mid‑`:DIR` transfer.
- Autoload is **guarded** by CRC/length to avoid booting garbage.
- Single, CRC‑protected **config block** in EEPROM with auto‑migration.
- Extra tooling: `:VER` and `:READSPI`.

---

## Known limitations

- The CRC/length is stored in **internal EEPROM** (not inside the SPI), so if you clone just the SPI chip you’ll need to regenerate metadata by running `:ini…` and `:EMUON` (or re‑upload with `:iniSPI1`).
- `:READSPI` prints hex in 16‑byte lines; if you want a raw binary stream or different formatting, it’s easy to tweak.

---

## File layout

- `Firmware/EPROM_EMU_NG_FW_4.0rc4c.ino` — this firmware
- `OtherVersions/` — your prior firmware for reference

---

## Credits & license

Original project & hardware by **mygeekyhobby.com** (Kris Sekula), with community contributions.  
This reliability pack preserves the original workflow while making failures less likely and easier to diagnose.
