# EPROM EMU NG – Firmware 2.0rc13 (Aug-2025)  
**Firmware-only reliability pack (backward compatible)**

This release hardens the Arduino firmware while keeping **full compatibility** with your existing Python uploader (e.g., `EPROM_EMU_NG_2.0rc10.py`) and command set.

## What’s in 2.0rc13 - All Adjustments created by GPT5 at the request of KD5VMF.

- **Robust serial framing** for `:SBN` and `:DIR` (consume CR/LF explicitly).
- **Gentle receive timeout** in `:DIR` to avoid indefinite hangs if the host dies mid-transfer.
- **Autoload sanity check**: CRC16 of SPI image stored in **internal EEPROM**; autoload only if CRC/length match current SPI contents.
- **Unified, versioned config struct** in EEPROM (with CRC), **auto-migrates** from legacy fields (`mem/save/auto/hw`) on first boot.
- **`:VER`** prints concise HW/FW/CFG/SPI status; **`:READSPI`** streams SPI bytes (for spot checks).
- All changes are **firmware-only**. Your current PC tool keeps working as before.

---

## Quick Start

1. Open `EPROM_EMU_NG_FW_2.0rc13.ino` in Arduino IDE.  
2. Board: **Arduino Nano** (ATmega328P; try “Old Bootloader” if needed).  
3. Port: your emulator’s COM port.  
4. Upload.  
5. (Optional) In a serial terminal, type `:VER` + Enter to confirm.

---

## Compatibility

- Existing commands (`:EMUOFF`, `:EMUON`, `:ini…`, `:iniSPI0/1`, `:iniAuto0/1`, `:SBN`, `:DIR`, `:SPICLR`, `:help`) behave the same.
- New commands (`:VER`, `:READSPI`) are **additive**; use them only if you want the new status/readback features.

---

## Changes, with “Before / After” code

Below, “**Before (2.0rc12)**” snippets are from your original sketch; “**After (2.0rc13)**” shows what we changed/added. Ellipses `…` mean unchanged context.

### 1) Robust serial framing for `:SBN` (consume CR/LF, exact length)

**Before (2.0rc12):**
```cpp
if ( rxFrame == ":SBN" ) {
  Serial.println(F("->in"));
  unsigned int startaddr = Serial.parseInt();
  unsigned int bytecount = Serial.parseInt();
  // now we know start address and how many bytes are coming
  int transfered = Serial.readBytes(byteBuffer,bytecount+1);   // may need bytecount + 1
  // now write the buffer to SRAM
  writeBuffer(startaddr, bytecount);
  if (saveSPI) {writeBufferSPI(startaddr, bytecount);}
  Serial.println(F("ACK"));
  rxFrame = "";
  return;
}
```

**After (2.0rc13):**
```cpp
if (rxFrame == ":SBN") {
  Serial.println(F("->in"));
  unsigned int startaddr = Serial.parseInt();
  unsigned int bytecount = Serial.parseInt();

  // consume any trailing CR/LF before payload
  while (Serial.peek() == '\r' || Serial.peek() == '\n') { Serial.read(); }

  // read exactly <bytecount> bytes
  unsigned int transferred = Serial.readBytes(byteBuffer, bytecount);

  writeBuffer(startaddr, bytecount);
  if (cfg.saveSPI) {
    // mirror to SPI (same behavior as before)
    unsigned short addr = startaddr;
    setAddressSPI(addr);
    int m=0;
    for (unsigned int k=0;k<bytecount;k++){
      if (m==128){ SS_HI; delay(5); delayMicroseconds(50); setAddressSPI(addr); m=0; }
      spi_transfer(byteBuffer[k]);
      addr++; m++;
    }
    SS_HI;
  }

  Serial.println(F("ACK"));
  rxFrame = "";
  return;
}
```

**What this fixes:** stray CR/LF no longer corrupts the first payload byte; exact length is honored.

---

### 2) Robust serial framing **and** gentle timeout for `:DIR`

**Before (2.0rc12):**
```cpp
if ( rxFrame == ":DIR" ) {
  Serial.println(F(">"));
  unsigned int startaddr = Serial.parseInt();
  unsigned int bytecount = Serial.parseInt();

  // ignore an extra /n in serial buffer
  byte in_data = Serial.read();

  while (bytecount>0){
    if (Serial.available() > 0){
      in_data = Serial.read();
      writeMemoryLocation(startaddr,in_data);
      startaddr++;
      bytecount--;
    }
  }
  Serial.println(F(">"));
  rxFrame = "";
  return;
}
```

**After (2.0rc13):**
```cpp
if (rxFrame == ":DIR") {
  Serial.println(F(">"));
  unsigned int startaddr = Serial.parseInt();
  unsigned int bytecount = Serial.parseInt();

  // consume any trailing CR/LF before payload
  while (Serial.peek() == '\r' || Serial.peek() == '\n') { Serial.read(); }

  unsigned long last = millis();
  while (bytecount > 0) {
    if (Serial.available() > 0) {
      byte b = Serial.read();
      writeMemoryLocation(startaddr, b);
      startaddr++; bytecount--;
      last = millis();
    } else if (millis() - last > DIR_BYTE_TIMEOUT_MS) {
      // gentle timeout: avoid hard hang if host dies mid-transfer
      break;
    }
  }
  Serial.println(F(">"));
  rxFrame = "";
  return;
}
```

**What this fixes:** steadier uploads; no “hang forever” if the sender disappears.

---

### 3) Autoload sanity check (CRC16 over SPI image)

**Before (2.0rc12) – autoload simply ran when `autoLoad == true`:**
```cpp
// load data from SPI EEPROM if configured
if (autoLoad == true ) {
  load_SPIEEPROM(lastEPROM);
}
```

**After (2.0rc13) – autoload only if CRC/length match:**
```cpp
if (cfg.autoLoad) {
  uint32_t need = cfg.spi_len;
  if (cfg.spi_has && need>0) {
    uint16_t spi_crc_now = spi_crc_compute(need);
    if (spi_crc_now == cfg.spi_crc) {
      load_SPIEEPROM(lastEPROM);
    } else {
      Serial.println(F("Autoload skipped: SPI CRC mismatch (stale or partial image)"));
    }
  } else {
    Serial.println(F("Autoload skipped: no valid SPI image metadata"));
  }
}
```

**New helpers created in 2.0rc13 (excerpt):**
```cpp
uint16_t crc16_ccitt(const uint8_t* d, size_t n, uint16_t c=0xFFFF);
uint16_t spi_crc_compute(uint32_t len); // CRC over SPI [0..len-1]

// After :iniXX or :EMUON, refresh stored SPI CRC/length:
void finalize_spi_crc_if_needed(){
  if (!cfg.saveSPI) { cfg.spi_has = 0; write_cfg(cfg); return; }
  cfg.spi_len = memLenBytes(cfg.mem);
  uint16_t crc = spi_crc_compute(cfg.spi_len);
  cfg.spi_crc = crc;
  cfg.spi_has = 1;
  write_cfg(cfg);
  Serial.println(F("SPI image metadata updated (CRC/length)"));
}
```

**What this adds:** prevents booting with garbage after a bad write or power glitch.

---

### 4) Unified, versioned config struct in EEPROM (with CRC) + migration

**Before (2.0rc12) – scattered bytes:**
```cpp
int mem_cfg  = 0;
int save_cfg = 1;
int auto_cfg = 2;
int ver_cfg  = 10;

lastEPROM = EEPROM.read(mem_cfg);
saveSPI   = EEPROM.read(save_cfg);
autoLoad  = EEPROM.read(auto_cfg);
EEPROM.get(ver_cfg, HW_ver);
```

**After (2.0rc13) – single struct with CRC, auto-migration from legacy:**
```cpp
struct CfgV1 {
  uint16_t magic;  // 0xC0DE
  uint8_t  ver;    // 1
  uint8_t  mem;    // romXX enum
  uint8_t  saveSPI;// 0/1
  uint8_t  autoLoad;// 0/1
  char     hw[10]; // "vX.Y" + NUL
  uint32_t spi_len;// expected bytes for current mem
  uint16_t spi_crc;// CRC16 of SPI image
  uint8_t  spi_has;// 1 if CRC/len valid
  uint16_t cfg_crc;// CRC16 over struct (except cfg_crc)
};

bool read_cfg(CfgV1 &out){
  EEPROM.get(0, out);
  if (out.magic != 0xC0DE || out.ver != 1) return false;
  uint16_t want = eeprom_crc16_cfg(out);
  return (want == out.cfg_crc);
}

void migrate_legacy_to_cfg(CfgV1 &out){
  uint8_t mem = EEPROM.read(0);
  uint8_t sav = EEPROM.read(1);
  uint8_t aut = EEPROM.read(2);
  char    hw[10]; EEPROM.get(10, hw);
  if (hw[0] != 'v') { EEPROM.put(10, DEFAULT_HW_VER); for (uint8_t i=0;i<10;i++) hw[i]=DEFAULT_HW_VER[i]; }
  if (mem < rom16K || mem >= lastType) mem = rom512K;
  if (sav > 1) sav = 0; if (aut > 1) aut = 0;

  out.magic=0xC0DE; out.ver=1; out.mem=mem; out.saveSPI=sav; out.autoLoad=aut;
  for (uint8_t i=0;i<10;i++) out.hw[i]=hw[i];
  out.spi_len=0; out.spi_crc=0; out.spi_has=0; out.cfg_crc=0;
}

void write_cfg(const CfgV1 &in){
  CfgV1 tmp = in;
  tmp.magic=0xC0DE; tmp.ver=1;
  tmp.cfg_crc = eeprom_crc16_cfg(tmp);
  EEPROM.put(0, tmp);
}
```

**What this buys you:** consistent boots and cleaner recovery from uninitialized/corrupt EEPROM. Still honors all `:ini…`/`:iniSPI…`/`:iniAuto…` commands.

---

### 5) `:VER` status + `:READSPI` tool

**Before (2.0rc12):** no concise version/status command; no simple SPI readout.

**After (2.0rc13):**
```cpp
if (rxFrame == ":VER") {
  Serial.print(F("VER HW:")); Serial.print(HW_ver);
  Serial.print(F(" FW:"));    Serial.print(FW_ver);
  Serial.print(F(" MEM:"));   Serial.print(memTable[cfg.mem]);
  Serial.print(F(" SAVE:"));  Serial.print((int)cfg.saveSPI);
  Serial.print(F(" AUTO:"));  Serial.print((int)cfg.autoLoad);
  Serial.print(F(" SPI_VALID:")); Serial.print((int)cfg.spi_has);
  Serial.print(F(" SPI_LEN:"));   Serial.print(cfg.spi_len);
  Serial.print(F(" SPI_CRC:0x")); Serial.println(cfg.spi_crc, HEX);
  rxFrame = ""; return;
}
```

Help text updated:
```cpp
":VER      Print concise HW/FW/CFG/SPI status\n"
":READSPI <addr> <len>   Stream raw SPI bytes (for checks)\n"
```

*(If you want `:READSPI` to output hex, raw, or spaced bytes, say the word and we’ll format to taste.)*

---

## Operational Notes

- **No Python changes required.** You can keep using your current `:SBN`/`:DIR` flow.  
- **Autoload behavior:** If SPI metadata is missing or CRC doesn’t match, autoload is skipped with a clear message. To re-establish metadata, set the EPROM type (`:ini…`) and/or complete a save-to-SPI upload (`:iniSPI1` active) then run `:EMUON` (the firmware will compute and store CRC/length).  
- **Troubleshooting:**  
  - `:VER` quickly tells you HW/FW and whether SPI metadata is valid.  
  - If uploads ever stall mid-`:DIR`, the gentle timeout ensures the firmware returns control without a power cycle.

---

## File Layout

- `Firmware/EPROM_EMU_NG_FW_2.0rc13.ino` ← **this release**  
- (Older releases remain untouched in `OtherVersions/` for reference.)

---

## License / Credits

- Original project & hardware by **mygeekyhobby.com** (Kris Sekula), with community contributions.  
- This firmware adds reliability while preserving the original workflow and command set.

---

## Appendix A – Why CRC and not a full SPI header?

- We chose **internal EEPROM** for CRC/length so existing SPI images remain valid—no header change on the SPI chip.  
- Firmware computes/validates CRC against the **current EPROM size** (e.g., 27512 = 64 KiB).  
- This approach is fully **backward compatible** with old data already on your SPI EEPROM.

---

If you’d like a tiny patch for your Python uploader to print `:VER` and show “SPI image valid ✓ / ✗” after each upload, we can do that later without changing your normal flow.
