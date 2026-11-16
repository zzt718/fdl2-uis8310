# fdl2-uis8310

A from-scratch **FDL2** (Flash Download Loader 2) for the **Spreadtrum UIS8310** platform:
a bare-metal implementation that erases, programs and reads NAND flash and speaks the BSL
protocol used by PC-side flashing tools.

> **Language:** **English** | [简体中文](README.zh-CN.md)

```
~27 KB · no libc · no vendor binary · software ECC · own USB device stack
```

---

## 1. What this is

When a device is put into download mode, the PC tool loads firmware in two stages:

```
PC tool ──USB──> BootROM ──> FDL1 (vendor boot downloader) ──> FDL2 (the one that does the work)
```

**FDL2 is the only stage that actually touches the flash**: it erases blocks, programs pages,
reads them back, and answers the host over the BSL protocol.

This project is an independent implementation of that stage. It is still delivered as
**a single `fdl2.bin` file**:

- drop it into the vendor package next to `dl/`, overwriting the file of the same name;
- the flashing tool, FDL1, drivers and `.bat` scripts stay untouched;
- it can also be loaded by your own flashing script (validated here with `libusb` / WinUSB).

## 2. Scope

| Item | Value |
|---|---|
| **SoC** | Spreadtrum **UIS8310** (Cortex-A55, running in 32-bit ARM state) |
| **NAND** | **UNIM UM19A8LISW**: SPI-NAND, SLC, 32 MB, 2048 B page, 128 KB block (64 pages), 78 B spare |
| **Devices** | Fengzhushou MIBOX-660X series (`EM40` battery model, `EM313A` power-bank model) |
| **Partitions** | Only 3: `KernelImg` (blocks 79–134) / `UserImg` (135–203) / `PreloadUdisk` (210–247) |
| **Not supported** | Other SoCs, other flash geometries, other partition layouts — see [docs/06-limitations.md](docs/06-limitations.md) |

> **Verify model and firmware version before flashing.** This implementation does not check
> the flash ID and does not parse the partition table. It is a purpose-built tool for known
> devices, not a universal flasher.

## 3. Why this exists

The stock FDL2 is a closed-source binary: you cannot audit it, patch it, port it to a new
flash chip, or debug it when something goes wrong. Once you need (a) full control over the
flashing flow, (b) the ability to troubleshoot a device nobody supports any more, and
(c) a codebase you can maintain for years, writing your own is the only option left.

The register configuration, command sequences and timing in this project were established
from **public controller documentation and experiments on our own hardware**; the code is
written from scratch. The ECC mapping was **derived from data produced by the device itself**
(see [docs/03-ecc.md](docs/03-ecc.md)).

## 4. Quick start

### 4.1 Build

```bash
# Requires arm-none-eabi-gcc (ARM GNU Toolchain or xPack) and python3
export PATH="/path/to/arm-none-eabi/bin:$PATH"
./build.sh            # if the exec bit is missing (common after unzip): bash build.sh
```

Output:

```
build/fdl2.bin   27 144 bytes   md5 7bce7f456e2f2ce92a846ee725efe0e1
```

That md5 is the reproducibility check: the same toolchain (GCC 15.2.1) should produce
byte-identical output. (The public release drops a few internal debug commands, so it is
572 bytes smaller than the earlier internal build; **the flashing path is identical**.)

### 4.2 Flash

Copy `build/fdl2.bin` over the `fdl2.bin` in your vendor package, then flash as usual.

> ⚠️ **Back up your device first.** BOOT1 usually still contains the stock FDL2 — it is your
> most reliable recovery path.

### 4.3 Verify

A successful flash looks like: three partitions written → device **powers off by itself** →
**press the power button** → system boots normally.

(This implementation maps BSL `0x05 NORMAL_RESET` to "power off" instead of "reboot";
see [docs/06-limitations.md](docs/06-limitations.md).)

## 5. Features

| Capability | Status | Notes |
|---|---|---|
| Own USB device stack | ✅ | Controller `0x20200000`, EP `0x85` (IN) / `0x06` (OUT), VID:PID `1782:4D00` — **no dependency on FDL1 internals** |
| BSL framing (HDLC) | ✅ | `7E` delimiter, `7D5E`/`7D5D` escaping, 16-bit one's-complement checksum |
| `0x00` CONNECT / `0x04` EXEC | ✅ | EXEC replies `0x96 + DA_INFO` to declare NAND storage |
| `0x01` START_DATA | ✅ | Both the 8-byte address form and the 76-byte partition-name form (UTF-16) |
| `0x02` MIDST_DATA | ✅ | Streams into the page buffer; up to **16 KB** per frame |
| `0x03` END_DATA | ✅ | Pads the tail page with `0xFF`, flushes the held pages 0/1 |
| `0x06` READ_FLASH | ✅ | Up to 16 KB per request, spans page boundaries |
| `0x0A` ERASE_FLASH | ✅ | Address form and partition-name form |
| `0x05` NORMAL_RESET | ⚠️ | Implemented as **power off** (reboot not solved on this platform) |
| Software ECC | ✅ | 78-byte OOB, 2 × 39-byte segments, 224-bit parity, table expanded at boot |
| MMU + D/I-cache | ✅ | 1 MB sections; 3.58 ms/page write (35 ms/page with caches off) |
| Diagnostic commands | ✅ | A set of `0xE1`–`0xFF` (read registers, read page, read OOB, set parameters, …) |

## 6. Repository layout

```
.
├── build.sh               one-command build
├── src/
│   ├── start.S            entry at 0x80100200, stack, BSS clear, jump to main
│   ├── main.c             the whole implementation (~4.5k lines, index in the file header)
│   ├── fdl2.h             types, register macros, command codes
│   ├── ecc_tab.h          generated: 224 seed rows + recurrence coefficients
│   └── linker.ld          base 0x80100000, code starts at +0x200
├── tools/
│   ├── mkimage.py         pack / unpack the DHTB header
│   └── gen_ecc_tab.py     regenerate src/ecc_tab.h from ecc_map.npz
└── docs/                  (currently written in Chinese)
    ├── 01-architecture.md
    ├── 02-nand-controller.md
    ├── 03-ecc.md
    ├── 04-usb-bsl.md
    ├── 05-building.md
    ├── 06-limitations.md
    └── 07-porting.md
```

> The detailed design documents are currently in Chinese. English translations of the most
> important parts are planned; the source comments are a mix of Chinese and English.

## 7. Where to start

| Goal | Read |
|---|---|
| Understand how it runs | [docs/01-architecture.md](docs/01-architecture.md) |
| Modify NAND access, debug bad writes | [docs/02-nand-controller.md](docs/02-nand-controller.md) |
| Understand ECC / OOB | [docs/03-ecc.md](docs/03-ecc.md) |
| Talk to it from your own tool | [docs/04-usb-bsl.md](docs/04-usb-bsl.md) |
| Build, flash, recover | [docs/05-building.md](docs/05-building.md) |
| Know the traps | [docs/06-limitations.md](docs/06-limitations.md) |
| Port to another chip | [docs/07-porting.md](docs/07-porting.md) |

## 8. Tested

| Platform | Result |
|---|---|
| Fengzhushou MIBOX-660X (power-bank model, `EM313A`) | ✅ all 3 partitions written (19.24 MB / 42 s), boots normally |
| Own flashing script (WinUSB) | ✅ read 19.24 MB / 11.8 s, write 19.24 MB / 35.3 s |
| Full-image ECC comparison, 4376 pages | ✅ byte-identical to the OOB the device itself produces |

Other models and other flash chips are **not** verified. Do not assume it works elsewhere.

## 8.5 How it differs from a general-purpose FDL2

Both are called "FDL2", but the goals are different: a general implementation targets
"one tool for many devices"; this one targets "reliable flashing of one device".

| Dimension | General-purpose FDL2 | This project |
|---|---|---|
| Goal | multi-device, multi-flash, multi-partition | **single device**, three partitions |
| Size | tens of KB (FTL / VBM / partition table / full command set) | **~27 KB** |
| Flash parameters | parameter table, looked up at runtime | **compile-time constants** |
| ECC | controller hardware ECC engine | **computed in software** (224-order recurrence) |
| Partitions | parses the partition table, consistency check | 3 fixed partitions, no parsing |
| Bad blocks / FTL | yes (virtual block mapping) | **none** (direct physical access, ECC-corrected) |
| Reset | reboot | **power off** |
| Speed | faster (~0.8 ms/page) | 3.58 ms/page |
| Portability | usually no code change for another flash | parameters and ECC must change |
| Code size | tens of thousands of lines | ~4 500 lines, single file |

**Where the two overlap, and where they do not:** the **only** shared part is the USB
controller register sequence (`0x20200000` base, endpoint descriptor layout), which follows a
public-domain project. Everything else is a different approach:

| Part | General-purpose FDL2 | This project |
|---|---|---|
| Flash parameters | `NandSpecTable` (69 × 56 B) | **no table**, compile-time constants |
| ECC | configures the controller's hardware ECC | **software**, 224-order recurrence lookup |
| Page write | through the FTL mapping | physical block direct, four 512-byte sectors, LOAD+EXECUTE each |
| Partitions | parsed from the partition table | 3 constants |
| Loading | reuses the send/recv helpers left by the boot stage | **own USB device stack**, fully independent |
| Reset | walks a PMIC command chain | writes the PMIC power-off bit directly |

## 9. Known limitations (summary)

- Flash parameters are **hard-coded** (2048 B page / 64 pages per block / 78 B OOB);
  a different chip needs code changes.
- Only 3 partitions; no partition table parsing; **no FTL and no bad-block management**.
- `reset` can only power the device off, not reboot it.
- The USB endpoints are not re-armed after the host disconnects — replug between sessions.
- No UART output; debugging is done through USB diagnostic commands.
- Write throughput is about 4× slower than the vendor tool.

Full list with causes and possible fixes: [docs/06-limitations.md](docs/06-limitations.md).

## 10. Contributing

Forks and pull requests are welcome. Before changing anything, read
[docs/06-limitations.md](docs/06-limitations.md) and the "traps" sections in the other
documents — many of them are **empirically disproved** approaches, and repeating them only
wastes your time.

Please include in a PR:

1. which layer you touched (USB / BSL / NAND / ECC / parameters);
2. the **`md5sum build/fdl2.bin`** after your change (so we can tell whether existing paths
   were affected);
3. the test conditions (model, board revision, flash chip, partitions written).

**Please do not submit code you did not write or that comes from an unknown source.**
Contributions must be your own work and licensed under the project license.

## 11. License and provenance

- **License: Unlicense (public domain)** — see [LICENSE](LICENSE). Use it, modify it, sell it,
  relicense it, no obligations.
- **Code provenance:** all code here is written for this project. Register configuration,
  command sequences and timing come from public controller documentation and from
  experiments on our own hardware; the ECC mapping is derived from data measured on the
  device.
- **Acknowledgement:** the USB controller register sequence (`0x20200000`, endpoint
  descriptor layout) follows the public-domain project
  [`ilyakurdyukov/spreadtrum_flash`](https://github.com/ilyakurdyukov/spreadtrum_flash)
  (Unlicense). This project's USB layer is a substantial rewrite on top of it: own HDLC
  framing and checksum, 32 KB receive buffer, and no dependency on the boot stage.
- No closed-source binary or fragment thereof is included; the repository contains only
  C/assembly sources, build scripts and documentation.

## 12. Disclaimer

Flashing modifies device partitions. **You are responsible for any consequences of using this
tool**; the authors provide no warranty of any kind. Back up first, validate on a small batch,
and keep the stock FDL2 as a recovery option.
