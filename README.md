# Bounce Evolution for PS Vita

Native PS Vita port of **Bounce Evolution** (Rovio, Nokia N900, 2009). The original ARM32
N900 binary is rebased and statically linked into the Vita eboot; libc, GL, X11, EGL,
PulseAudio and dbus calls are replaced by shims. 60 fps, audio, accelerometer-style tilt.

Inspired by and credited to **[Thomas Perl's Maemulator](https://thp.io/2023/maemulator/)**
(see [CREDITS.md](CREDITS.md)).

**This repository contains no Rovio code or data.** You must supply the game yourself from
your own copy of `bounce_1.0.0_armel.deb`.

## Build
Requires VitaSDK, vitaGL (build with `NO_DEBUG=1 NO_SPLASHSCREEN=1 HAVE_SHADER_CACHE=1`), vitashark, mathneon.

```sh
ar x bounce_1.0.0_armel.deb && mkdir extract && tar -C extract -xf data.tar.*
export BOUNCE_ELF=$PWD/extract/opt/bounce/bin/bounce
python3 tools/make_bounce_reloc.py     # produces bounce_reloc.o (run in repo root)
mkdir build && cd build && cmake .. && make
```
Output: `BounceEvolution.vpk` (contains no game data).

## Install
1. Install the VPK with VitaShell (needs `libshacccg.suprx` and unsafe homebrew enabled).
2. Copy the *contents* of the deb's `opt/bounce/data/` (`bundles/`, `resources/`, `shaders/`)
   into `ux0:data/BounceEvolution/`.
3. First launch compiles shaders (~1 min); cached afterwards.

## Controls
Sticks = tilt, Cross = jump. SELECT recalibrate; SELECT+L flip roll; SELECT+R flip pitch;
SELECT+Up/Down tilt gain. Settings in `ux0:data/BounceEvolution/vita.cfg`.
Create `debug.txt` in that folder for verbose logging.
