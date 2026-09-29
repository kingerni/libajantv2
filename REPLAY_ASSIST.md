# ReplayAssist integration branch

This fork carries the AJA NTV2 capture extensions used by ReplayAssist. The
parent project pins an exact commit from the `codex/replay-assist-capture`
branch, so normal consumers should clone ReplayAssist with submodules instead
of cloning this repository independently.

## ReplayAssist-specific changes

- one-shot raw BGRA snapshots from `ntv2capture`
- raw BGRA frame streaming over standard output
- memory-mapped frame delivery for the ReplayAssist preview path
- frame decimation and downsampling controls
- timecode, audio, and camera-metadata telemetry
- expanded input-source handling and Level-B/TSI capture routing

The branch starts from upstream AJA commit
`a3294fd785b2f753c361aa96a003c3e6fcca3451`.

## Build the required tools

Requirements: CMake 3.15 or newer, Ninja, a supported macOS toolchain, the AJA
desktop software/driver required by the attached hardware, and an AJA device
available for hardware validation.

```bash
cmake -S . -B build-arm64-demos-tools \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build-arm64-demos-tools --target \
  ntv2enumerateboards \
  ntv2capture \
  ntv2outputtestpattern
```

Generated build directories are intentionally ignored. Commit source changes
only, then update the pinned submodule commit in the ReplayAssist repository.

## Remotes

- `origin`: `https://github.com/kingerni/libajantv2.git`
- `upstream`: `https://github.com/aja-video/libajantv2.git`

To inspect upstream changes without moving the ReplayAssist integration branch:

```bash
git fetch upstream
git log --oneline --left-right HEAD...upstream/main
```
