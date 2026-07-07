# loop_charger_inverter

Inverter monitoring daemon for Linux. Supports x86 and ARM (aarch64) targets.

## Build dependencies

### x86

```bash
sudo apt install gcc libjson-c-dev
```

### ARM cross-compilation

```bash
sudo apt install gcc-aarch64-linux-gnu
```

The json-c arm64 library is downloaded automatically from ports.ubuntu.com
on the first `make ARCH=arm` run and cached in `build/deps/` (gitignored).
No other setup or system changes are required.

## Build

```bash
# x86 (default)
make ARCH=x86

# ARM – json-c is fetched automatically on first run
make ARCH=arm

# Debug build
make ARCH=x86 DEBUG=DEBUG

# Clean compiled objects and output binaries
make clean

# Rebuild from scratch
make ARCH=<x86|arm> rebuild
```

## Custom SDK / sysroot

### ARM

Override `JSONC_ARM_LIB` and `JSONC_ARM_INC` to use a pre-built SDK instead
of the auto-downloaded package:

```bash
make ARCH=arm \
     JSONC_ARM_LIB=/opt/my-sdk/lib/libjson-c.a \
     JSONC_ARM_INC=/opt/my-sdk/include
```

### x86

Override `PKG_CONFIG` to point at a custom wrapper:

```bash
make ARCH=x86 PKG_CONFIG=/opt/my-sdk/bin/pkg-config
```
