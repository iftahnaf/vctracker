# Build System Status

## ✅ What's Working

1. **Build Script UI** - Works perfectly
   - Command: `bash scripts/build.sh`
   - Displays menu with 8 options
   - Can select targets by number (1-8)
   - Shows help with `bash scripts/build.sh --help`

2. **Submodule Management**
   - pico-sdk ✓
   - micro-ROS SDK ✓
   - vcgimbal ✓

3. **CMake Configuration**
   - Properly configured
   - Detects all dependencies
   - Paths set correctly

## ❌ Current Issues

### Code Compilation Errors

1. **StatusLED.h** - Constructor parameter mismatch
   - Header declares no `uint pin` parameter
   - Implementation tries to use it

2. **Exception Handling** - Disabled in embedded targets
   - GPSModule.cpp line 157 uses try/catch
   - Needs `-fexceptions` flag application or code refactor

3. **ROSParser.cpp** - Missing C declarations
   - `pico_serial_transport_*` functions not visible to C++
   - Need extern "C" wrapper or proper source linking

## Next Steps to Fix

1. Fix StatusLED.h constructor signature
2. Add extern "C" to pico_uart_transport.c declarations
3. Either:
   - Replace exceptions in GPSModule with error codes, OR
   - Ensure `-fexceptions` is properly applied to all targets

## Quick Commands

```bash
# Show help
bash scripts/build.sh --help

# Select build target (interactive)
bash scripts/build.sh

# Build specific target
bash scripts/build.sh 1    # main
bash scripts/build.sh 2    # test_gps
bash scripts/build.sh 7    # all
```

