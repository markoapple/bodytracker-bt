# C:\bt debug context package

Created: 2026-06-07T05:16:17.7965285+02:00
Source: C:\bt

Included:
- Root project files: AGENTS.md, README.md, CMakeLists.txt, CMakePresets.json, vcpkg.json
- Source/config/docs/tests/tools/qa/logs
- Nested bt source snapshot, excluding nested models/build/release/binaries
- Selected build metadata: build/release/CMakeCache.txt, CTestTestfile.cmake, DartConfiguration.tcl, vcpkg-manifest-install.log, bodytracker.sln
- Model metadata only: README, deploy/detail/pipeline JSON, sha256 sidecars, generated model inventory with full ONNX hashes

Excluded on purpose:
- Full models (*.onnx)
- build/ and release/ payloads except small metadata above
- EXE/DLL/LIB/PDB/OBJ/ZIP/video/archive/big files
- Any file over 5 MiB

See _debug_context/included_files.csv and _debug_context/excluded_large_binary_model_build_release_files.csv for audit.
