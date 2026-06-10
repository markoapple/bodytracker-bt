# Windows Build Environment

This project needs CMake 3.21 or newer, a C++20 compiler, Git, and vcpkg before the presets work.

Minimum local setup:

```powershell
winget install Git.Git
winget install Kitware.CMake
winget install Microsoft.VisualStudio.2022.BuildTools
winget install Ninja-build.Ninja

git clone https://github.com/microsoft/vcpkg C:\src\vcpkg
C:\src\vcpkg\bootstrap-vcpkg.bat
[Environment]::SetEnvironmentVariable('VCPKG_ROOT', 'C:\src\vcpkg', 'User')
```

Restart the shell after setting `VCPKG_ROOT`, then verify:

```powershell
git --version
cmake --version
ninja --version
cl
$env:VCPKG_ROOT
```

`cl` must be run from a Visual Studio Developer PowerShell or from a shell where the MSVC environment has been loaded.

Then build and test:

If you are using the small/source-only archive, `models/rtmw-dw-x-l-cocktail14-384x288.onnx` and `models/rtmw3d-x-cocktail14-384x288.onnx` may be absent. CMake skips ONNX metadata inspection tests in that case; runtime inference needs the primary Cocktail model. VRChat transfer uses calculated 3D from the solver; RTMW3D is optional and disabled by default. For strict full-package validation, copy both models into `models/` and run the model asset test with `BODYTRACKER_REQUIRE_ONNX_ASSET=1`.

```powershell
cmake --preset windows-vcpkg-release
cmake --build --preset release --parallel
ctest --preset release
```

For dependency-light source validation, including the config schema contract
test, use:

```powershell
cmake --preset source-sanity
cmake --build --preset source-sanity --parallel
ctest --preset source-sanity
```

For local coverage on a GNU/Clang toolchain, configure the same source-sanity
build with coverage instrumentation:

```bash
cmake --preset source-sanity
cmake --build --preset source-sanity --parallel
ctest --preset source-sanity
python3 -m gcovr build/source-sanity --root . --filter 'src/' --xml-pretty --xml coverage.xml --html-details coverage.html
```

The repository has a `vcpkg.json` manifest. On first configure, vcpkg installs the required packages: nlohmann-json, OpenCV, ONNX Runtime, and WebView2. First install can take a while because OpenCV is large.

If CMake or vcpkg live elsewhere, set `VCPKG_ROOT` to that path. The repository does not vendor those tools.

## Common Failures

If CMake says it cannot find `opencv2/highgui.hpp`, the OpenCV highgui module was not installed or exposed through vcpkg. Re-run configure from a clean build directory after vcpkg completes package installation.

If ONNX Runtime is missing, verify that the vcpkg manifest restore completed and that CMake is using `$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake`.

If the build succeeds but runtime inference fails, that is probably not a compiler problem. The app still needs the compatible RTMW-DW-X-L Cocktail14 384x288 ONNX export at `models/rtmw-dw-x-l-cocktail14-384x288.onnx`; RTMW3D at `models/rtmw3d-x-cocktail14-384x288.onnx` is optional and off by default.


## What this does not prove

A successful configure/build/CTest run proves the native project compiles and the checked tests pass on that machine. It still does not prove tracking quality. Real quality needs model load, live camera frames, valid stereo calibration or monocular scale setup, replay/debug inspection, and tracker-space/OSC validation.

If a sandbox cannot run OpenCV, ONNX Runtime, WebView2, or the Windows camera/OSC stack, record that as a limitation instead of implying runtime correctness.

## Performance and QA infrastructure

See `docs/PERFORMANCE_QA_INFRASTRUCTURE.md` for the runtime profiler, replay performance budget gate, and deterministic checks.
