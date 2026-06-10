param(
    [string]$DriverRoot = "C:\bt\release\bodytracker\steamvr_driver"
)

$ErrorActionPreference = 'Stop'
$vrpathregCandidates = @(
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe",
    "C:\Program Files\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
)
$vrpathreg = $vrpathregCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $vrpathreg) {
    throw "SteamVR vrpathreg.exe was not found. Install SteamVR or pass/register the driver root manually."
}
if (-not (Test-Path (Join-Path $DriverRoot 'driver.vrdrivermanifest'))) {
    throw "Driver manifest missing: $DriverRoot\driver.vrdrivermanifest"
}
if (-not (Test-Path (Join-Path $DriverRoot 'bin\win64\driver_bodytracker.dll'))) {
    throw "Driver DLL missing: $DriverRoot\bin\win64\driver_bodytracker.dll"
}
& $vrpathreg adddriver $DriverRoot
& $vrpathreg show
