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
    throw "SteamVR vrpathreg.exe was not found."
}
& $vrpathreg removedriver $DriverRoot
& $vrpathreg show
