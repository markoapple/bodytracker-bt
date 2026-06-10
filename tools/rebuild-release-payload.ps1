param(
    [string]$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$Preset = "release",
    [string]$Target = "bodytracker",
    [string]$BuildDir = (Join-Path $env:LOCALAPPDATA "bodytracker-build\release")
)

$ErrorActionPreference = "Stop"

$Repo = (Resolve-Path $Repo).Path
$sourceDir = Join-Path $BuildDir "Release"
$payloadDir = Join-Path $Repo "release\bodytracker"
$builtExe = Join-Path $sourceDir "bodytracker.exe"
$vcvars = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"

function Invoke-BuildToolsCommand([string]$Command, [string]$Failure) {
    if (Test-Path $vcvars) {
        & cmd.exe /d /c "call `"$vcvars`" x64 >nul && $Command"
    } else {
        & cmd.exe /d /c $Command
    }
    if ($LASTEXITCODE -ne 0) {
        throw "$Failure with exit code $LASTEXITCODE"
    }
}

if (-not (Test-Path (Join-Path $BuildDir "bodytracker.sln"))) {
    if (-not $env:VCPKG_ROOT) {
        throw "VCPKG_ROOT is not set; cannot configure external release build."
    }
    $buildTools = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools"
    if (Test-Path (Join-Path $buildTools "VC\Auxiliary\Build\vcvarsall.bat")) {
        $env:VCPKG_VISUAL_STUDIO_PATH = $buildTools
    }
    $toolchain = Join-Path $env:VCPKG_ROOT "scripts\buildsystems\vcpkg.cmake"
    Invoke-BuildToolsCommand "cmake -B `"$BuildDir`" -S `"$Repo`" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`"$toolchain`"" "release configure failed"
}

if (Test-Path $builtExe) {
    Remove-Item -LiteralPath $builtExe -Force
}

Push-Location $Repo
try {
    Invoke-BuildToolsCommand "cmake --build `"$BuildDir`" --config Release --target `"$Target`"" "bodytracker build failed"
} finally {
    Pop-Location
}

if (-not (Test-Path $builtExe)) {
    throw "Built executable missing: $builtExe"
}

$resolvedRepo = (Resolve-Path $Repo).Path
if (Test-Path $payloadDir) {
    $resolvedPayload = (Resolve-Path $payloadDir).Path
    if (-not $resolvedPayload.StartsWith($resolvedRepo, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to clean payload outside repo: $resolvedPayload"
    }
    Remove-Item -LiteralPath $resolvedPayload -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $payloadDir | Out-Null

Copy-Item -LiteralPath $builtExe -Destination $payloadDir -Force
Get-ChildItem -Path $sourceDir -File -Filter *.dll | Copy-Item -Destination $payloadDir -Force
Copy-Item -LiteralPath (Join-Path $sourceDir "ui") -Destination (Join-Path $payloadDir "ui") -Recurse -Force

$payloadPhoneDir = Join-Path $payloadDir "ui\phone"
$phoneSourceDir = Join-Path $Repo "src\ui\app\phone"
if (Test-Path $phoneSourceDir) {
    New-Item -ItemType Directory -Force -Path $payloadPhoneDir | Out-Null
    Get-ChildItem -LiteralPath $phoneSourceDir -Force | Copy-Item -Destination $payloadPhoneDir -Recurse -Force
}

$apk = Join-Path $Repo "android\FBTPhoneCamera\app\build\outputs\apk\debug\app-debug.apk"
if (Test-Path $apk) {
    New-Item -ItemType Directory -Force -Path $payloadPhoneDir | Out-Null
    Copy-Item -LiteralPath $apk -Destination (Join-Path $payloadPhoneDir "app-debug.apk") -Force
}

if (Test-Path $payloadPhoneDir) {
    Set-Content -LiteralPath (Join-Path $payloadPhoneDir "phone-site.json") -Encoding utf8 -Value ([ordered]@{
        enabled = $false
        url = ""
        urls = @()
        status = "disabled"
        apk = [bool](Test-Path (Join-Path $payloadPhoneDir "app-debug.apk"))
        updated = (Get-Date).ToString("o")
    } | ConvertTo-Json -Compress)
}

foreach ($folder in @("config", "calib", "models")) {
    $src = Join-Path $Repo $folder
    if (Test-Path $src) {
        Copy-Item -LiteralPath $src -Destination (Join-Path $payloadDir $folder) -Recurse -Force
    }
}

$stamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$packageTime = Get-Date
Get-ChildItem -LiteralPath $payloadDir -Recurse -File | ForEach-Object { $_.LastWriteTime = $packageTime }
Set-Content -Path (Join-Path $payloadDir "RELEASE_PAYLOAD.txt") -Value @(
    "bodytracker clean runnable payload"
    "rebuilt: $stamp"
    "source: $Repo"
    ""
    "Run setup UI:"
    ".\bodytracker.exe --setup config\default.json"
    ""
    "Run tracker:"
    ".\bodytracker.exe --run config\default.json"
)

Write-Host "Release payload ready: $payloadDir"
