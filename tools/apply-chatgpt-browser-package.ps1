param(
    [string]$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$ZipPath = "C:\Users\oobys\Downloads\Potential Junk\thingything\bodytracker-chatgpt-browser-package.zip",
    [switch]$WhatIfApply,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$Repo = (Resolve-Path $Repo).Path.TrimEnd("\")
if (-not (Test-Path -LiteralPath $ZipPath)) {
    throw "Package zip not found: $ZipPath"
}
$ZipPath = (Resolve-Path $ZipPath).Path

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$workRoot = Join-Path $env:TEMP "bodytracker-chatgpt-apply-$stamp"
$extractRoot = Join-Path $workRoot "extract"
$backupRoot = Join-Path $Repo ".chatgpt-apply-backups\$stamp"
$reportPath = Join-Path $Repo "logs\chatgpt-apply-$stamp.log"

New-Item -ItemType Directory -Force -Path $extractRoot | Out-Null
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $reportPath) | Out-Null

function Write-Step([string]$Message) {
    $line = "[$(Get-Date -Format 'HH:mm:ss')] $Message"
    Write-Host $line
    Add-Content -LiteralPath $reportPath -Value $line
}

function Get-RelativePath([string]$Base, [string]$Path) {
    $baseUri = [Uri]((Resolve-Path $Base).Path.TrimEnd("\") + "\")
    $pathUri = [Uri](Resolve-Path $Path).Path
    return [Uri]::UnescapeDataString($baseUri.MakeRelativeUri($pathUri).ToString()).Replace("/", "\")
}

function Test-SkipPackageFile([string]$RelPath) {
    $p = $RelPath.Replace("/", "\").TrimStart("\")
    if ($p -match '(^|\\)\.git(\\|$)') { return $true }
    if ($p -match '(^|\\)build(\\|$)') { return $true }
    if ($p -match '(^|\\)logs(\\|$)') { return $true }
    if ($p -match '(^|\\)__pycache__(\\|$)') { return $true }
    if ($p -match '(^|\\)\.pytest_cache(\\|$)') { return $true }
    if ($p -match '(^|\\)\.gradle(\\|$)') { return $true }
    if ($p -match '(^|\\)\.kotlin(\\|$)') { return $true }
    if ($p -match 'bodytracker\.exe\.WebView2(\\|$)') { return $true }
    if ($p -match '\.onnx$') { return $true }
    if ($p -match '\.pdb$|\.ilk$|\.obj$|\.pch$|\.ipch$|\.tlog$|\.lastbuildstate$|\.suo$|\.user$|\.log$') { return $true }
    if ($p -match '(^|\\)local\.properties$') { return $true }
    if ($p -match '^android\\FBTPhoneCamera\\app\\build\\' -and $p -notmatch '^android\\FBTPhoneCamera\\app\\build\\outputs\\apk\\debug\\app-debug\.apk$') { return $true }
    return $false
}

function Find-PackageRoot([string]$Root) {
    $bodytracker = Join-Path $Root "bodytracker"
    if ((Test-Path -LiteralPath (Join-Path $bodytracker "AGENTS.md")) -and
        (Test-Path -LiteralPath (Join-Path $bodytracker "src"))) {
        return (Resolve-Path $bodytracker).Path
    }

    $direct = Join-Path $Root "AGENTS.md"
    if ((Test-Path -LiteralPath $direct) -and (Test-Path -LiteralPath (Join-Path $Root "src"))) {
        return (Resolve-Path $Root).Path
    }

    $candidates = Get-ChildItem -LiteralPath $Root -Directory -Recurse -ErrorAction SilentlyContinue |
        Where-Object {
            (Test-Path -LiteralPath (Join-Path $_.FullName "AGENTS.md")) -and
            (Test-Path -LiteralPath (Join-Path $_.FullName "src"))
        } |
        Select-Object -First 1
    if ($candidates) {
        return $candidates.FullName
    }

    throw "Could not find package root inside zip. Expected bodytracker/AGENTS.md and bodytracker/src."
}

function Invoke-Checked([string]$Description, [scriptblock]$Command) {
    Write-Step $Description
    & $Command
    if ($LASTEXITCODE -ne 0) {
        throw "$Description failed with exit code $LASTEXITCODE"
    }
}

try {
    Write-Step "Extracting $ZipPath"
    Expand-Archive -LiteralPath $ZipPath -DestinationPath $extractRoot -Force
    $packageRoot = Find-PackageRoot $extractRoot
    Write-Step "Package root: $packageRoot"

    $changed = 0
    $new = 0
    $same = 0
    $skipped = 0
    $backedUp = 0
    $changedPaths = New-Object System.Collections.Generic.List[string]
    $newPaths = New-Object System.Collections.Generic.List[string]

    $files = Get-ChildItem -LiteralPath $packageRoot -Recurse -File
    foreach ($file in $files) {
        $rel = Get-RelativePath $packageRoot $file.FullName
        if (Test-SkipPackageFile $rel) {
            $skipped++
            continue
        }

        $dest = Join-Path $Repo $rel
        $destParent = Split-Path -Parent $dest
        if (-not (Test-Path -LiteralPath $destParent)) {
            New-Item -ItemType Directory -Force -Path $destParent | Out-Null
        }

        if (Test-Path -LiteralPath $dest) {
            $srcHash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
            $dstHash = (Get-FileHash -LiteralPath $dest -Algorithm SHA256).Hash
            if ($srcHash -eq $dstHash) {
                $same++
                continue
            }

            if (-not $WhatIfApply) {
                $backup = Join-Path $backupRoot $rel
                $backupParent = Split-Path -Parent $backup
                New-Item -ItemType Directory -Force -Path $backupParent | Out-Null
                Copy-Item -LiteralPath $dest -Destination $backup -Force
                $backedUp++
                Copy-Item -LiteralPath $file.FullName -Destination $dest -Force
            }
            $changed++
            $changedPaths.Add($rel) | Out-Null
        } else {
            if (-not $WhatIfApply) {
                Copy-Item -LiteralPath $file.FullName -Destination $dest -Force
            }
            $new++
            $newPaths.Add($rel) | Out-Null
        }
    }

    if ($WhatIfApply) {
        Write-Step "WhatIf apply complete: would_change=$changed would_add=$new identical_skipped=$same filtered_skipped=$skipped"
    } else {
        Write-Step "Apply complete: changed=$changed new=$new identical_skipped=$same filtered_skipped=$skipped backups=$backedUp"
    }
    foreach ($path in ($changedPaths | Select-Object -First 40)) {
        Write-Step "changed: $path"
    }
    foreach ($path in ($newPaths | Select-Object -First 40)) {
        Write-Step "new: $path"
    }
    if (($changedPaths.Count + $newPaths.Count) -gt 80) {
        Write-Step "Only first 40 changed and first 40 new paths were logged."
    }
    if ($backedUp -gt 0) {
        Write-Step "Backups: $backupRoot"
    }

    if ($WhatIfApply) {
        Write-Step "WhatIfApply set; not rebuilding."
        exit 0
    }

    if ($SkipBuild) {
        Write-Step "SkipBuild set; not rebuilding."
        exit 0
    }

    Push-Location $Repo
    try {
        Invoke-Checked "Checking desktop UI JavaScript" { node --check "src\ui\app\app.js" }
        Invoke-Checked "Running runtime control wiring tests" { python "tests\runtime_control_wiring_test.py" }
    } finally {
        Pop-Location
    }

    $androidDir = Join-Path $Repo "android\FBTPhoneCamera"
    $gradlew = Join-Path $androidDir "gradlew.bat"
    $gradleCandidates = @(
        $gradlew,
        "C:\Users\oobys\.gradle\manual-dists\gradle-9.4.1\bin\gradle.bat",
        "C:\Users\oobys\.gradle\manual\gradle-8.13\bin\gradle.bat",
        "C:\Users\oobys\.gradle\wrapper\dists\gradle-8.13-bin\5xuhj0ry160q40clulazy9h7d\gradle-8.13\bin\gradle.bat"
    )
    $gradle = $gradleCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if (-not $gradle) {
        $cmd = Get-Command gradle.bat -ErrorAction SilentlyContinue
        if ($cmd) {
            $gradle = $cmd.Source
        }
    }
    if (-not $gradle) {
        throw "No Gradle found. Install Gradle or open android\FBTPhoneCamera in Android Studio once, then rerun this .bat."
    }

    Push-Location $androidDir
    try {
        Invoke-Checked "Building Android debug APK with $gradle" { & $gradle --no-daemon assembleDebug }
    } finally {
        Pop-Location
    }

    Push-Location $Repo
    try {
        Invoke-Checked "Rebuilding Windows release payload" {
            powershell.exe -NoProfile -ExecutionPolicy Bypass -File "tools\rebuild-release-payload.ps1" -Repo $Repo -BuildDir "C:\tmp\bodytracker-release-build"
        }
    } finally {
        Pop-Location
    }

    $apk = Join-Path $Repo "android\FBTPhoneCamera\app\build\outputs\apk\debug\app-debug.apk"
    if (-not (Test-Path -LiteralPath $apk)) {
        throw "Android APK build completed but APK is missing: $apk"
    }

    $releaseAndroid = Join-Path $Repo "release\android"
    New-Item -ItemType Directory -Force -Path $releaseAndroid | Out-Null
    Copy-Item -LiteralPath $apk -Destination (Join-Path $releaseAndroid "FBTPhoneCamera-debug.apk") -Force

    $releasePhone = Join-Path $Repo "release\bodytracker\ui\phone"
    New-Item -ItemType Directory -Force -Path $releasePhone | Out-Null
    Copy-Item -LiteralPath $apk -Destination (Join-Path $releasePhone "app-debug.apk") -Force

    $exe = Join-Path $Repo "release\bodytracker\bodytracker.exe"
    if (-not (Test-Path -LiteralPath $exe)) {
        throw "Windows release rebuild completed but EXE is missing: $exe"
    }

    Write-Step "Done. Windows EXE: $exe"
    Write-Step "Done. Android APK: $(Join-Path $releaseAndroid 'FBTPhoneCamera-debug.apk')"
} finally {
    if (Test-Path -LiteralPath $workRoot) {
        Remove-Item -LiteralPath $workRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
}
