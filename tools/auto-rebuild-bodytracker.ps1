param(
    [string]$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$Preset = "release",
    [string]$Target = "bodytracker",
    [int]$DebounceSeconds = 1,
    [switch]$InitialBuild
)

$ErrorActionPreference = "Stop"

$Repo = (Resolve-Path $Repo).Path
$logDir = Join-Path $Repo "logs"
$logPath = Join-Path $logDir "auto-rebuild.log"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

function Write-RebuildLog([string]$Message) {
    $stamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    Add-Content -Path $logPath -Value "[$stamp] $Message"
}

$mutexName = "Global\BodytrackerAutoRebuild-$([Math]::Abs($Repo.ToLowerInvariant().GetHashCode()))"
$mutex = New-Object System.Threading.Mutex($false, $mutexName)
if (-not $mutex.WaitOne(0)) {
    Write-RebuildLog "Watcher already running for $Repo"
    return
}

function Test-WatchedPath([string]$Path) {
    if ([string]::IsNullOrWhiteSpace($Path)) {
        return $false
    }
    $exclude = "\\(build|release|generated|logs|\.git|\.pytest_cache|__pycache__|jdk17|gradle9)\\"
    $include = "\.(cpp|h|hpp|c|cc|hh|js|css|html|json|cmake|txt|md|ps1|kts|xml)$"
    $name = Split-Path -Leaf $Path
    return $Path -notmatch $exclude -and ($name -in @("CMakeLists.txt", "CMakePresets.json", "vcpkg.json") -or $name -match $include)
}

function Invoke-BodytrackerBuild([string]$Reason) {
    Write-RebuildLog "Build start: $Reason"
    Push-Location $Repo
    try {
        $output = & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $Repo "tools\rebuild-release-payload.ps1") -Repo $Repo -Preset $Preset -Target $Target 2>&1
        $exit = $LASTEXITCODE
        if ($output) {
            Add-Content -Path $logPath -Value ($output | Out-String)
        }
        if ($exit -eq 0) {
            Write-RebuildLog "Build ok: $Target via preset $Preset"
        } else {
            Write-RebuildLog "Build failed: exit $exit"
        }
    } finally {
        Pop-Location
    }
}

try {
    Write-RebuildLog "Watcher start: repo=$Repo preset=$Preset target=$Target debounce=${DebounceSeconds}s"
    if ($InitialBuild) {
        Invoke-BodytrackerBuild "initial"
    }

    $watcher = New-Object System.IO.FileSystemWatcher
    $watcher.Path = $Repo
    $watcher.IncludeSubdirectories = $true
    $watcher.NotifyFilter = [System.IO.NotifyFilters]"FileName, DirectoryName, LastWrite, Size"
    $watcher.EnableRaisingEvents = $true

    $sourcePrefix = "BodytrackerAutoRebuild.$PID"
    $registrations = @(
        Register-ObjectEvent -InputObject $watcher -EventName Changed -SourceIdentifier "$sourcePrefix.Changed"
        Register-ObjectEvent -InputObject $watcher -EventName Created -SourceIdentifier "$sourcePrefix.Created"
        Register-ObjectEvent -InputObject $watcher -EventName Deleted -SourceIdentifier "$sourcePrefix.Deleted"
        Register-ObjectEvent -InputObject $watcher -EventName Renamed -SourceIdentifier "$sourcePrefix.Renamed"
    )

    while ($true) {
        $event = Wait-Event -Timeout 86400
        if (-not $event) {
            continue
        }

        $changed = $false
        while ($event) {
            $args = $event.SourceEventArgs
            if ((Test-WatchedPath $args.FullPath) -or (Test-WatchedPath $args.OldFullPath)) {
                $changed = $true
            }
            Remove-Event -EventIdentifier $event.EventIdentifier
            $event = Wait-Event -Timeout ([Math]::Max(1, $DebounceSeconds))
        }

        if ($changed) {
            Invoke-BodytrackerBuild "source edit detected"
        }
    }
} finally {
    if ($registrations) {
        $registrations | ForEach-Object { Unregister-Event -SubscriptionId $_.Id -ErrorAction SilentlyContinue }
    }
    if ($watcher) {
        $watcher.Dispose()
    }
    $mutex.ReleaseMutex() | Out-Null
    $mutex.Dispose()
}
