param(
    [string]$Repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$Branch = "main",
    [int]$IntervalSeconds = 60,
    [switch]$Once
)

$ErrorActionPreference = "Stop"

function Write-SyncLog {
    param([string]$Message)
    $logDir = Join-Path $Repo "logs"
    New-Item -ItemType Directory -Force -Path $logDir | Out-Null
    $stamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    Add-Content -Path (Join-Path $logDir "git-auto-sync.log") -Value "[$stamp] $Message"
}

function Invoke-Git {
    $oldErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $output = & git -C $Repo @args 2>&1
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldErrorActionPreference
    }
    if ($code -ne 0) {
        throw "git $($args -join ' ') failed with exit code ${code}: $output"
    }
    return $output
}

function Sync-Once {
    if (-not (Test-Path (Join-Path $Repo ".git"))) {
        throw "Not a Git repository: $Repo"
    }

    $lockPath = Join-Path $Repo ".git\auto-sync.lock"
    $lock = [System.IO.File]::Open($lockPath, [System.IO.FileMode]::OpenOrCreate, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
    try {
        Invoke-Git @("fetch", "origin", $Branch) | Out-Null
        Invoke-Git @("pull", "--rebase", "--autostash", "origin", $Branch) | Out-Null

        $status = Invoke-Git @("status", "--porcelain")
        if ($status) {
            Invoke-Git @("add", "-A") | Out-Null
            $commitStatus = Invoke-Git @("status", "--porcelain")
            if ($commitStatus) {
                $message = "Auto-sync desktop changes $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
                Invoke-Git @("commit", "-m", $message) | Out-Null
            }
        }

        Invoke-Git @("push", "origin", $Branch) | Out-Null
        Write-SyncLog "sync ok"
    } finally {
        $lock.Dispose()
        Remove-Item -LiteralPath $lockPath -Force -ErrorAction SilentlyContinue
    }
}

do {
    try {
        Sync-Once
    } catch {
        Write-SyncLog "sync failed: $($_.Exception.Message)"
    }
    if ($Once) {
        break
    }
    Start-Sleep -Seconds ([Math]::Max(10, $IntervalSeconds))
} while ($true)
