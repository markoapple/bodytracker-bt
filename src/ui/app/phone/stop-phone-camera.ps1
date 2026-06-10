$ErrorActionPreference = "SilentlyContinue"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$ServerScript = Join-Path $Root "phone-camera-server.mjs"
$resolved = (Resolve-Path -LiteralPath $ServerScript).Path
$needle = [System.Management.Automation.WildcardPattern]::Escape($resolved)
Get-CimInstance Win32_Process |
    Where-Object { $_.Name -match "node(\.exe)?$" -and $_.CommandLine -like "*$needle*" } |
    ForEach-Object {
        Stop-Process -Id $_.ProcessId -Force
    }

$StatePath = Join-Path $Root "phone-site.json"
$previous = $null
try {
    $previous = Get-Content -LiteralPath $StatePath -Raw | ConvertFrom-Json
} catch {
}
$webPort = if ($previous.web_port) { $previous.web_port } else { 39443 }
$targetPort = if ($previous.target_port) { $previous.target_port } else { 39555 }
$ApkCandidates = @(
    (Join-Path $Root "app-debug.apk"),
    (Join-Path $Root "..\..\..\android\FBTPhoneCamera\app\build\outputs\apk\debug\app-debug.apk"),
    (Join-Path $Root "..\..\..\..\android\FBTPhoneCamera\app\build\outputs\apk\debug\app-debug.apk")
)
$ApkAvailable = [bool]($ApkCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1)
Set-Content -LiteralPath $StatePath -Encoding utf8 -Value ([ordered]@{
    enabled = $false
    url = ""
    urls = @()
    status = "disabled"
    web_port = $webPort
    target_port = $targetPort
    target = "127.0.0.1:$targetPort"
    apk = $ApkAvailable
    updated = (Get-Date).ToString("o")
} | ConvertTo-Json -Compress)
