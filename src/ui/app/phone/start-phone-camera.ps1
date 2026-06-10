param(
    [int]$WebPort = 39443,
    [int]$TargetPort = 39555,
    [switch]$Open
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$ServerScript = Join-Path $Root "phone-camera-server.mjs"
$CertDir = Join-Path $Root "certs"
$PfxPath = Join-Path $CertDir "bodytracker-phone-camera.pfx"
$PfxPassphrase = "bodytracker"
$StatePath = Join-Path $Root "phone-site.json"

function Write-PhoneSiteState([bool]$Enabled, [string]$Status, [string]$Url, [string[]]$Urls, [bool]$ApkAvailable) {
    $ips = @(Get-PhoneCameraIps)
    Set-Content -LiteralPath $StatePath -Encoding utf8 -Value ([ordered]@{
        enabled = $Enabled
        url = $Url
        urls = @($Urls)
        status = $Status
        web_port = $WebPort
        target_host = "127.0.0.1"
        target_port = $TargetPort
        target = "127.0.0.1:$TargetPort"
        pc_ips = @($ips)
        apk = $ApkAvailable
        cert = "self-signed"
        updated = (Get-Date).ToString("o")
    } | ConvertTo-Json -Compress)
}

function Test-PhoneCameraVirtualAdapterName([string]$Name) {
    if (-not $Name) { return $false }
    return $Name -match '(?i)vEthernet|Hyper-V|VMware|VirtualBox|Docker|WSL|Loopback|Npcap|ZeroTier|Tailscale|WireGuard|TAP|Tunnel'
}

function Get-PhoneCameraIps {
    $candidates = @()
    try {
        $candidates += Get-NetIPConfiguration |
            Where-Object { $_.IPv4Address.IPAddress -and $_.NetAdapter.Status -eq "Up" } |
            ForEach-Object {
                $alias = $_.InterfaceAlias
                foreach ($addr in @($_.IPv4Address)) {
                    [pscustomobject]@{
                        IPAddress = $addr.IPAddress
                        InterfaceAlias = $alias
                        HasGateway = [bool]$_.IPv4DefaultGateway
                        Virtual = Test-PhoneCameraVirtualAdapterName $alias
                    }
                }
            }
    } catch {
    }
    try {
        $candidates += Get-NetIPAddress -AddressFamily IPv4 |
            Where-Object { $_.IPAddress -notlike "127.*" -and $_.IPAddress -notlike "169.254.*" -and $_.PrefixOrigin -ne "WellKnown" } |
            ForEach-Object {
                [pscustomobject]@{
                    IPAddress = $_.IPAddress
                    InterfaceAlias = $_.InterfaceAlias
                    HasGateway = $false
                    Virtual = Test-PhoneCameraVirtualAdapterName $_.InterfaceAlias
                }
            }
    } catch {
    }
    @($candidates) |
        Where-Object { $_.IPAddress -and $_.IPAddress -notlike "127.*" -and $_.IPAddress -notlike "169.254.*" } |
        Sort-Object @{Expression = { if ($_.Virtual) { 1 } else { 0 } }}, @{Expression = { if ($_.HasGateway) { 0 } else { 1 } }}, InterfaceAlias, IPAddress |
        Select-Object -ExpandProperty IPAddress -Unique
}

function Add-Asn1Length([System.Collections.Generic.List[byte]]$Bytes, [int]$Length) {
    if ($Length -lt 128) {
        $Bytes.Add([byte]$Length)
        return
    }
    $parts = [System.Collections.Generic.List[byte]]::new()
    $value = $Length
    while ($value -gt 0) {
        $parts.Insert(0, [byte]($value -band 0xff))
        $value = $value -shr 8
    }
    $Bytes.Add([byte](0x80 -bor $parts.Count))
    foreach ($part in $parts) { $Bytes.Add($part) }
}

function Add-Asn1TaggedValue([System.Collections.Generic.List[byte]]$Bytes, [byte]$Tag, [byte[]]$Value) {
    $Bytes.Add($Tag)
    Add-Asn1Length $Bytes $Value.Length
    foreach ($part in $Value) { $Bytes.Add($part) }
}

function New-SubjectAlternativeNameExtension([string[]]$IpAddresses) {
    $entries = [System.Collections.Generic.List[byte]]::new()
    foreach ($dnsName in @("localhost", "bodytracker-phone-camera")) {
        Add-Asn1TaggedValue $entries 0x82 ([System.Text.Encoding]::ASCII.GetBytes($dnsName))
    }
    foreach ($ipText in @("127.0.0.1") + @($IpAddresses | Select-Object -Unique)) {
        $parsed = [System.Net.IPAddress]::None
        if ([System.Net.IPAddress]::TryParse($ipText, [ref]$parsed) -and $parsed.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork) {
            Add-Asn1TaggedValue $entries 0x87 $parsed.GetAddressBytes()
        }
    }
    $raw = [System.Collections.Generic.List[byte]]::new()
    $raw.Add(0x30)
    Add-Asn1Length $raw $entries.Count
    foreach ($part in $entries) { $raw.Add($part) }
    [System.Security.Cryptography.X509Certificates.X509Extension]::new("2.5.29.17", $raw.ToArray(), $false)
}

function Test-PhoneCertificate([string]$Path, [string]$Passphrase, [string]$PrimaryIp) {
    if (-not (Test-Path $Path)) { return $false }
    try {
        $cert = [System.Security.Cryptography.X509Certificates.X509Certificate2]::new($Path, $Passphrase)
        if ($cert.NotAfter -lt (Get-Date).AddDays(30)) { return $false }
        $san = $cert.Extensions | Where-Object { $_.Oid.Value -eq "2.5.29.17" } | Select-Object -First 1
        if (-not $san) { return $false }
        $sanText = $san.Format($false)
        if ($sanText -notlike "*localhost*") { return $false }
        if ($PrimaryIp -and $sanText -notlike "*$PrimaryIp*") { return $false }
        return $true
    } catch {
        return $false
    }
}

function New-PhoneCertificate([string[]]$IpAddresses) {
    $rsa = [System.Security.Cryptography.RSA]::Create(2048)
    try {
        $req = [System.Security.Cryptography.X509Certificates.CertificateRequest]::new(
            "CN=bodytracker-phone-camera",
            $rsa,
            [System.Security.Cryptography.HashAlgorithmName]::SHA256,
            [System.Security.Cryptography.RSASignaturePadding]::Pkcs1)
        $req.CertificateExtensions.Add([System.Security.Cryptography.X509Certificates.X509BasicConstraintsExtension]::new($false, $false, 0, $false))
        $usage = [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::DigitalSignature -bor [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::KeyEncipherment
        $req.CertificateExtensions.Add([System.Security.Cryptography.X509Certificates.X509KeyUsageExtension]::new($usage, $false))
        $eku = [System.Security.Cryptography.OidCollection]::new()
        [void]$eku.Add([System.Security.Cryptography.Oid]::new("1.3.6.1.5.5.7.3.1"))
        $req.CertificateExtensions.Add([System.Security.Cryptography.X509Certificates.X509EnhancedKeyUsageExtension]::new($eku, $false))
        $req.CertificateExtensions.Add((New-SubjectAlternativeNameExtension $IpAddresses))
        $cert = $req.CreateSelfSigned([DateTimeOffset]::Now.AddDays(-1), [DateTimeOffset]::Now.AddYears(3))
        $bytes = $cert.Export([System.Security.Cryptography.X509Certificates.X509ContentType]::Pfx, $PfxPassphrase)
        [System.IO.File]::WriteAllBytes($PfxPath, $bytes)
    } finally {
        $rsa.Dispose()
    }
}

function Stop-ExistingPhoneServer([string]$ScriptPath) {
    if (-not (Test-Path -LiteralPath $ScriptPath)) { return }
    $resolved = (Resolve-Path -LiteralPath $ScriptPath).Path
    $needle = [System.Management.Automation.WildcardPattern]::Escape($resolved)
    Get-CimInstance Win32_Process |
        Where-Object { $_.Name -match "node(\.exe)?$" -and $_.CommandLine -and $_.CommandLine -like "*$needle*" } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
}

function Add-PhoneFirewallRule([string]$Name, [int]$Port) {
    try {
        netsh advfirewall firewall delete rule name="$Name" | Out-Null
        netsh advfirewall firewall add rule name="$Name" dir=in action=allow protocol=TCP localport=$Port profile=any | Out-Null
    } catch {
    }
}

$ipList = @(Get-PhoneCameraIps)
$ip = $ipList | Select-Object -First 1
$url = if ($ip) { "https://$ip`:$WebPort/" } else { "https://127.0.0.1:$WebPort/" }
$urls = if ($ipList.Count -gt 0) { @($ipList | ForEach-Object { "https://$_`:$WebPort/" }) } else { @($url) }

$ApkCandidates = @(
    (Join-Path $Root "app-debug.apk"),
    (Join-Path $Root "..\..\..\android\FBTPhoneCamera\app\build\outputs\apk\debug\app-debug.apk"),
    (Join-Path $Root "..\..\..\..\android\FBTPhoneCamera\app\build\outputs\apk\debug\app-debug.apk")
)
$ApkAvailable = [bool]($ApkCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1)

try {
    $node = (Get-Command node -ErrorAction Stop).Source
} catch {
    Write-PhoneSiteState $false "failed: node.exe not found; install Node.js or put node.exe on PATH" "" @() $ApkAvailable
    throw
}

if (-not (Test-Path -LiteralPath $ServerScript)) {
    Write-PhoneSiteState $false "failed: missing phone-camera-server.mjs" "" @() $ApkAvailable
    throw "Missing $ServerScript"
}

if (-not (Test-Path $CertDir)) {
    New-Item -ItemType Directory -Force -Path $CertDir | Out-Null
}

if (-not (Test-PhoneCertificate $PfxPath $PfxPassphrase $ip)) {
    New-PhoneCertificate $ipList
}

Add-PhoneFirewallRule "BodyTracker phone web bridge $WebPort" $WebPort
Add-PhoneFirewallRule "BodyTracker phone backend $TargetPort" $TargetPort

Stop-ExistingPhoneServer $ServerScript
Write-PhoneSiteState $true "starting" $url $urls $ApkAvailable

$env:BT_PHONE_WEB_PORT = [string]$WebPort
$env:BT_PHONE_TARGET_HOST = "127.0.0.1"
$env:BT_PHONE_TARGET_PORT = [string]$TargetPort
$env:BT_PHONE_PFX = $PfxPath
$env:BT_PHONE_PFX_PASSPHRASE = $PfxPassphrase

$exitCode = 0
if ($Open) {
    Start-Process $url
}
try {
    & $node $ServerScript
    $exitCode = $LASTEXITCODE
} finally {
    try {
        $current = Get-Content -LiteralPath $StatePath -Raw | ConvertFrom-Json
        if (-not ($current.enabled -eq $false -and $current.status -eq "disabled")) {
            $status = if ($exitCode -eq 0) { "stopped" } else { "failed" }
            Write-PhoneSiteState $false $status "" @() $ApkAvailable
        }
    } catch {
    }
}
exit $exitCode