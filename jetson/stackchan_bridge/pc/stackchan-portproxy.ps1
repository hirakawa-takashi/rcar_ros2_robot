# Windows (Administrator): pass LAN 192.168.11.7:8000/8003 to the StackChan bridge standby in WSL.
# StackChan connects here when the mini PC does not answer (xiaozhi-fallback-pc.patch).
# Keep this file ASCII only: Windows PowerShell 5.1 reads BOM-less files as the ANSI code page.
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host 'NG: not Administrator. Right-click Start > Terminal (Admin), then run this again.' -ForegroundColor Red
    exit 1
}
$ip = '192.168.11.7'
foreach ($port in 8000, 8003) {
    netsh interface portproxy delete v4tov4 "listenaddress=$ip" "listenport=$port" 2>$null | Out-Null
    netsh interface portproxy add v4tov4 "listenaddress=$ip" "listenport=$port" connectaddress=127.0.0.1 "connectport=$port"
}
Get-NetFirewallRule -DisplayName 'AI-CAR StackChan bridge' -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName 'AI-CAR StackChan bridge' -Direction Inbound -Protocol TCP -LocalPort 8000,8003 `
    -RemoteAddress LocalSubnet -Action Allow -Profile Any | Out-Null
netsh interface portproxy show v4tov4
Write-Host 'OK' -ForegroundColor Green
