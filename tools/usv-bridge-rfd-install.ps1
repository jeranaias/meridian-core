# Re-register the bridge for RFD900 telemetry path:
#   FC -> TELEM1 -> RFD900 air -> radio -> RFD900 ground -> USB (COM10) -> laptop
# RFD900 stock baud = 57600. Bridge auto-detect now matches FTDI VID 0403.
$ErrorActionPreference = "SilentlyContinue"
$taskName = "MeridianMAVLinkBridge"

Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*mavlink-ws-bridge*" } | ForEach-Object {
    Write-Host ("killing previous bridge PID " + $_.ProcessId)
    Stop-Process -Id $_.ProcessId -Force
}
Stop-ScheduledTask -TaskName $taskName -EA SilentlyContinue
Unregister-ScheduledTask -TaskName $taskName -Confirm:$false -EA SilentlyContinue
Start-Sleep -Seconds 2

$held = Get-NetTCPConnection -LocalPort 5760 -State Listen -EA SilentlyContinue
if ($held) { Stop-Process -Id $held.OwningProcess -Force -EA SilentlyContinue; Start-Sleep -Milliseconds 800 }
Remove-Item C:\Users\vangu\bridge.out.log, C:\Users\vangu\bridge.err.log -EA SilentlyContinue

$py     = "C:\Users\vangu\AppData\Local\Programs\Python\Python313\python.exe"
$bridge = "C:\Users\vangu\mavlink-ws-bridge.py"
$logOut = "C:\Users\vangu\bridge.out.log"

# Baud 57600 = RFD900 stock air rate. --serial auto picks COM10 (FTDI 0403).
$cmd = "& '$py' '$bridge' --serial auto --baud 57600 --ws-host 0.0.0.0 --ws-port 5760 *>&1 | Tee-Object -FilePath '$logOut'"

$action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument "-NoProfile -ExecutionPolicy Bypass -Command `"$cmd`""
$trigger = New-ScheduledTaskTrigger -Once -At (Get-Date).AddSeconds(2)
$logonTrigger = New-ScheduledTaskTrigger -AtLogOn

$settings = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
    -ExecutionTimeLimit (New-TimeSpan -Hours 24) -StartWhenAvailable -Hidden `
    -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1)

$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive

Register-ScheduledTask -TaskName $taskName -Action $action `
    -Trigger @($trigger, $logonTrigger) -Settings $settings -Principal $principal | Out-Null

Write-Host "task registered, waiting 5 s..."
Start-Sleep -Seconds 5

Write-Host ""
Write-Host "=== bridge log ==="
Get-Content $logOut -Tail 12 -EA SilentlyContinue
Write-Host ""
Write-Host "=== port 5760 listener ==="
$listener = Get-NetTCPConnection -LocalPort 5760 -State Listen -EA SilentlyContinue
if ($listener) { Write-Host ("  LISTENING on " + $listener.LocalAddress + ":" + $listener.LocalPort + " (PID " + $listener.OwningProcess + ")") }
else            { Write-Host "  NOT LISTENING" }
Write-Host ""
Write-Host "=== bridge process ==="
Get-Process python -EA SilentlyContinue | Format-Table Id, StartTime
