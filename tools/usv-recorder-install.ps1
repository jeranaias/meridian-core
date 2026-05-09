# Install the MAVLink recorder as a parallel Scheduled Task on the USV
# laptop. Runs alongside the bridge -- subscribes to ws://localhost:5760
# and writes .tlog + .jsonl + .summary.json into
# C:\Users\vangu\Documents\Vanguard-Backups\flightlogs.
#
# Survives bridge restarts (its own retry loop), survives laptop reboots
# (logon trigger), runs forever (24h time limit + RestartCount=999).
$ErrorActionPreference = "SilentlyContinue"
$taskName = "MeridianMAVLinkRecorder"

# 1. Tear down any prior install
Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*mavlink-recorder*" } | ForEach-Object {
    Write-Host ("killing previous recorder PID " + $_.ProcessId)
    Stop-Process -Id $_.ProcessId -Force
}
Stop-ScheduledTask -TaskName $taskName -EA SilentlyContinue
Unregister-ScheduledTask -TaskName $taskName -Confirm:$false -EA SilentlyContinue
Start-Sleep -Seconds 1

# 2. Make sure the log directory exists
$logDir = "C:\Users\vangu\Documents\Vanguard-Backups\flightlogs"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

# 3. Define and register the task
$py     = "C:\Users\vangu\AppData\Local\Programs\Python\Python313\python.exe"
$rec    = "C:\Users\vangu\mavlink-recorder.py"
$logOut = "C:\Users\vangu\recorder.out.log"

$cmd = "& '$py' '$rec' --ws ws://localhost:5760 --log-dir '$logDir' *>&1 | Tee-Object -FilePath '$logOut'"

$action = New-ScheduledTaskAction -Execute "powershell.exe" `
    -Argument "-NoProfile -ExecutionPolicy Bypass -Command `"$cmd`""
$trigger = New-ScheduledTaskTrigger -Once -At (Get-Date).AddSeconds(3)
$logonTrigger = New-ScheduledTaskTrigger -AtLogOn

$settings = New-ScheduledTaskSettingsSet `
    -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
    -ExecutionTimeLimit (New-TimeSpan -Hours 24) `
    -StartWhenAvailable -Hidden `
    -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1)

$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive

Register-ScheduledTask -TaskName $taskName `
    -Action $action `
    -Trigger @($trigger, $logonTrigger) `
    -Settings $settings -Principal $principal | Out-Null

Write-Host "task registered, waiting 5s for startup..."
Start-Sleep -Seconds 5

Write-Host ""
Write-Host "=== recorder log ==="
Get-Content $logOut -Tail 12 -EA SilentlyContinue
Write-Host ""
Write-Host "=== log directory ==="
Get-ChildItem $logDir -EA SilentlyContinue | Select-Object Name, Length, LastWriteTime | Format-Table -AutoSize
Write-Host ""
Write-Host "=== process ==="
Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*mavlink-recorder*" -and $_.CommandLine -notlike "*Where-Object*" } | Format-Table ProcessId, Name -AutoSize
