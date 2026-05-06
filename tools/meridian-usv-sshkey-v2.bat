@echo off
setlocal enabledelayedexpansion

REM ============================================================
REM  MERIDIAN USV - SSH KEY INSTALL (V2 - more bulletproof)
REM  - Installs key in all locations sshd might check
REM  - Forces correct ownership
REM  - Reconfigures sshd_config to avoid admin-keys quirk
REM  - Restarts sshd
REM  - Prints diagnostics
REM ============================================================

REM --- Self-elevate ---
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo.
    echo   This needs administrator permission.
    echo   Windows will show a popup - please click "Yes".
    echo.
    timeout /t 2 /nobreak >nul
    powershell -NoProfile -Command "try { Start-Process -FilePath '%~f0' -Verb RunAs -ErrorAction Stop } catch { Write-Host 'You clicked No.' -ForegroundColor Red; Read-Host 'Press Enter to close' }"
    exit /b
)

title Meridian USV - SSH Key Install v2
color 0B
cls
echo.
echo   ==========================================================
echo      MERIDIAN USV - SSH KEY INSTALL (V2)
echo      Bulletproof install: key + config + restart.
echo   ==========================================================
echo.

set "PUBKEY=ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIN5vTYtsGUc+4pmIwMqp7lQtgQ8YN6u+rlXKf2nymexP jesse@thornveil-usv"

echo [1/6] Writing user-level authorized_keys...
if not exist "%USERPROFILE%\.ssh" mkdir "%USERPROFILE%\.ssh"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$key = $env:PUBKEY; $p = Join-Path $env:USERPROFILE '.ssh\authorized_keys'; if (Test-Path $p) { $lines = @(Get-Content $p) | Where-Object { $_ -and ($_ -notlike '*jesse@thornveil-usv*') }; $lines += $key; Set-Content -Path $p -Value $lines -Encoding ASCII } else { Set-Content -Path $p -Value $key -Encoding ASCII }; Write-Host ('       wrote ' + $p)"
icacls "%USERPROFILE%\.ssh" /inheritance:r /grant "%USERNAME%:F" /grant "SYSTEM:F" >nul 2>&1
icacls "%USERPROFILE%\.ssh\authorized_keys" /inheritance:r /grant "%USERNAME%:F" /grant "SYSTEM:F" >nul 2>&1

echo.
echo [2/6] Writing administrators_authorized_keys...
if not exist "%PROGRAMDATA%\ssh" mkdir "%PROGRAMDATA%\ssh"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$key = $env:PUBKEY; $p = Join-Path $env:PROGRAMDATA 'ssh\administrators_authorized_keys'; if (Test-Path $p) { $lines = @(Get-Content $p) | Where-Object { $_ -and ($_ -notlike '*jesse@thornveil-usv*') }; $lines += $key; Set-Content -Path $p -Value $lines -Encoding ASCII } else { Set-Content -Path $p -Value $key -Encoding ASCII }; Write-Host ('       wrote ' + $p)"
REM FORCE ownership + lock ACL (OpenSSH rejects the file silently if owner wrong)
takeown /F "%PROGRAMDATA%\ssh\administrators_authorized_keys" /A >nul 2>&1
icacls "%PROGRAMDATA%\ssh\administrators_authorized_keys" /setowner "BUILTIN\Administrators" >nul 2>&1
icacls "%PROGRAMDATA%\ssh\administrators_authorized_keys" /inheritance:r /grant "BUILTIN\Administrators:F" /grant "SYSTEM:F" >nul 2>&1
echo        administrators key file ownership: Administrators, perms: Admins+SYSTEM only.

echo.
echo [3/6] Disabling admin-keys special-case in sshd_config...
REM The default Windows sshd_config has a "Match Group administrators" block that
REM forces admin users to use administrators_authorized_keys. We comment it out
REM so sshd falls back to the normal user-level authorized_keys.
powershell -NoProfile -ExecutionPolicy Bypass -Command "$cfg = Join-Path $env:PROGRAMDATA 'ssh\sshd_config'; if (Test-Path $cfg) { $lines = Get-Content $cfg; $out = @(); $inMatch = $false; foreach ($ln in $lines) { if ($ln -match '^\s*Match\s+Group\s+administrators') { $inMatch = $true; $out += '# ' + $ln + '   (disabled by meridian setup)'; continue }; if ($inMatch -and ($ln -match '^\s*\S' -and $ln -notmatch '^\s*Match\s+')) { $out += '# ' + $ln; continue }; if ($inMatch -and $ln -match '^\s*Match\s+') { $inMatch = $false }; $out += $ln }; Set-Content -Path $cfg -Value $out -Encoding ASCII; Write-Host '       sshd_config: admin-keys block commented out.' } else { Write-Host '       sshd_config not found - skipping' -ForegroundColor Yellow }"

echo.
echo [4/6] Restarting sshd...
powershell -NoProfile -Command "Restart-Service sshd -Force -ErrorAction SilentlyContinue; Start-Sleep -Seconds 1; $s = (Get-Service sshd).Status; Write-Host ('       sshd status: ' + $s) -ForegroundColor Green"

echo.
echo [5/6] Diagnostics - checking key landed in both files:
powershell -NoProfile -Command "$u = Join-Path $env:USERPROFILE '.ssh\authorized_keys'; $a = Join-Path $env:PROGRAMDATA 'ssh\administrators_authorized_keys'; if (Test-Path $u) { $uc = (Get-Content $u | Measure-Object -Line).Lines; $uk = (Select-String -Path $u -Pattern 'jesse@thornveil-usv' -SimpleMatch -Quiet); Write-Host ('       USER file: ' + $uc + ' lines, has-jesse-key: ' + $uk) } else { Write-Host ('       USER file MISSING: ' + $u) -ForegroundColor Red }; if (Test-Path $a) { $ac = (Get-Content $a | Measure-Object -Line).Lines; $ak = (Select-String -Path $a -Pattern 'jesse@thornveil-usv' -SimpleMatch -Quiet); Write-Host ('       ADMIN file: ' + $ac + ' lines, has-jesse-key: ' + $ak) } else { Write-Host ('       ADMIN file MISSING: ' + $a) -ForegroundColor Red }"

echo.
echo [6/6] Current user groups (are you admin?):
powershell -NoProfile -Command "whoami /groups /fo csv | Select-String 'Administrators','Users' | ForEach-Object { $_.Line.Split(',')[0] } | ForEach-Object { Write-Host ('       ' + $_) }"

echo.
color 0A
echo   ==========================================================
echo      DONE. Jesse should now be able to SSH in.
echo      If it still fails, screenshot this whole window.
echo   ==========================================================
echo.
pause
endlocal
