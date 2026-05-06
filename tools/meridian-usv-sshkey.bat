@echo off
setlocal enabledelayedexpansion

REM ============================================================
REM  MERIDIAN USV - SSH KEY INSTALL
REM  Adds Jesse's public SSH key so he can log in without a password.
REM  Safe to re-run anytime.
REM ============================================================

REM --- Self-elevate to Administrator ---
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo.
    echo   This needs administrator permission.
    echo   Windows will show a popup - please click "Yes".
    echo.
    timeout /t 2 /nobreak >nul
    powershell -NoProfile -Command "try { Start-Process -FilePath '%~f0' -Verb RunAs -ErrorAction Stop } catch { Write-Host ''; Write-Host '   You clicked No on the Windows popup.' -ForegroundColor Red; Write-Host '   Please run this file again and click Yes.' -ForegroundColor Yellow; Write-Host ''; Read-Host 'Press Enter to close' }"
    exit /b
)

title Meridian USV - SSH Key Install
color 0B
cls
echo.
echo   ==========================================================
echo      MERIDIAN USV - SSH KEY INSTALL
echo      Installing Jesse's SSH key so he can log in remotely.
echo      Takes about 5 seconds. Safe to re-run.
echo   ==========================================================
echo.

REM Jesse's public key (ed25519)
set "PUBKEY=ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIN5vTYtsGUc+4pmIwMqp7lQtgQ8YN6u+rlXKf2nymexP jesse@thornveil-usv"

echo [1/4] Setting up user-level authorized_keys...
if not exist "%USERPROFILE%\.ssh" mkdir "%USERPROFILE%\.ssh"
powershell -NoProfile -Command "$key = $env:PUBKEY; $path = Join-Path $env:USERPROFILE '.ssh\authorized_keys'; if (-not (Test-Path $path)) { New-Item -ItemType File -Path $path -Force | Out-Null }; $existing = Get-Content $path -ErrorAction SilentlyContinue; if ($existing -contains $key) { Write-Host '       key already present (user)' -ForegroundColor Green } else { Add-Content -Path $path -Value $key; Write-Host '       key added (user)' -ForegroundColor Green }"
icacls "%USERPROFILE%\.ssh" /inheritance:r /grant "%USERNAME%:F" /grant "SYSTEM:F" >nul 2>&1
icacls "%USERPROFILE%\.ssh\authorized_keys" /inheritance:r /grant "%USERNAME%:F" /grant "SYSTEM:F" >nul 2>&1
echo        user .ssh permissions locked down.

echo.
echo [2/4] Setting up administrators_authorized_keys (Windows OpenSSH quirk)...
if not exist "%PROGRAMDATA%\ssh" mkdir "%PROGRAMDATA%\ssh"
powershell -NoProfile -Command "$key = $env:PUBKEY; $path = Join-Path $env:PROGRAMDATA 'ssh\administrators_authorized_keys'; if (-not (Test-Path $path)) { New-Item -ItemType File -Path $path -Force | Out-Null }; $existing = Get-Content $path -ErrorAction SilentlyContinue; if ($existing -contains $key) { Write-Host '       key already present (admin)' -ForegroundColor Green } else { Add-Content -Path $path -Value $key; Write-Host '       key added (admin)' -ForegroundColor Green }"
icacls "%PROGRAMDATA%\ssh\administrators_authorized_keys" /inheritance:r /grant "Administrators:F" /grant "SYSTEM:F" >nul 2>&1
echo        administrators_authorized_keys permissions locked down.

echo.
echo [3/4] Ensuring sshd is running...
powershell -NoProfile -Command "$s = Get-Service sshd -ErrorAction SilentlyContinue; if (-not $s) { Write-Host '       sshd NOT INSTALLED - run meridian-usv-setup.bat first' -ForegroundColor Red; exit 1 }; if ($s.Status -ne 'Running') { Start-Service sshd }; Write-Host ('       sshd status: ' + (Get-Service sshd).Status) -ForegroundColor Green"

echo.
echo [4/4] Verifying key files...
powershell -NoProfile -Command "$u = Join-Path $env:USERPROFILE '.ssh\authorized_keys'; $a = Join-Path $env:PROGRAMDATA 'ssh\administrators_authorized_keys'; Write-Host ('       user file: ' + $u + ' (' + (Get-Item $u -ErrorAction SilentlyContinue).Length + ' bytes)') -ForegroundColor Green; Write-Host ('       admin file: ' + $a + ' (' + (Get-Item $a -ErrorAction SilentlyContinue).Length + ' bytes)') -ForegroundColor Green"

echo.
color 0A
echo   ==========================================================
echo      DONE! Jesse can now SSH in without a password.
echo      You can close this window.
echo   ==========================================================
echo.
pause
endlocal
