@echo off
setlocal enabledelayedexpansion

REM ============================================================
REM  MERIDIAN USV - REMOTE ACCESS SETUP
REM  One-click installer. Safe to re-run anytime.
REM  Enables Jesse's remote access via Tailscale.
REM ============================================================

REM --- Self-elevate to Administrator ---
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo.
    echo   This setup needs administrator permission.
    echo   Windows will show a popup - please click "Yes".
    echo.
    timeout /t 2 /nobreak >nul
    powershell -NoProfile -Command "try { Start-Process -FilePath '%~f0' -Verb RunAs -ErrorAction Stop } catch { Write-Host ''; Write-Host '   You clicked No on the Windows popup.' -ForegroundColor Red; Write-Host '   This setup cannot continue without admin access.' -ForegroundColor Red; Write-Host '   Please run this file again and click Yes.' -ForegroundColor Yellow; Write-Host ''; Read-Host 'Press Enter to close' }"
    exit /b
)

title Meridian USV - Remote Access Setup
color 0B
cls
echo.
echo   ==========================================================
echo      MERIDIAN USV - REMOTE ACCESS SETUP
echo      Takes about 30 seconds. Safe to re-run anytime.
echo   ==========================================================
echo.

set /a STEP_OK=0
set /a STEP_FAIL=0

REM ==== STEP 1: OpenSSH Server capability ====
echo [1/5] Installing Windows OpenSSH Server...
powershell -NoProfile -ExecutionPolicy Bypass -Command "try { $c = Get-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0 -ErrorAction Stop; if ($c.State -ne 'Installed') { Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0 -ErrorAction Stop | Out-Null; Write-Host '       Installed OpenSSH Server.' -ForegroundColor Green } else { Write-Host '       Already installed.' -ForegroundColor Green }; exit 0 } catch { Write-Host ('       FAILED: ' + $_.Exception.Message) -ForegroundColor Red; exit 1 }"
if !errorLevel! equ 0 (set /a STEP_OK+=1) else (set /a STEP_FAIL+=1)

REM ==== STEP 2: sshd service ====
echo.
echo [2/5] Starting SSH service (auto-start on boot)...
powershell -NoProfile -ExecutionPolicy Bypass -Command "try { if (-not (Get-Service sshd -ErrorAction SilentlyContinue)) { Write-Host '       FAILED: sshd service not found (step 1 may have failed).' -ForegroundColor Red; exit 1 }; Set-Service -Name sshd -StartupType Automatic -ErrorAction Stop; Start-Service sshd -ErrorAction SilentlyContinue; Start-Sleep -Seconds 1; $s = (Get-Service sshd).Status; if ($s -eq 'Running') { Write-Host '       sshd is running.' -ForegroundColor Green; exit 0 } else { Write-Host ('       WARNING: sshd status = ' + $s) -ForegroundColor Yellow; exit 1 } } catch { Write-Host ('       FAILED: ' + $_.Exception.Message) -ForegroundColor Red; exit 1 }"
if !errorLevel! equ 0 (set /a STEP_OK+=1) else (set /a STEP_FAIL+=1)

REM ==== STEP 3: Tailscale NIC profile ====
echo.
echo [3/5] Setting Tailscale network to Private profile...
powershell -NoProfile -ExecutionPolicy Bypass -Command "try { $nics = Get-NetConnectionProfile -ErrorAction Stop | Where-Object { $_.InterfaceAlias -like '*Tailscale*' }; if ($nics) { foreach ($n in $nics) { Set-NetConnectionProfile -InterfaceIndex $n.InterfaceIndex -NetworkCategory Private -ErrorAction Stop; Write-Host ('       ' + $n.InterfaceAlias + ' -> Private') -ForegroundColor Green }; exit 0 } else { Write-Host '       Tailscale interface not visible (may already be ok).' -ForegroundColor Yellow; exit 0 } } catch { Write-Host ('       FAILED: ' + $_.Exception.Message) -ForegroundColor Red; exit 1 }"
if !errorLevel! equ 0 (set /a STEP_OK+=1) else (set /a STEP_FAIL+=1)

REM ==== STEP 4: Tailscale SSH ====
echo.
echo [4/5] Enabling Tailscale SSH...
if exist "C:\Program Files\Tailscale\tailscale.exe" (
    "C:\Program Files\Tailscale\tailscale.exe" set --ssh
    if !errorLevel! equ 0 (
        echo        Tailscale SSH enabled.
        set /a STEP_OK+=1
    ) else (
        echo        Tailscale SSH command failed. Normal SSH still works.
        set /a STEP_FAIL+=1
    )
) else (
    echo        Tailscale not found at default path - skipping.
    set /a STEP_FAIL+=1
)

REM ==== STEP 5: Firewall rules ====
echo.
echo [5/5] Opening Windows Firewall for Meridian ports...
powershell -NoProfile -ExecutionPolicy Bypass -Command "try { $ports = @(22, 8080, 5760); foreach ($p in $ports) { $name = 'Meridian-USV-Port-' + $p; Remove-NetFirewallRule -DisplayName $name -ErrorAction SilentlyContinue; New-NetFirewallRule -DisplayName $name -Direction Inbound -Protocol TCP -LocalPort $p -Action Allow -Profile Any -ErrorAction Stop | Out-Null; Write-Host ('       Port ' + $p + ' allowed.') -ForegroundColor Green }; exit 0 } catch { Write-Host ('       FAILED: ' + $_.Exception.Message) -ForegroundColor Red; exit 1 }"
if !errorLevel! equ 0 (set /a STEP_OK+=1) else (set /a STEP_FAIL+=1)

REM ==== VERIFICATION ====
echo.
echo   ------------------ VERIFICATION ------------------
powershell -NoProfile -ExecutionPolicy Bypass -Command "Write-Host '   sshd service:' -ForegroundColor Cyan; $s = Get-Service sshd -ErrorAction SilentlyContinue; if ($s) { Write-Host ('     Status=' + $s.Status + ' StartType=' + $s.StartType) -ForegroundColor Green } else { Write-Host '     NOT INSTALLED' -ForegroundColor Red }; Write-Host '   Firewall rules:' -ForegroundColor Cyan; $rules = Get-NetFirewallRule -DisplayName 'Meridian-USV-*' -ErrorAction SilentlyContinue; if ($rules) { foreach ($r in $rules) { Write-Host ('     ' + $r.DisplayName + ' Enabled=' + $r.Enabled) -ForegroundColor Green } } else { Write-Host '     None found' -ForegroundColor Red }; Write-Host '   Tailscale:' -ForegroundColor Cyan; $ts = 'C:\Program Files\Tailscale\tailscale.exe'; if (Test-Path $ts) { $out = & $ts status 2>&1; if ($LASTEXITCODE -eq 0) { $out | Select-Object -First 2 | ForEach-Object { Write-Host ('     ' + $_) -ForegroundColor Green } } else { Write-Host '     Tailscale not running or not logged in' -ForegroundColor Yellow } } else { Write-Host '     Tailscale not installed' -ForegroundColor Red }"

echo.
if !STEP_FAIL! equ 0 (
    color 0A
    echo   ==========================================================
    echo      ALL GOOD - !STEP_OK! of 5 steps succeeded.
    echo      Jesse can now connect. You can close this window.
    echo   ==========================================================
) else (
    color 0E
    echo   ==========================================================
    echo      PARTIAL SUCCESS - !STEP_OK! of 5 steps worked.
    echo      !STEP_FAIL! step^(s^) had problems.
    echo      Please screenshot this whole window and text Jesse.
    echo   ==========================================================
)

echo.
pause
endlocal
