@echo off
title ConeRobot 13-Fleet Mission Control
echo ======================================================================
echo    ConeRobot 13-Robot Fleet Monitoring Dashboard
echo ======================================================================
echo.
echo Starting Python Fleet Telemetry Server on Port 8000...
echo.

py app.py

if %ERRORLEVEL% NEQ 0 (
    python app.py
)

pause
