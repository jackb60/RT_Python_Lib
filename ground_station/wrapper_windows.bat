@echo off

REM 🔑 Change working directory to this script's location
cd /d "%~dp0"

REM ▶️ Run the Python script
REM if you need to install:
REM python3 -m pip install serialtools numpy pyqt5 scipy 
python3 wrapper_windows.py