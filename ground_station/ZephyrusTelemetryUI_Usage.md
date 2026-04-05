# Zephyrus Telemetry UI — Usage Guide

## Before you start

Please make sure Python is installed on your system and that the following Python packages are available:

- `serialtools`
- `numpy`
- `matplotlib`
- `scipy`
- `pyqt5`
- `pandas`

You can install them with:

```bash
python3 -m pip install serialtools numpy matplotlib scipy pyqt5 pandas
```

---

## Mac usage

1. Download and extract the provided folder.
2. Open the extracted folder.
3. Launch **`ZephyrusTelemetryUI.app`**.
4. If macOS warns you about opening an app from an unidentified developer:
   - Right-click the app
   - Choose **Open**
   - Confirm that you want to run it

---

## Windows usage

1. Download and extract the provided folder.
2. Open the extracted folder.
3. Double-click **`wrapper_windows.bat`**.
4. If Windows Defender or SmartScreen shows a warning:
   - Review the file location
   - Choose the option to continue only if you trust the source
5. If it fails, open a command prompt at that folder and run
```bash
python3 wrapper_windows.py
```

---

## Notes

- Keep all files from the downloaded folder together.
- Do not move the app or batch file out of the folder.
- If the UI does not launch, first verify that the required Python packages are installed.
- If multiple Python versions are installed, make sure the required packages were installed for the same Python interpreter used by the app or batch file.
