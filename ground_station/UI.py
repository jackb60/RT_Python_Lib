import sys 
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QMessageBox,
    QTableWidget,
    QTableWidgetItem,
    QLineEdit,
    QGroupBox,
    QCheckBox
)
from PyQt5.QtCore import QTimer, Qt

from PyQt5.QtGui import QColor

from pointer import pointer

# Import rocket class from rocket.py (must be in same folder)
try:
    from rocket import rocket
except Exception as e:
    raise ImportError("Could not import `rocket` from rocket.py. Ensure rocket.py is in the same folder.") from e

POLL_MS = 1

class RocketUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Safe Rocket Telemetry UI")
        self.rocket = rocket()   # Do not open serial here
        self.pointer = pointer()
        self.tracking_enabled = False
        self.poll_timer = QTimer()
        self.poll_timer.setInterval(POLL_MS)
        self.poll_timer.timeout.connect(self.poll_telemetry)
        self.polling = False

        self._build_ui()
        self.refresh_ports()
        self.update_ui_state()

    def _build_ui(self):
        # --- Top: serial port selection ---
        self.top_row = QHBoxLayout()
        self.top_row.addWidget(QLabel("Serial Port:"))

        self.port_combo = QComboBox()
        self.top_row.addWidget(self.port_combo)

        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        self.top_row.addWidget(self.refresh_ports_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_serial)
        self.top_row.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.top_row.addWidget(self.disconnect_btn)

        # --- Main horizontal layout ---
        main_layout = QHBoxLayout()

        # --- Left: telemetry + safe commands ---
        left_layout = QVBoxLayout()
        left_layout.addLayout(self.top_row)

        # Top telemetry table
        self.telemetry_table = QTableWidget(0, 2)
        self.telemetry_table.setHorizontalHeaderLabels(["Field", "Value"])
        self.telemetry_table.verticalHeader().setVisible(False)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        left_layout.addWidget(self.telemetry_table)

        # Safe command buttons
        safe_group = QGroupBox("Safe Commands")
        s_layout = QHBoxLayout()
        self.zero_roll_btn = QPushButton("Zero Roll")
        self.zero_roll_btn.clicked.connect(self.zero_roll)
        s_layout.addWidget(self.zero_roll_btn)
        self.zero_alt_btn = QPushButton("Zero Alt")
        self.zero_alt_btn.clicked.connect(self.zero_alt)
        s_layout.addWidget(self.zero_alt_btn)
        self.zero_velo_btn = QPushButton("Zero Velo")
        self.zero_velo_btn.clicked.connect(self.zero_velo)
        s_layout.addWidget(self.zero_velo_btn)
        self.zero_servos_btn = QPushButton("Zero Servos")
        self.zero_servos_btn.clicked.connect(self.zero_servos)
        s_layout.addWidget(self.zero_servos_btn)
        self.advance_state_btn = QPushButton("Advance State")
        self.advance_state_btn.clicked.connect(self.advance_state)
        s_layout.addWidget(self.advance_state_btn)
        self.pd_activate_btn = QPushButton("PD Activate")
        self.pd_activate_btn.clicked.connect(self.pd_activate)
        s_layout.addWidget(self.pd_activate_btn)
        safe_group.setLayout(s_layout)
        left_layout.addWidget(safe_group)

        main_layout.addLayout(left_layout, stretch=2)

        # --- Right: Pyros + Servos table ---
        pyro_group = QGroupBox("Pyros & Servos Status (READ-ONLY)")
        p_layout = QVBoxLayout()
        self.pyro_table = QTableWidget(0, 3)  # start empty
        self.pyro_table.setHorizontalHeaderLabels(["#", "Status", "A/F"])
        self.pyro_table.verticalHeader().setVisible(False)
        self.pyro_table.setEditTriggers(QTableWidget.NoEditTriggers)
        p_layout.addWidget(self.pyro_table)
        warn = QLabel("<b style='color:red'>WARNING:</b> Remote arming/firing controls are intentionally disabled.")
        warn.setWordWrap(True)
        p_layout.addWidget(warn)
        pyro_group.setLayout(p_layout)

        main_layout.addWidget(pyro_group, stretch=1)

        # --- Bottom: status label ---
        final_layout = QVBoxLayout()
        final_layout.addLayout(main_layout)
        self.status_label = QLabel("Status: Ready")
        final_layout.addWidget(self.status_label)

        self.setLayout(final_layout)

        # --- Servo control (safe) ---
        servo_group = QGroupBox("Servo Control (safe)")
        servo_layout = QHBoxLayout()
        servo_layout.addWidget(QLabel("Angle (deg):"))
        self.servo_angle_input = QLineEdit("0.0")
        servo_layout.addWidget(self.servo_angle_input)
        self.set_servo_btn = QPushButton("Set Servo Angle")
        self.set_servo_btn.clicked.connect(self.set_servo_angle)
        servo_layout.addWidget(self.set_servo_btn)
        servo_group.setLayout(servo_layout)
        left_layout.addWidget(servo_group)

        # --- Pointer Control ---
        pointer_group = QGroupBox("Pointer Control")
        grid = QGridLayout()

        self.btn_up = QPushButton("↑")
        self.btn_down = QPushButton("↓")
        self.btn_left = QPushButton("←")
        self.btn_right = QPushButton("→")
        self.btn_zero = QPushButton("ZERO")

        self.btn_up.clicked.connect(lambda: self.pointer_cmd("up"))
        self.btn_down.clicked.connect(lambda: self.pointer_cmd("down"))
        self.btn_left.clicked.connect(lambda: self.pointer_cmd("left"))
        self.btn_right.clicked.connect(lambda: self.pointer_cmd("right"))
        self.btn_zero.clicked.connect(lambda: self.pointer_cmd("zero"))

        grid.addWidget(self.btn_up,    0, 1)
        grid.addWidget(self.btn_left,  1, 0)
        grid.addWidget(self.btn_zero,  1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_down,  2, 1)

        # Tracking toggle
        self.track_toggle = QCheckBox("Tracking ON")
        self.track_toggle.stateChanged.connect(self.toggle_tracking)
        grid.addWidget(self.track_toggle, 3, 0, 1, 3)

        pointer_group.setLayout(grid)
        left_layout.addWidget(pointer_group)

        # --- Poll / Log controls ---
        pl_row = QHBoxLayout()
        self.poll_btn = QPushButton("Start Polling")
        self.poll_btn.clicked.connect(self.toggle_polling)
        pl_row.addWidget(self.poll_btn)

        self.log_btn = QPushButton("Start Logging")
        self.log_btn.setCheckable(True)
        self.log_btn.clicked.connect(self.toggle_logging)
        pl_row.addWidget(self.log_btn)

        left_layout.addLayout(pl_row)

    def pointer_cmd(self, direction):
        if self.pointer is None:
            QMessageBox.warning(self, "Pointer", "Pointer not connected")
            return

        try:
            getattr(self.pointer, direction)()
            self.status_label.setText(f"Pointer {direction}")
        except Exception as e:
            QMessageBox.critical(self, "Pointer Error", str(e))

    def toggle_tracking(self, state):
        self.tracking_enabled = state == Qt.Checked
        self.status_label.setText(
            "Tracking enabled" if self.tracking_enabled else "Tracking disabled"
        )
    
    # -------------------------
    # Port management
    # -------------------------
    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.port_combo.addItem("No ports")
        else:
            for p in ports:
                self.port_combo.addItem(p.device)

    def connect_serial(self):
        port = self.port_combo.currentText()
        if not port or port == "No ports":
            QMessageBox.warning(self, "No Port", "No serial port selected.")
            return
        try:
            ok, err = self.rocket.connect_serial(port)
        except Exception as e:
            ok, err = False, str(e)
        if ok:
            self.status_label.setText(f"Connected to {port}")
            QMessageBox.information(self, "Connected", f"Connected to {port} (115200 baud)")
        else:
            self.status_label.setText("Connection failed")
            QMessageBox.critical(self, "Connect Failed", f"Could not open {port}:\n{err}")
        self.update_ui_state()

    def disconnect_serial(self):
        try:
            if hasattr(self.rocket, "disconnect_serial"):
                self.rocket.disconnect_serial()
            else:
                if getattr(self.rocket, "ser", None) is not None:
                    try:
                        self.rocket.ser.close()
                    except Exception:
                        pass
                    self.rocket.ser = None
            self.status_label.setText("Disconnected")
        except Exception as e:
            QMessageBox.warning(self, "Disconnect Error", str(e))
        self.update_ui_state()

    # -------------------------
    # Polling / Logging
    # -------------------------
    def toggle_polling(self):
        if self.polling:
            self.poll_timer.stop()
            self.polling = False
            self.poll_btn.setText("Start Polling")
            self.status_label.setText("Polling stopped")
        else:
            self.poll_telemetry()
            self.poll_timer.start()
            self.polling = True
            self.poll_btn.setText("Stop Polling")
            self.status_label.setText("Polling started")
        self.update_ui_state()

    def toggle_logging(self):
        if self.log_btn.isChecked():
            try:
                if hasattr(self.rocket, "log_data_start"):
                    self.rocket.log_data_start()
                    self.log_btn.setText("Stop Logging")
                    self.status_label.setText("Logging started")
                else:
                    raise RuntimeError("rocket.log_data_start not implemented")
            except Exception as e:
                QMessageBox.critical(self, "Logging Error", str(e))
                self.log_btn.setChecked(False)
        else:
            try:
                if hasattr(self.rocket, "log_data_stop"):
                    self.rocket.log_data_stop()
                    self.log_btn.setText("Start Logging")
                    self.status_label.setText("Logging stopped")
                else:
                    raise RuntimeError("rocket.log_data_stop not implemented")
            except Exception as e:
                QMessageBox.warning(self, "Logging Error", str(e))

    # -------------------------
    # Telemetry polling
    # -------------------------
    def poll_telemetry(self):
        if self.tracking_enabled:
            try:
                rocket_lat = getattr(self.rocket, "lat", None)
                rocket_lon = getattr(self.rocket, "lon", None)
                rocket_fix = getattr(self.rocket, "gps_fix", 0)
                rocket_alt = getattr(self.rocket, "barofilteredalt", None)
                azimuth, elevation = self.pointer.calc_angles(rocket_fix, rocket_lat, rocket_lon, rocket_alt)
                if rocket_fix:
                    self.pointer.send_angles(azimuth, elevation)

            except Exception as e:
                self.status_label.setText(f"Pointer tracking error: {e}")
        try:
            ok = False
            if hasattr(self.rocket, "telemetry_downlink_update"):
                ok = self.rocket.telemetry_downlink_update()
            else:
                self.status_label.setText("No telemetry method on rocket")
                return False
        except Exception as e:
            self.status_label.setText(f"Telemetry error: {e}")
            return False

        # --- Top telemetry snapshot ---
        snapshot = {
            "State": getattr(self.rocket, "state", "").name,
            "RSSI": getattr(self.rocket, "rssi", ""),
            "RX RSSI": getattr(self.rocket, "rxrssi", ""),
            "Last Rec (ms)": getattr(self.rocket, "last_rec", ""),
            "Battery (V)": getattr(self.rocket, "batt_voltage", ""),
            "GPS Fix": getattr(self.rocket, "gps_fix", ""),
            "GPS Lat": getattr(self.rocket, "lat", ""),
            "GPS Lon": getattr(self.rocket, "lon", ""),
            "GPS Alt (m)": getattr(self.rocket, "gpsalt", ""),
            "GPS Max Alt (m)": getattr(self.rocket, "gps_max_alt", ""),
            "Baro Filtered Alt (m)": getattr(self.rocket, "barofilteredalt", ""),
            "Baro Max Alt (m)": getattr(self.rocket, "baro_max_alt", ""),
            "Roll (deg)": getattr(self.rocket, "roll_gyro_int", ""),
            "Accel Integrated Velo (m/s)": getattr(self.rocket, "accel_integrated_velo", ""),
            "Temp (°C)": getattr(self.rocket, "temp", "")
        }



        self.telemetry_table.setRowCount(len(snapshot))
        for row, (k, v) in enumerate(snapshot.items()):
            if isinstance(v, float):
                if k in ["GPS Lat", "GPS Lon"]:
                    display_val = f"{v:.5f}"
                else:
                    display_val = f"{v:.3f}"
            elif isinstance(v, list):
                display_val = str([round(x,3) if isinstance(x,float) else x for x in v])
            else:
                display_val = str(v)
            self.telemetry_table.setItem(row, 0, QTableWidgetItem(str(k)))
            self.telemetry_table.setItem(row, 1, QTableWidgetItem(display_val))

        # --- Pyros + Servos table ---
        try:
            pyros = getattr(self.rocket, "pyros", [None]*8)
            armed = getattr(self.rocket, "armed", [0]*8)
            fired = getattr(self.rocket, "fired", [0]*8)
            servos = getattr(self.rocket, "servos", [0]*8)

            display_rows = list(range(1, 7)) + ["Servo 6", "Servo 7"]  # pyros 1-6 + servos 6&7
            self.pyro_table.setRowCount(len(display_rows))
            status_map = {0: ("FAIL", Qt.red), 1: ("UNCONNECTED", Qt.gray),
                        2: ("CONNECTED", Qt.darkYellow), 3: ("FIRED", Qt.green)}

            for i, idx in enumerate(display_rows):
                if isinstance(idx, int):  # Pyros
                    s_text, color = status_map.get(pyros[idx], ("Unknown", Qt.black))
                    self.pyro_table.setItem(i, 0, QTableWidgetItem(str(idx)))

                    item = QTableWidgetItem(s_text)
                    item.setBackground(QColor(color))  # color the background
                    self.pyro_table.setItem(i, 1, item)
                    self.pyro_table.setItem(i, 2, QTableWidgetItem(f"A:{armed[idx]} F:{fired[idx]}"))
                else:  # Servos
                    servo_idx = 6 if idx == "Servo 6" else 7
                    self.pyro_table.setItem(i, 0, QTableWidgetItem(idx))
                    self.pyro_table.setItem(i, 1, QTableWidgetItem(str(servos[servo_idx])))
                    self.pyro_table.setItem(i, 2, QTableWidgetItem(""))

        except Exception:
            self.pyro_table.setRowCount(0)

        self.status_label.setText("Telemetry updated")
        return True
    # -------------------------
    # Safe command wrappers
    # -------------------------
    def zero_roll(self):
        if hasattr(self.rocket, "zero_roll"): self.rocket.zero_roll()
        self.status_label.setText("zero_roll sent")

    def zero_alt(self):
        if hasattr(self.rocket, "zero_alt"): self.rocket.zero_alt()
        self.status_label.setText("zero_alt sent")

    def zero_velo(self):
        if hasattr(self.rocket, "zero_velo"): self.rocket.zero_velo()
        self.status_label.setText("zero_velo sent")

    def zero_servos(self):
        if hasattr(self.rocket, "zero_servos"): self.rocket.zero_servos()
        self.status_label.setText("zero_servos sent")

    def advance_state(self):
        confirm = QMessageBox.question(
            self, "Confirm Advance State",
            "Advance state will increment the onboard state machine. Are you sure?",
            QMessageBox.Yes | QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            try:
                self.rocket.advance_state()
                self.status_label.setText("Sent advance_state command (safe).")
            except Exception as e:
                QMessageBox.critical(self, "Command Error", f"advance_state failed: {e}")


    def set_servo_angle(self):
        try:
            val = float(self.servo_angle_input.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Angle must be a number")
            return
        if hasattr(self.rocket, "servos_set_angle"):
            self.rocket.servos_set_angle(val)
            self.status_label.setText(f"servos_set_angle({val}) sent")
    
    def pd_activate(self):
        try:
            if hasattr(self.rocket, "pd_activate"):
                self.rocket.pd_activate()
                self.status_label.setText("pd_activate sent")
            else:
                raise RuntimeError("pd_activate not available")
        except Exception as e:
            QMessageBox.critical(self, "Command Error", str(e))

    # -------------------------
    # UI state helpers
    # -------------------------
    def update_ui_state(self):
        connected = getattr(self.rocket, "ser", None) is not None
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.poll_btn.setEnabled(connected)
        self.log_btn.setEnabled(connected)
        self.zero_roll_btn.setEnabled(connected)
        self.zero_alt_btn.setEnabled(connected)
        self.zero_velo_btn.setEnabled(connected)
        self.zero_servos_btn.setEnabled(connected)
        self.advance_state_btn.setEnabled(connected)
        self.set_servo_btn.setEnabled(connected)
        self.pd_activate_btn.setEnabled(connected)

def main():
    app = QApplication(sys.argv)
    win = RocketUI()
    win.resize(900, 700)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()