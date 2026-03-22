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
    QCheckBox,
    QHeaderView,
    QFrame,
    QMenuBar,
    QAction
)
from PyQt5.QtGui import QFont, QKeySequence

from PyQt5.QtCore import QTimer, Qt

from PyQt5.QtGui import QColor

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from pointer import pointer
import numpy as np
# Import rocket class from rocket.py (must be in same folder)
try:
    from rocket import rocket
except Exception as e:
    raise ImportError("Could not import `rocket` from rocket.py. Ensure rocket.py is in the same folder.") from e

POLL_MS = 1
DEFAULT_WINDOW_TITLE = "Unlocked Rkt Telemetry UI"
IS_MACOS = True


class RocketUI(QWidget):
    def __init__(self):
        super().__init__()
        self.menubar = QMenuBar(self)

        self.setWindowTitle(DEFAULT_WINDOW_TITLE)
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
        self.ground_label = QLabel("Ground Station Comm. Port:")
        self.top_row.addWidget(self.ground_label)

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


        self.top_row_antenna = QHBoxLayout()
        self.antenna_label = QLabel("Antenna Pointer Port:")
        self.top_row_antenna.addWidget(self.antenna_label)

        self.port_combo_antenna = QComboBox()
        self.top_row_antenna.addWidget(self.port_combo_antenna)

        self.refresh_ports_btn_antenna = QPushButton("Refresh")
        self.refresh_ports_btn_antenna.clicked.connect(self.refresh_ports)
        self.top_row_antenna.addWidget(self.refresh_ports_btn_antenna)

        self.connect_btn_antenna = QPushButton("Connect")
        self.connect_btn_antenna.clicked.connect(self.connect_serial_antenna)
        self.top_row_antenna.addWidget(self.connect_btn_antenna)

        self.disconnect_btn_antenna = QPushButton("Disconnect")
        self.disconnect_btn_antenna.clicked.connect(self.disconnect_serial_antenna)
        self.top_row_antenna.addWidget(self.disconnect_btn_antenna)

        # --- Main horizontal layout ---
        main_layout = QHBoxLayout()
        main_layout.setMenuBar(self.menubar)

        #refresh_ports
        actionsmenu = self.menubar.addMenu("Actions")
        self.refreshaction = QAction("Refresh Ports", self)
        self.refreshaction.triggered.connect(self.refresh_ports)
        self.refreshaction.setShortcut(QKeySequence("Ctrl+R"))
        actionsmenu.addAction(self.refreshaction)

        self.toggleConnectGND = QAction("Connect Ground Station", self)
        self.toggleConnectGND.triggered.connect(self.connect_btn.animateClick)
        self.toggleConnectGND.setShortcut(QKeySequence("Ctrl+Alt+C"))
        actionsmenu.addAction(self.toggleConnectGND)

        self.toggleConnectAntenna = QAction("Connect Antenna Pointer", self)
        self.toggleConnectAntenna.triggered.connect(self.connect_btn_antenna.animateClick)
        self.toggleConnectAntenna.setShortcut(QKeySequence("Shift+Ctrl+Alt+C"))
        actionsmenu.addAction(self.toggleConnectAntenna)

        self.togglePollingAction = QAction("Start Polling", self)
        self.togglePollingAction.triggered.connect(self.toggle_polling)
        self.togglePollingAction.setShortcut(QKeySequence("Ctrl+Return"))
        self.togglePollingAction.setEnabled(False)
        actionsmenu.addAction(self.togglePollingAction)

        self.toggleLoggingAction = QAction("Start Logging", self)
        self.toggleLoggingAction.triggered.connect(self.toggle_logging)
        self.toggleLoggingAction.setShortcut(QKeySequence("Ctrl+L"))
        self.toggleLoggingAction.setEnabled(False)
        actionsmenu.addAction(self.toggleLoggingAction)




        pointermenu = self.menubar.addMenu("Antenna Pointer")

        self.pointerupaction = QAction("Pointer Up", self)
        self.pointerupaction.triggered.connect(lambda: ((print("[PTR] UP") == None) & (self.pointer_cmd("up") == None)))
        self.pointerupaction.setShortcut(QKeySequence("Ctrl+Up"))
        pointermenu.addAction(self.pointerupaction)

        self.pointerdownaction = QAction("Pointer Down", self)
        self.pointerdownaction.triggered.connect(lambda: ((print("[PTR] DOWN") == None) & (self.pointer_cmd("down") == None)))
        self.pointerdownaction.setShortcut(QKeySequence("Ctrl+Down"))
        pointermenu.addAction(self.pointerdownaction)
        self.pointerrightaction = QAction("Pointer Right", self)
        self.pointerrightaction.triggered.connect(lambda: ((print("[PTR] Right") == None) & (self.pointer_cmd("right") == None)))
        self.pointerrightaction.setShortcut(QKeySequence("Ctrl+Right"))
        pointermenu.addAction(self.pointerrightaction)

        self.pointerleftaction = QAction("Pointer Left", self)
        self.pointerleftaction.triggered.connect(lambda: ((print("[PTR] Left") == None) & (self.pointer_cmd("left") == None)))
        self.pointerleftaction.setShortcut(QKeySequence("Ctrl+Left"))
        pointermenu.addAction(self.pointerleftaction)

        self.pointerleftaction = QAction("Pointer Zero", self)
        self.pointerleftaction.triggered.connect(lambda: ((print("[PTR] Zero") == None) & (self.pointer_cmd("zero") == None)))
        self.pointerleftaction.setShortcut(QKeySequence("Ctrl+0"))
        pointermenu.addAction(self.pointerleftaction)

        

        # --- Left: telemetry + safe commands ---
        left_layout = QVBoxLayout()
        left_layout.addLayout(self.top_row)
        left_layout.addLayout(self.top_row_antenna)

        # Table headers

        rowLabels = QHBoxLayout()
        tmp = QLabel("Rocket Dynamics")
        tmp.setStyleSheet("font-weight:bold;")
        rowLabels.addWidget(tmp,alignment=Qt.AlignCenter)
        separator = QFrame()
        separator.setFrameStyle(QFrame.VLine | QFrame.Sunken)
        rowLabels.addWidget(separator)
        tmp = QLabel("GPS Info")
        tmp.setStyleSheet("font-weight:bold;")
        rowLabels.addWidget(tmp,alignment=Qt.AlignCenter)
        separator = QFrame()
        separator.setFrameStyle(QFrame.VLine | QFrame.Sunken)
        rowLabels.addWidget(separator)
        tmp = QLabel("Power Status")
        tmp.setStyleSheet("font-weight:bold;")
        rowLabels.addWidget(tmp,alignment=Qt.AlignCenter)

        left_layout.addLayout(rowLabels)


        # Top telemetry table
        row = QHBoxLayout()
        
        self.telemetry_table = QTableWidget(0, 2)
        self.telemetry_table.setHorizontalHeaderLabels(["Field", "Value"])
        self.telemetry_table.verticalHeader().setVisible(False)
        self.telemetry_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.telemetry_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.telemetry_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        row.addWidget(self.telemetry_table)
        separator = QFrame()
        separator.setFrameStyle(QFrame.VLine | QFrame.Sunken)
        row.addWidget(separator)

        self.gps_table = QTableWidget(0, 2)
        self.gps_table.setHorizontalHeaderLabels(["Field", "Value"])
        self.gps_table.verticalHeader().setVisible(False)
        self.gps_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.gps_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.gps_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        row.addWidget(self.gps_table)
        separator = QFrame()
        separator.setFrameStyle(QFrame.VLine | QFrame.Sunken)
        row.addWidget(separator)

        self.power_table = QTableWidget(0, 4)
        self.power_table.setHorizontalHeaderLabels(["Name","Enabled", "Voltage", "Current"])
        self.power_table.verticalHeader().setVisible(False)
        self.power_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.power_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.power_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        row.addWidget(self.power_table)
        left_layout.addLayout(row)

        row = QHBoxLayout()
        vert = QVBoxLayout()
        row.addLayout(vert)


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
        vert.addWidget(safe_group)

        # --- Servo control (safe) ---
        self.servo_group = QGroupBox("Servo Control (safe)")
        whole_servo_layout = QVBoxLayout()

        labelsLayout = QHBoxLayout()
        tmp = QLabel("Roll Control")
        tmp.setStyleSheet("font-weight:bold;")
        labelsLayout.addWidget(tmp,alignment=Qt.AlignCenter)
        #separator = QFrame()
        #separator.setFrameStyle(QFrame.VLine | QFrame.Sunken)
        #labelsLayout.addWidget(separator)
        tmp = QLabel("Airbrakes Deployment")
        tmp.setStyleSheet("font-weight:bold;")
        labelsLayout.addWidget(tmp,alignment=Qt.AlignCenter)
        whole_servo_layout.addLayout(labelsLayout)

        separator = QFrame()
        separator.setFrameStyle(QFrame.HLine | QFrame.Sunken)
        whole_servo_layout.addWidget(separator)


        servo_layout = QHBoxLayout()
        servoLeftLayout = QHBoxLayout()
        servoRightLayout = QHBoxLayout()

        # Roll Control
        servoLeftLayout.addWidget(QLabel("Angle (deg):"))
        self.rollCtrl_angle_input = QLineEdit("0.0")
        servoLeftLayout.addWidget(self.rollCtrl_angle_input)
        self.set_rollCtrl_btn = QPushButton("Set Servo Angle")
        self.set_rollCtrl_btn.clicked.connect(self.set_roll_control_servo_angle)
        servoLeftLayout.addWidget(self.set_rollCtrl_btn)
        servo_layout.addLayout(servoLeftLayout)
        separator = QFrame()
        separator.setFrameStyle(QFrame.VLine | QFrame.Sunken)
        servo_layout.addWidget(separator)


        # Airbrakes
        servoRightLayout.addWidget(QLabel("Angle (Deg):"))
        self.airbrakes_angle_input = QLineEdit("0.0")
        servoRightLayout.addWidget(self.airbrakes_angle_input)
        self.set_airbrakes_btn = QPushButton("Set Airbrakes Ang.")
        self.set_airbrakes_btn.clicked.connect(self.set_airbrakes_angle)
        servoRightLayout.addWidget(self.set_airbrakes_btn)


        servo_layout.addLayout(servoRightLayout)

        whole_servo_layout.addLayout(servo_layout)
        self.servo_group.setLayout(whole_servo_layout)
        vert.addWidget(self.servo_group)


        # --- Poll / Log controls ---
        pl_row = QHBoxLayout()
        self.poll_btn = QPushButton("Start Polling ({} Enter)".format('Cmd' if IS_MACOS else 'Ctrl'))
        self.poll_btn.setStyleSheet("color: rgba(83, 139, 230,0.5); font-weight:bold;")
        self.poll_btn.clicked.connect(self.toggle_polling)
        pl_row.addWidget(self.poll_btn)

        self.log_btn = QPushButton("Start Logging")
        self.log_btn.setCheckable(True)
        self.log_btn.clicked.connect(self.toggle_logging)
        pl_row.addWidget(self.log_btn)

        vert.addLayout(pl_row)

        # --- Pointer Control ---
        
        
        self.pointer_group = QGroupBox("Pointer Control")
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

        self.pointer_group.setLayout(grid)
        row.addWidget(self.pointer_group)
        left_layout.addLayout(row)

        




        main_layout.addLayout(left_layout, stretch=2)

            # --- Right side panel ---
        right_layout = QVBoxLayout()

        # --------------------
        # Pyro table
        # --------------------
        pyro_group = QGroupBox("Pyros")

        p_layout = QVBoxLayout()

        self.pyro_table = QTableWidget(0, 5)
        self.pyro_table.setHorizontalHeaderLabels(["Select", "#", "Status", "A/F", "Rstnc."])
        self.pyro_table.verticalHeader().setVisible(False)
        self.pyro_table.setEditTriggers(QTableWidget.NoEditTriggers)

        # make select + # columns small
        self.pyro_table.setColumnWidth(0, 35)
        self.pyro_table.setColumnWidth(1, 40)
        self.pyro_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.pyro_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)

        p_layout.addWidget(self.pyro_table)

        btn_layout = QHBoxLayout()

        self.pyro_arm_btn = QPushButton("💪 ARM 💪")
        self.pyro_arm_btn.clicked.connect(self.pyro_arm)
        btn_layout.addWidget(self.pyro_arm_btn)

        self.pyro_fire_btn = QPushButton("💥 FIRE 💥")
        self.pyro_fire_btn.clicked.connect(self.pyro_fire)
        btn_layout.addWidget(self.pyro_fire_btn)

        p_layout.addLayout(btn_layout)

        pyro_group.setLayout(p_layout)

        right_layout.addWidget(pyro_group)


        # --------------------
        # Servo table
        # --------------------
        servo_group = QGroupBox("Servos")

        servo_layout = QVBoxLayout()

        self.servo_table = QTableWidget(0, 2)
        self.servo_table.setHorizontalHeaderLabels(["Servo", "Angle"])
        self.servo_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.servo_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.servo_table.verticalHeader().setVisible(False)
        self.servo_table.setEditTriggers(QTableWidget.NoEditTriggers)

        servo_layout.addWidget(self.servo_table)

        servo_group.setLayout(servo_layout)

        right_layout.addWidget(servo_group)

        # add the whole right panel
        main_layout.addLayout(right_layout, stretch=1)
        # --- Bottom: status label ---
        final_layout = QVBoxLayout()
        final_layout.addLayout(main_layout)
        self.status_label = QLabel("Status: Ready")
        final_layout.addWidget(self.status_label)

        self.setLayout(final_layout)

        

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

    def toggle_voltage_3V(self):
        print("[UI] NEW REQ STATE OF 3V: {}".format("ON" if self.chkbx_3V.isChecked() else "OFF"))
        self.updateMasterToggle()

    def toggle_voltage_5V(self):
        print("[UI] NEW REQ STATE OF 5V: {}".format("ON" if self.chkbx_5V.isChecked() else "OFF"))
        self.updateMasterToggle()

    def toggle_voltage_7p4V(self):
        print("[UI] NEW REQ STATE OF 7.4V: {}".format("ON" if self.chkbx_7p4V.isChecked() else "OFF"))
        self.updateMasterToggle()

    def toggle_voltage_8p4V(self):
        print("[UI] NEW REQ STATE OF 8.4V: {}".format("ON" if self.chkbx_8p4V.isChecked() else "OFF"))
        self.updateMasterToggle()

    def toggle_voltage_28V(self):
        print("[UI] NEW REQ STATE OF 28V: {}".format("ON" if self.chkbx_28V.isChecked() else "OFF"))
        self.updateMasterToggle()

    def updateMasterToggle(self):
        is3Von   = self.chkbx_3V.isChecked()
        is5Von   = self.chkbx_5V.isChecked()
        is7p4Von = self.chkbx_7p4V.isChecked()
        is8p4Von = self.chkbx_8p4V.isChecked()
        is28Von  = self.chkbx_28V.isChecked()
        self.master_power_checkbox.setCheckState(Qt.PartiallyChecked if not(is3Von and is5Von and is7p4Von and is8p4Von and is28Von) else Qt.Checked)
    
    # -------------------------
    # Port management
    # -------------------------
    def refresh_ports(self):
        print("[UI] Refreshing Ports")
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.port_combo.addItem("No ports")
        else:
            for p in ports:
                self.port_combo.addItem(p.device)

        self.port_combo_antenna.clear()
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.port_combo_antenna.addItem("No ports")
        else:
            for p in ports:
                self.port_combo_antenna.addItem(p.device)

    def connect_serial(self):
        self.poll_btn.setStyleSheet("color: #538be6; font-weight:bold;")
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
        if self.polling:
            self.toggle_polling()
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



    def connect_serial_antenna(self):
        port = self.port_combo.currentText()
        if not port or port == "No ports":
            QMessageBox.warning(self, "No Port", "No serial port selected.")
            return
        try:
            ok, err  = self.pointer.connect(port)
        except Exception as e:
            ok, err = False, str(e)
        if ok:
            self.status_label.setText(f"Connected to {port}")
            QMessageBox.information(self, "Connected", f"Connected to {port} (115200 baud)")
        else:
            self.status_label.setText("Connection failed")
            QMessageBox.critical(self, "Connect Failed", f"Could not open {port}:\n{err}")
        self.update_ui_state()

    def disconnect_serial_antenna(self):
        try:
            if hasattr(self.pointer, "disconnect"):
                self.pointer.disconnect()
            else:
                if getattr(self.pointer, "ser", None) is not None:
                    try:
                        self.pointer.ser.close()
                    except Exception:
                        pass
                    self.pointer.ser = None
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
            self.poll_btn.setText("Start Polling ({} Enter)".format('Cmd' if IS_MACOS else 'Ctrl'))
            self.status_label.setText("Polling stopped")
            self.poll_btn.setStyleSheet("color: #538be6; font-weight:bold;")
            if hasattr(self,"togglePollingAction"):
                self.togglePollingAction.setText("Start Polling");
        else:
            self.poll_telemetry()
            self.poll_timer.start()
            self.polling = True
            self.poll_btn.setText("Stop Polling ({} Enter)".format('Cmd' if IS_MACOS else 'Ctrl'))
            self.poll_btn.setStyleSheet("color: #9e3131; font-weight:bold;")
            self.status_label.setText("Polling started")
            if hasattr(self,"togglePollingAction"):
                self.togglePollingAction.setText("Stop Polling");
        self.update_ui_state()

    def toggle_logging(self):
        if not hasattr(self,'is_logging'):
            self.is_logging = False
        self.is_logging = not self.is_logging
        self.log_btn.setChecked(self.log_btn.isChecked())
        if self.is_logging:
            try:
                if hasattr(self.rocket, "log_data_start"):
                    self.rocket.log_data_start()
                    self.log_btn.setText("Stop Logging")
                    self.status_label.setText("Logging started")
                    if hasattr(self,"toggleLoggingAction"):
                        self.toggleLoggingAction.setText("Stop Logging");
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
                    if hasattr(self,"toggleLoggingAction"):
                        self.toggleLoggingAction.setText("Start Logging");
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
            "Baro Filtered Alt (m)": getattr(self.rocket, "barofilteredalt", ""),
            "Baro Max Alt (m)": getattr(self.rocket, "baro_max_alt", ""),
            "Roll (deg)": getattr(self.rocket, "roll_gyro_int", ""),
            "Accel Int. Velo (m/s)": getattr(self.rocket, "accel_integrated_velo", ""),
        }

        GPS_snapshot = {
            "GPS Fix": getattr(self.rocket, "gps_fix", ""),
            "GPS Lat": getattr(self.rocket, "lat", ""),
            "GPS Lon": getattr(self.rocket, "lon", ""),
            "GPS Alt (m)": getattr(self.rocket, "gpsalt", ""),
            "GPS Max Alt (m)": getattr(self.rocket, "gps_max_alt", ""),
            "GPS Horiz Prec. (m)": getattr(self.rocket, "gps_horiz_prec", ""),
            "GPS Vert. Prec. (m)": getattr(self.rocket, "gps_vert_prec", ""),
            "GPS Satellite No.": getattr(self.rocket, "gps_num_sat", "")
        }




        old_power_snapshot = {
            "Cell 1 (V)": getattr(self.rocket, "cell_voltages", "")[0],
            "Cell 2 (V)": getattr(self.rocket, "cell_voltages", "")[1],
            "Cell 3 (V)": getattr(self.rocket, "cell_voltages", "")[2],
            "Total Current (A)": getattr(self.rocket, "total_current", ""),
            "3V Voltage (V): " : getattr(self.rocket, "converter_voltages", "")[0],
            "3V Current (V): " : getattr(self.rocket, "converter_currents", "")[0],

            "3.3V Voltage (V): " : getattr(self.rocket, "converter_voltages", "")[1],
            "3.3V Current (V): " : getattr(self.rocket, "converter_currents", "")[1],

            "5V Voltage (V): " : getattr(self.rocket, "converter_voltages", "")[2],
            "5V Current (V): " : getattr(self.rocket, "converter_currents", "")[2],

            "7.4V Voltage (V): " : getattr(self.rocket, "converter_voltages", "")[3],
            "7.4V Current (V): " : getattr(self.rocket, "converter_currents", "")[3],

            "8.4V Voltage (V): " : getattr(self.rocket, "converter_voltages", "")[4],
            "8.4V Current (V): " : getattr(self.rocket, "converter_currents", "")[4],

            "28V Voltage (V): " : getattr(self.rocket, "converter_voltages", "")[5],
            "28V Current (V): " : getattr(self.rocket, "converter_currents", "")[5],
        }
        power_snapshot = [
            ["Total", "-", getattr(self.rocket, "total_current", "")],
            ["3V",   getattr(self.rocket, "converter_voltages", "")[0], getattr(self.rocket, "converter_currents", "")[0]],
            ["3.3V", getattr(self.rocket, "converter_voltages", "")[1], getattr(self.rocket, "converter_currents", "")[1]],
            ["5V",   getattr(self.rocket, "converter_voltages", "")[2], getattr(self.rocket, "converter_currents", "")[2]],
            ["7.4V", getattr(self.rocket, "converter_voltages", "")[3], getattr(self.rocket, "converter_currents", "")[3]],
            ["8.4V", getattr(self.rocket, "converter_voltages", "")[4], getattr(self.rocket, "converter_currents", "")[4]],
            ["28V", getattr(self.rocket, "converter_voltages", "")[5], getattr(self.rocket, "converter_currents", "")[5]],
        ]

        power_snapshot_testing = [
            ["Total", "-", 0],
            ["3V",   3,   0],
            ["3.3V", 3, 0],
            ["5V",   5,   0],
            ["7.4V", 8, 0],
            ["8.4V", 8.6, 0],
            ["28V",  20,  0],
        ]

        #power_snapshot = power_snapshot_testing

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
            font = QFont()
            font.setBold(True)
            self.telemetry_table.setItem(row, 0, QTableWidgetItem(str(k)))
            self.telemetry_table.setItem(row, 1, QTableWidgetItem(display_val))
            if row == 0:
                self.telemetry_table.item(0,0).setFont(font)
                self.telemetry_table.item(0,1).setFont(font)
                # PLACEHOLDER FOR COLORING THE STATE CELL 

        self.gps_table.setRowCount(len(GPS_snapshot))
        for row, (k, v) in enumerate(GPS_snapshot.items()):
            if isinstance(v, float):
                if k in ["GPS Lat", "GPS Lon"]:
                    display_val = f"{v:.5f}"
                else:
                    display_val = f"{v:.3f}"
            elif isinstance(v, list):
                display_val = str([round(x,3) if isinstance(x,float) else x for x in v])
            else:
                display_val = str(v)
            self.gps_table.setItem(row, 0, QTableWidgetItem(str(k)))
            self.gps_table.setItem(row, 1, QTableWidgetItem(display_val))

        self.power_table.setRowCount(len(power_snapshot))
        for row,dat in enumerate(power_snapshot):
            item = QTableWidgetItem(dat[0])
            item.setTextAlignment(Qt.AlignCenter)
            self.power_table.setItem(row, 0, item)
            if self.power_table.cellWidget(row, 1) is None and row != 0 and row != 2:
                checkbox = QCheckBox()
                func = self.toggle_voltage_3V
                if row == 1:
                    self.chkbx_3V = checkbox
                if row == 3:
                    func = self.toggle_voltage_5V
                    self.chkbx_5V = checkbox
                elif row == 4:
                    func = self.toggle_voltage_7p4V
                    self.chkbx_7p4V = checkbox
                elif row == 5:
                    func = self.toggle_voltage_8p4V
                    self.chkbx_8p4V = checkbox
                elif row == 6:
                    func = self.toggle_voltage_28V
                    self.chkbx_28V = checkbox
                w = QWidget()
                l = QHBoxLayout()
                l.setAlignment(Qt.AlignCenter)
                checkbox.setChecked(True)
                checkbox.stateChanged.connect(func)
                l.addWidget(checkbox)
                w.setLayout(l)
                self.power_table.setCellWidget(row, 1, w)
            elif self.power_table.cellWidget(row, 1) is None and (row == 0 or row == 2):
                checkbox = QCheckBox()
                if row == 0:
                    self.master_power_checkbox = checkbox
                checkbox.setChecked(True)
                checkbox.setEnabled(False)
                w = QWidget()
                l = QHBoxLayout()
                l.setAlignment(Qt.AlignCenter)
                l.addWidget(checkbox)
                w.setLayout(l)
                self.power_table.setCellWidget(row, 1, w)
            correctVoltage = self.getVoltageByRow(row)
            color = Qt.black
            if correctVoltage != -1 and correctVoltage != 100: # two error codes: -1 is row 0 and 100 is unknown row
                try:
                    measuredVoltage = float(dat[1])
                    percErr = np.abs(measuredVoltage-correctVoltage)/correctVoltage
                    if percErr < 0.02:
                        color = Qt.darkGreen
                    elif percErr < 0.05: 
                        color = QColor(Qt.yellow)
                        color.setRgb(208, 219, 2) # orange
                    elif percErr < 0.1:
                        color = QColor(Qt.red)
                        color.setRgb(255, 127, 14)
                    else:
                        color = Qt.red
                except:
                    print("[UI] [Power] Failed conversion of voltage for row {} to number.".format(row))
                    color = Qt.red
            if row == 0:
                color = QColor(Qt.black)
                color.setAlpha(0)

            item = QTableWidgetItem(str(dat[1])) # voltage
            item.setBackground(QColor(color))
            item2 = QTableWidgetItem(str(dat[2])) # current
            item.setTextAlignment(Qt.AlignCenter)
            item2.setTextAlignment(Qt.AlignCenter)
            self.power_table.setItem(row, 2, item)
            self.power_table.setItem(row, 3, item2)

        # --- Pyros table ---
        try:
            pyros = getattr(self.rocket, "pyros", [None]*8)
            armed = getattr(self.rocket, "armed", [0]*8)
            fired = getattr(self.rocket, "fired", [0]*8)
            servos = getattr(self.rocket, "servos", [0]*8)
            resistances = getattr(self.rocket, "pyro_resistances", [None]*8)

            display_rows = list(range(0, 6))
            self.pyro_table.setRowCount(len(display_rows))
            status_map = {0: ("FAIL", Qt.red), 1: ("UNCONNECTED", Qt.gray),
                        2: ("CONNECTED", Qt.darkYellow), 3: ("FIRED", Qt.green)}

            for i, idx in enumerate(display_rows):

                # create checkbox only once
                
                if self.pyro_table.cellWidget(i, 0) is None:
                    if not hasattr(self,"pyro_checkboxes"):
                        self.pyro_checkboxes = []
                    checkbox = QCheckBox()
                    try:
                        self.pyro_checkboxes[i] = checkbox
                    except:
                        self.pyro_checkboxes.append(checkbox)
                    w = QWidget()
                    l = QHBoxLayout()
                    l.setAlignment(Qt.AlignCenter)
                    l.addWidget(checkbox)
                    w.setLayout(l)
                    self.pyro_table.setCellWidget(i, 0, w)

                # pyro number
                self.pyro_table.setItem(i, 1, QTableWidgetItem(str(idx)))

                # status lookup
                s_text, color = status_map.get(pyros[idx], ("Unknown", Qt.black))

                item = QTableWidgetItem(s_text)
                item.setBackground(QColor(color))
                self.pyro_table.setItem(i, 2, item)

                # armed / fired
                self.pyro_table.setItem(i, 3, QTableWidgetItem(f"A:{armed[idx]} F:{fired[idx]}"))

                # resistance
                res_val = resistances[idx]
                if isinstance(res_val, float):
                    res_text = f"{res_val:.2f}"
                else:
                    res_text = str(res_val)

                self.pyro_table.setItem(i, 4, QTableWidgetItem(res_text))


            for row in range(self.pyro_table.rowCount()):
                for col in range(self.pyro_table.columnCount()):
                    item = self.pyro_table.item(row, col)
                    if item is not None:
                        item.setTextAlignment(Qt.AlignCenter)


        except Exception:
            self.pyro_table.setRowCount(0)

        # --- Servo table ---
        self.servo_table.setRowCount(4)

        for i in range(4):
            servo_idx = i
            item = QTableWidgetItem(str(servo_idx))
            item.setTextAlignment(Qt.AlignCenter)
            self.servo_table.setItem(i, 0, item)
            item = QTableWidgetItem(str(servos[servo_idx]))
            item.setTextAlignment(Qt.AlignCenter)
            self.servo_table.setItem(i, 1, item)

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


    def set_roll_control_servo_angle(self):
        print("[UI] [RollCtrl] Sending Roll Control Servo Angle")
        try:
            val = float(self.rollCtrl_angle_input.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Angle must be a number")
            return
        if hasattr(self.rocket, "set_roll_control_servo_angle"):
            self.rocket.set_roll_control_servo_angle(val)
            self.status_label.setText(f"set_roll_control_servo_angle({val}) sent")
            print("[UI] [RollCtrl] Sent angle {} to roll control".format(val))
    

    def set_airbrakes_angle(self):
        print("[UI] [Airbrakes] Sending Airbrakes Servo Angle")
        try:
            val = float(self.airbrakes_angle_input.text())
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Angle must be a number")
            return
        if hasattr(self.rocket, "set_airbrakes_angle"):
            self.rocket.set_airbrakes_angle(val)
            self.status_label.setText(f"set_airbrakes_angle({val}) sent")
            print("[UI] [Airbrakes] Sent angle {} to airbrakes".format(val))

    def pd_activate(self):
        try:
            if hasattr(self.rocket, "pd_activate"):
                self.rocket.pd_activate()
                self.status_label.setText("pd_activate sent")
                print("[UI] [RollCtrl] PD Activation Req Sent")
            else:
                raise RuntimeError("pd_activate not available")
        except Exception as e:
            QMessageBox.critical(self, "Command Error", str(e))

    def get_selected_pyros(self):
        selected = []
        try:
            for row in range(len(self.pyro_checkboxes)):
                checkbox = self.pyro_checkboxes[row]
                if checkbox.isChecked():
                    idx_item = self.pyro_table.item(row, 1)
                    try:
                        idx = int(idx_item.text())
                        selected.append(idx)
                    except:
                        pass
        except:
            print("[UI] [Pyros] Could not obtain selected pyros: uninitialized.")

        return selected
    
    def pyro_arm(self):
        pyros = self.get_selected_pyros()

        if not pyros:
            QMessageBox.warning(self, "No Pyros Selected", "Select at least one pyro")
            return

        confirm = QMessageBox.question(
            self,
            "Confirm ARM",
            f"Send ARM to pyros {pyros}?",
            QMessageBox.Yes | QMessageBox.No
        )

        if confirm == QMessageBox.Yes:
            try:
                if hasattr(self.rocket, "arm_pyros"):
                    self.rocket.arm_pyros(pyros)

                self.status_label.setText(f"ARM sent to {pyros}")

            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def pyro_fire(self):
        pyros = self.get_selected_pyros()

        if not pyros:
            QMessageBox.warning(self, "No Pyros Selected", "Select at least one pyro")
            return

        confirm = QMessageBox.question(
            self,
            "Confirm FIRE",
            f"FIRE pyros {pyros}?",
            QMessageBox.Yes | QMessageBox.No
        )

        if confirm == QMessageBox.Yes:
            try:
                if hasattr(self.rocket, "fire_pyros"):
                    self.rocket.fire_pyros(pyros)

                self.status_label.setText(f"FIRE sent to {pyros}")

            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    # -------------------------
    # UI state helpers
    # -------------------------
    def update_ui_state(self):
        connected = getattr(self.rocket, "ser", None) is not None
        connected_antenna = getattr(self.pointer, "isConnected", False)
        if hasattr(self,"togglePollingAction"):
            self.togglePollingAction.setEnabled(connected)
        if hasattr(self,"toggleLoggingAction"):
            self.toggleLoggingAction.setEnabled(connected)

        self.setWindowTitle(DEFAULT_WINDOW_TITLE + ": GroundStation {}; AntennaPtr {}".format(
            "ONLINE ✅" if connected else "OFFLINE ❌",
            "ONLINE ✅" if connected_antenna else "OFFLINE ❌"))

        if connected:
            self.ground_label.setStyleSheet("font-weight:bold; color:#38761D;")
            if hasattr(self,"toggleConnectGND"):
                self.toggleConnectGND.setText("Disconnect Ground Station")
                self.toggleConnectGND.triggered.connect(self.disconnect_btn.animateClick)
        else:
            self.ground_label.setStyleSheet("font-weight:normal; color:#9e3131;")
            if hasattr(self,"toggleConnectGND"):
                self.toggleConnectGND.setText("Connect Ground Station")
                self.toggleConnectGND.triggered.connect(self.connect_btn.animateClick)
        if connected_antenna:
            self.antenna_label.setStyleSheet("font-weight:bold; color:#38761D;")
            if hasattr(self,"toggleConnectAntenna"):
                self.toggleConnectAntenna.setText("Disconnect Antenna Pointer")
                self.toggleConnectAntenna.triggered.connect(self.disconnect_btn_antenna.animateClick)
        else:
            self.antenna_label.setStyleSheet("font-weight:normal; color:#9e3131;")
            if hasattr(self,"toggleConnectAntenna"):
                self.toggleConnectAntenna.setText("Connect Antenna Pointer")
                self.toggleConnectAntenna.triggered.connect(self.connect_btn_antenna.animateClick)

        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.connect_btn_antenna.setEnabled(not connected_antenna)
        self.disconnect_btn_antenna.setEnabled(connected_antenna)
        self.pointer_group.setEnabled(connected_antenna)
        self.poll_btn.setEnabled(connected)
        if not connected:
            self.poll_btn.setStyleSheet("color: rgba(29, 112, 245,0.5); font-weight:bold;")
        self.log_btn.setEnabled(connected)
        self.zero_roll_btn.setEnabled(connected)
        self.zero_alt_btn.setEnabled(connected)
        self.zero_velo_btn.setEnabled(connected)
        self.zero_servos_btn.setEnabled(connected)
        self.advance_state_btn.setEnabled(connected)
        self.servo_group.setEnabled(connected)
        self.pd_activate_btn.setEnabled(connected)
        self.pyro_arm_btn.setEnabled(connected)
        self.pyro_fire_btn.setEnabled(connected)




    ## Random

    def getVoltageByRow(self,row):
        if row == 0:
            return -1
        if row == 1:
            return 3
        if row == 2:
            return 3.3
        if row == 3:
            return 5
        if row == 4:
            return 7.4
        if row == 5:
            return 8.4
        if row == 6:
            return 28
        else:
            return 100 # unknown.

def main():
    app = QApplication(sys.argv)
    try:
        app.setFont(QFont("Lucida Grande", 12)) # Mac
        IS_MACOS = True
    except Exception as e:
        try:
            app.setFont(QFont("Segoe UI", 12)) # Windows
        except Exception as e:
            print(e)
            print("Set font failed :(")
        print(e)
        print("Set font failed :(")

    win = RocketUI()
    win.resize(900, 700)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()