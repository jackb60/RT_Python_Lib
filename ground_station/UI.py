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
import time 
from PyQt5.QtCore import QTimer, Qt

from PyQt5.QtGui import QColor
import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from datetime import datetime


from pointer import pointer
import numpy as np
# Import rocket class from rocket.py (must be in same folder)
try:
    from rocket import rocket
except Exception as e:
    raise ImportError("Could not import `rocket` from rocket.py. Ensure rocket.py is in the same folder.") from e

POLL_MS = 1
ANGLE_CALC_MS = 200
DEFAULT_WINDOW_TITLE = "Unlocked Rkt Telemetry UI"
IS_MACOS = (sys.platform == "darwin")
REASONABLE_TEMP = 30
print("[UI] [startup] Detected System: {}".format("macOS" if IS_MACOS else "Win/Linux"))


class RocketUI(QWidget):
    def __init__(self):
        super().__init__()
        self.menubar = QMenuBar(self)

        self.setWindowTitle(DEFAULT_WINDOW_TITLE)
        self.rocket = rocket()   # Do not open serial here
        self.pointer = pointer()
        self.tracking_enabled = False
        self.poll_timer = QTimer()
        self.time_timer = QTimer()
        self.calc_angle_timer = QTimer()
        self.poll_timer.setInterval(POLL_MS)
        self.time_timer.setInterval(POLL_MS)
        self.calc_angle_timer.setInterval(ANGLE_CALC_MS)
        self.poll_timer.timeout.connect(self.poll_telemetry)
        self.time_timer.timeout.connect(self.update_time)
        self.calc_angle_timer.timeout.connect(self.send_calc_angles_req)
        self.time_timer.start()
        self.calc_angle_timer.start()
        self.polling = False
        self.is_gndgps_frozen = False
        self.frozen_lat = 0
        self.frozen_lon = 0
        self.frozen_alt = 0
        self.has_polled_at_least_once = False

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

        self.power_table = QTableWidget(0, 5)
        self.power_table.setHorizontalHeaderLabels(["Name","Req. Stat.", "Enabled", "Voltage", "Current"])
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
        self.safe_group = QGroupBox("Safe Commands")
            
        hll = QHBoxLayout()
        s_layoutA = QHBoxLayout()
        s_layoutB = QHBoxLayout()
        vll = QVBoxLayout()
        vll.addLayout(s_layoutA)
        vll.addLayout(s_layoutB)

        self.advance_state_btn = QPushButton("Advance State")
        self.advance_state_btn.clicked.connect(self.advance_state)
        s_layoutA.addWidget(self.advance_state_btn)

        self.zero_pitchYawRoll_btn = QPushButton("Zero P/Y/Roll")
        self.zero_pitchYawRoll_btn.clicked.connect(self.zero_pitchYawRoll)
        s_layoutA.addWidget(self.zero_pitchYawRoll_btn)
        self.zero_alt_btn = QPushButton("Zero Alt")
        self.zero_alt_btn.clicked.connect(self.zero_alt)
        s_layoutA.addWidget(self.zero_alt_btn)
        self.zero_velo_btn = QPushButton("Zero Velo")
        self.zero_velo_btn.clicked.connect(self.zero_velo)
        s_layoutB.addWidget(self.zero_velo_btn)
        self.zero_servos_btn = QPushButton("Zero Servos")
        self.zero_servos_btn.clicked.connect(self.zero_servos)
        s_layoutB.addWidget(self.zero_servos_btn)
        self.pd_activate_btn = QPushButton("PD Activate")
        self.pd_activate_btn.clicked.connect(self.pd_activate)
        s_layoutB.addWidget(self.pd_activate_btn)
    
        hll.addLayout(vll)
        
        hl2 = QHBoxLayout()
        l = QLabel("                    VTX Power Setting: ")
        hl2.addWidget(l)
        self.vtxPower_btn = QComboBox()
        self.vtxPower_btn.addItem("1 W")
        self.vtxPower_btn.addItem("3 W")
        self.vtxPower_btn.addItem("5 W")
        self.vtxPower_btn.addItem("8 W")
        self.vtxPower_btn.currentIndexChanged.connect(self.update_vtx_power)
        hl2.addWidget(self.vtxPower_btn)
        hll.addLayout(hl2)

        self.safe_group.setLayout(hll)
        vert.addWidget(self.safe_group)

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
        
        a_vl = QVBoxLayout()


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
        a_vl.addWidget(self.pointer_group)



        ## lat lon fix of antenna pointer, with button to fix

        self.gndgpsbox = QGroupBox("Ground Station GPS")

        vl = QVBoxLayout()

        gridL = QGridLayout()

        latL = QLabel("Lat: ")
        lonL = QLabel("Lon: ")
        altL = QLabel("Alt (m): ")
        fixL = QLabel("Fix: ")

        self.latValL = QLabel(str("-"))
        self.lonValL = QLabel(str("-"))
        self.altValL = QLabel(str("-"))
        self.fixValL = QLabel(str("-"))

        gridL.addWidget(latL,0,0)
        gridL.addWidget(self.latValL,0,1)
        gridL.addWidget(lonL,1,0)
        gridL.addWidget(self.lonValL,1,1)
        gridL.addWidget(altL,2,0)
        gridL.addWidget(self.altValL,2,1)
        gridL.addWidget(fixL,3,0)
        gridL.addWidget(self.fixValL,3,1)
        vl.addLayout(gridL)


        self.hold_gnd_gps_fixed_btn = QPushButton("Send to AntPtr")
        self.hold_gnd_gps_fixed_btn.setCheckable(True)
        self.hold_gnd_gps_fixed_btn.clicked.connect(self.hold_gnd_gps_fixed)
        vl.addWidget(self.hold_gnd_gps_fixed_btn)
        
        self.gndgpsbox.setLayout(vl)




        a_vl.addWidget(self.gndgpsbox)








        row.addLayout(a_vl)
        left_layout.addLayout(row)

        




        main_layout.addLayout(left_layout, stretch=2)

            # --- Right side panel ---
        right_layout = QVBoxLayout()

        self.emerg_group = QGroupBox("EMERGENCY COMMANDS: DANGER ZONE")
        self.emerg_group.setStyleSheet("""
            QGroupBox {
                background-color: rgba(171,0,0,0.1);
            }
            """)
        vl = QVBoxLayout()
        hl1 = QHBoxLayout()
        hl2 = QHBoxLayout()
        
        self.EMERG_piston_btn = QPushButton("PISTON")
        self.EMERG_piston_btn.clicked.connect(self.EMERG_DEPLOY_PISTON)
        hl1.addWidget(self.EMERG_piston_btn)

        self.EMERG_bp_wells_btn = QPushButton("BP WELLS")
        self.EMERG_bp_wells_btn.clicked.connect(self.EMERG_DEPLOY_BP_WELLS)
        hl1.addWidget(self.EMERG_bp_wells_btn)

        self.EMERG_td_btn = QPushButton("TNDR DSNDR")
        self.EMERG_td_btn.clicked.connect(self.EMERG_DEPLOY_TD)
        hl1.addWidget(self.EMERG_td_btn)

        self.EMERG_all_btn = QPushButton("FIRE ALL RECOVERY MEASURES")
        self.EMERG_all_btn.clicked.connect(self.EMERG_DEPLOY_ALL)
        hl2.addWidget(self.EMERG_all_btn)

        vl.addLayout(hl1)
        vl.addLayout(hl2)

        self.emerg_group.setLayout(vl)
        right_layout.addWidget(self.emerg_group)
        #self.emerg_group.setFixedHeight(120)








        # --------------------
        # Pyro table
        # --------------------
        pyro_group = QGroupBox("Pyros")
        #pyro_group.setFixedHeight(700)

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

        self.pyro_arm_btn = QPushButton("ARM")
        self.pyro_arm_btn.clicked.connect(self.pyro_arm)
        btn_layout.addWidget(self.pyro_arm_btn)

        self.pyro_fire_btn = QPushButton("FIRE")
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
        #servo_group.setFixedHeight(400)

        servo_layout.addWidget(self.servo_table)

        servo_group.setLayout(servo_layout)

        right_layout.addWidget(servo_group)




        ## Power Protections Status Table
        self.pwrprotecgrp = QGroupBox("Power Protection Status")
        vl = QVBoxLayout()

        rll = QHBoxLayout()
        rll.setContentsMargins(0, 0, 0, 0)

        label = QLabel("BMS Protections Enabled: ")
        self.bms_protec_checkbox = QCheckBox()
        self.bms_protec_checkbox.setChecked(True)
        self.bms_protec_checkbox.stateChanged.connect(self.toggle_bms_protec)

        rll.addWidget(label)
        rll.addWidget(self.bms_protec_checkbox)
        rll.addStretch()
        vl.addLayout(rll)

        self.power_protec_table = QTableWidget(2, 6)
        self.power_protec_table.setHorizontalHeaderLabels(["SCD", "OCD2", "OCD1", "OCC","COV","CUV","RSVD_0","RSVD_0"])
        self.power_protec_table.setVerticalHeaderLabels(["Enabled","Triggered"])
        self.power_protec_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.power_protec_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.pwrprotecgrp.setFixedHeight(150 if IS_MACOS else 350)
        vl.addWidget(self.power_protec_table)
        self.pwrprotecgrp.setLayout(vl)
        right_layout.addWidget(self.pwrprotecgrp)


















        # add the whole right panel
        main_layout.addLayout(right_layout, stretch=1)







        # --- Bottom: status label ---
        final_layout = QVBoxLayout()
        final_layout.addLayout(main_layout)



        self.status_label  = QLabel("Status: Ready")
        self.status_label2 = QLabel("Polling Status: Not Polling")
        self.status_label3 = QLabel("Current time: ")
        final_layout.addWidget(self.status_label)
        final_layout.addWidget(self.status_label2)
        final_layout.addWidget(self.status_label3)


        self.setLayout(final_layout)



        #refresh_ports
        actionsmenu = self.menubar.addMenu("Actions")
        self.refreshaction = QAction("Refresh Ports", self)
        self.refreshaction.triggered.connect(self.refresh_ports_btn.animateClick)
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
        self.togglePollingAction.triggered.connect(self.poll_btn.animateClick)
        self.togglePollingAction.setShortcut(QKeySequence("Ctrl+Return"))
        self.togglePollingAction.setEnabled(False)
        actionsmenu.addAction(self.togglePollingAction)

        self.toggleLoggingAction = QAction("Start Logging", self)
        self.toggleLoggingAction.triggered.connect(self.log_btn.animateClick)
        self.toggleLoggingAction.setShortcut(QKeySequence("Ctrl+L"))
        self.toggleLoggingAction.setEnabled(False)
        actionsmenu.addAction(self.toggleLoggingAction)




        pointermenu = self.menubar.addMenu("Antenna Pointer")

        self.pointerupaction = QAction("Pointer Up", self)
        self.pointerupaction.triggered.connect(self.btn_up.animateClick)
        self.pointerupaction.setShortcut(QKeySequence("Ctrl+Up"))
        pointermenu.addAction(self.pointerupaction)

        self.pointerdownaction = QAction("Pointer Down", self)
        self.pointerdownaction.triggered.connect(self.btn_down.animateClick)
        self.pointerdownaction.setShortcut(QKeySequence("Ctrl+Down"))
        pointermenu.addAction(self.pointerdownaction)
        self.pointerrightaction = QAction("Pointer Right", self)
        self.pointerrightaction.triggered.connect(self.btn_right.animateClick)
        self.pointerrightaction.setShortcut(QKeySequence("Ctrl+Right"))
        pointermenu.addAction(self.pointerrightaction)

        self.pointerleftaction = QAction("Pointer Left", self)
        self.pointerleftaction.triggered.connect(self.btn_left.animateClick)
        self.pointerleftaction.setShortcut(QKeySequence("Ctrl+Left"))
        pointermenu.addAction(self.pointerleftaction)

        self.pointerzeroaction = QAction("Pointer Zero", self)
        self.pointerzeroaction.triggered.connect(self.btn_zero.animateClick)
        self.pointerzeroaction.setShortcut(QKeySequence("Ctrl+0"))
        pointermenu.addAction(self.pointerzeroaction)


        

        

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
        self.updateMasterToggles()
        self.sendPowerInfoToRocket()

    def toggle_voltage_5V(self):
        print("[UI] NEW REQ STATE OF 5V: {}".format("ON" if self.chkbx_5V.isChecked() else "OFF"))
        self.updateMasterToggles()
        self.sendPowerInfoToRocket()

    def toggle_voltage_7p4V(self):
        print("[UI] NEW REQ STATE OF 7.4V: {}".format("ON" if self.chkbx_7p4V.isChecked() else "OFF"))
        self.updateMasterToggles()
        self.sendPowerInfoToRocket()

    def toggle_voltage_8p4V(self):
        print("[UI] NEW REQ STATE OF 8.4V: {}".format("ON" if self.chkbx_8p4V.isChecked() else "OFF"))
        self.updateMasterToggles()
        self.sendPowerInfoToRocket()

    def toggle_voltage_28V(self):
        print("[UI] NEW REQ STATE OF 28V: {}".format("ON" if self.chkbx_28V.isChecked() else "OFF"))
        self.updateMasterToggles()
        self.sendPowerInfoToRocket()

    def record_toggle_master(self):
        print("[UI] CHANGED STATE OF MASTER: NOW {} at time {}".format("ON" if self.chkbx_master_readonly.isChecked() else "OFF", int(time.time())))

    def record_toggle_3V(self):
        print("[UI] CHANGED STATE OF 3V: NOW {} at time {}".format("ON" if self.chkbx_3V_readonly.isChecked() else "OFF", int(time.time())))

    def record_toggle_3p3V(self):
        print("[UI] CHANGED STATE OF 3.3V: NOW {} at time {}".format("ON" if self.chkbx_3p3V_readonly.isChecked() else "OFF", int(time.time())))

    def record_toggle_5V(self):
        print("[UI] CHANGED STATE OF 5V: NOW {} at time {}".format("ON" if self.chkbx_5V_readonly.isChecked() else "OFF", int(time.time())))

    def record_toggle_7p4V(self):
        print("[UI] CHANGED STATE OF 7.4V: NOW {} at time {}".format("ON" if self.chkbx_7p4V_readonly.isChecked() else "OFF", int(time.time())))

    def record_toggle_8p4V(self):
        print("[UI] CHANGED STATE OF 8.4V: NOW {} at time {}".format("ON" if self.chkbx_8p4V_readonly.isChecked() else "OFF", int(time.time())))

    def record_toggle_28V(self):
        print("[UI] CHANGED STATE OF 28V: NOW {} at time {}".format("ON" if self.chkbx_28V_readonly.isChecked() else "OFF", int(time.time())))

    
    def toggle_bms_protec(self):
        req_status = self.bms_protec_checkbox.isChecked()

        if hasattr(self.rocket,"bmsprotections"):
            self.rocket.bmsprotections(req_status)
            self.status_label.setText("Sent BMS Status: {}".format("ON" if req_status else "OFF"))
            print("[UI] [PWR] Sent new BMS Protections Status: {}".format("ON" if req_status else "OFF"))
        else:
            print("[UI] [PWR] ERROR Failed to send BMS protections: uninitialized in rocket.py")



    def sendPowerInfoToRocket(self):
        req_is3Von   = self.chkbx_3V.isChecked()
        req_is3p3Von = True
        req_is5Von   = self.chkbx_5V.isChecked()
        req_is7p4Von = self.chkbx_7p4V.isChecked()
        req_is8p4Von = self.chkbx_8p4V.isChecked()
        req_is28Von  = self.chkbx_28V.isChecked()
        if hasattr(self.rocket,"update_converters"):
            powerDat = [req_is3Von,req_is3p3Von,req_is5Von,req_is7p4Von,req_is8p4Von,req_is28Von]
            self.rocket.update_converters(powerDat)
            print("[UI] [PWR] Sent new power status {}".format(dict(keys=['3','3.3','5','7.4','8.4','28'],values=powerDat)))

    def updateMasterToggles(self):
        req_is3Von   = self.chkbx_3V.isChecked()
        req_is3p3Von = True
        req_is5Von   = self.chkbx_5V.isChecked()
        req_is7p4Von = self.chkbx_7p4V.isChecked()
        req_is8p4Von = self.chkbx_8p4V.isChecked()
        req_is28Von  = self.chkbx_28V.isChecked()
        self.master_power_checkbox.setCheckState(Qt.PartiallyChecked if not(req_is3Von and 
                                                                            req_is5Von and 
                                                                            req_is7p4Von and 
                                                                            req_is8p4Von and 
                                                                            req_is28Von) 
                                                                        else Qt.Checked)


        readonly_is3Von   = self.chkbx_3V_readonly.isChecked()
        readonly_is3p3Von = self.chkbx_3p3V_readonly.isChecked()
        readonly_is5Von   = self.chkbx_5V_readonly.isChecked()
        readonly_is7p4Von = self.chkbx_7p4V_readonly.isChecked()
        readonly_is8p4Von = self.chkbx_8p4V_readonly.isChecked()
        readonly_is28Von  = self.chkbx_28V_readonly.isChecked()
        self.chkbx_master_readonly.setCheckState(Qt.PartiallyChecked if (readonly_is3Von or 
                                                                            readonly_is3p3Von or 
                                                                            readonly_is5Von or 
                                                                            readonly_is7p4Von or 
                                                                            readonly_is8p4Von or 
                                                                            readonly_is28Von) 
                                                                        else (Qt.Checked if (readonly_is3Von and 
                                                                            readonly_is3p3Von and 
                                                                            readonly_is5Von and 
                                                                            readonly_is7p4Von and 
                                                                            readonly_is8p4Von and 
                                                                            readonly_is28Von) else Qt.Unchecked))


    
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
            self.status_label.setText(f"Connected to Ground Station on {port}")
            QMessageBox.information(self, "Connected to Ground Station", f"Connected to Ground Station on {port} (115200 baud)")
        else:
            self.status_label.setText("Connection to Ground Station failed")
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
            self.status_label.setText("Disconnected from Ground Station")
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
            self.status_label.setText(f"Connected to Antenna Pointer on {port}")
            QMessageBox.information(self, "Connected", f"Connected to Antenna Pointer on {port} (115200 baud)")
        else:
            self.status_label.setText("Connection failed to Antenna Pointer")
            QMessageBox.critical(self, "Connect Failed", f"Could not open comms to Antenna Pointer on {port}:\n{err}")
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
            self.status_label.setText("Disconnected from Antenna Pointer")
        except Exception as e:
            QMessageBox.warning(self, "Disconnect Error from Antenna Pointer", str(e))
        self.update_ui_state()


    # -------------------------
    # Polling / Logging
    # -------------------------
    def toggle_polling(self):
        self.has_polled_at_least_once = True
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
        """ ## HANDLED BY SEPARATE TIMER
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
                self.status_label.setText(f"Pointer tracking error: {e}")"""
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
            "Angle From Vert. (°)": getattr(self.rocket, "angleFromVertical", ""),
            "Temperature (°C)": getattr(self.rocket, "temp", ""),
        }

        GPS_snapshot = {
            "GPS Fix": getattr(self.rocket, "gps_fix", False),
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
            ["Total", "-", np.round(getattr(self.rocket, "total_current", ""),2)],
            ["3V",   np.round(getattr(self.rocket, "converter_voltages", "")[0],2) np.round(getattr(self.rocket, "converter_currents", "")[0],2)],
            ["3.3V", np.round(getattr(self.rocket, "converter_voltages", "")[1],2) np.round(getattr(self.rocket, "converter_currents", "")[1],2)],
            ["5V",   np.round(getattr(self.rocket, "converter_voltages", "")[2],2) np.round(getattr(self.rocket, "converter_currents", "")[2],2)],
            ["7.4V", np.round(getattr(self.rocket, "converter_voltages", "")[3],2) np.round(getattr(self.rocket, "converter_currents", "")[3],2)],
            ["8.4V", np.round(getattr(self.rocket, "converter_voltages", "")[4],2) np.round(getattr(self.rocket, "converter_currents", "")[4],2)],
            ["28V",  np.round(getattr(self.rocket, "converter_voltages", "")[5],2) np.round(getattr(self.rocket, "converter_currents", "")[5],2)],
            ["Pwr Temp (°C)", np.round(getattr(self.rocket, "bms_temp", ""),1), "-"],
        ]

        power_snapshot_testing = [
            ["Total", "-", 0],
            ["3V",   3,   0],
            ["3.3V", 3, 0],
            ["5V",   5,   0],
            ["7.4V", 8, 0],
            ["8.4V", 8.6, 0],
            ["28V",  20,  0],
            ["Pwr Temp (°C)",  25,  "-"],
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

        if GPS_snapshot["GPS Fix"]:
            self.gps_table.item(0,0).setBackground(QBrush(Qt.darkGreen))
            self.gps_table.item(0,1).setBackground(QBrush(Qt.darkGreen))
        else:
            self.gps_table.item(0,0).setBackground(QBrush(Qt.red))
            self.gps_table.item(0,1).setBackground(QBrush(Qt.red))

        self.power_table.setRowCount(len(power_snapshot))
        for row,dat in enumerate(power_snapshot):
            item = QTableWidgetItem(dat[0])
            item.setTextAlignment(Qt.AlignCenter)
            self.power_table.setItem(row, 0, item)
            specialRows = [0,2,7]
            if self.power_table.cellWidget(row, 1) is None and row not in specialRows:
                checkbox = QCheckBox()
                w = QWidget()       
                func = self.toggle_voltage_3V
                if row == 1:
                    self.chkbx_3V = checkbox
                    self.qw_3V = w
                if row == 3:
                    func = self.toggle_voltage_5V
                    self.chkbx_5V = checkbox
                    self.qw_5V = w
                elif row == 4:
                    func = self.toggle_voltage_7p4V
                    self.chkbx_7p4V = checkbox
                    self.qw_7p4V = w
                elif row == 5:
                    func = self.toggle_voltage_8p4V
                    self.chkbx_8p4V = checkbox
                    self.qw_8p4V = w
                elif row == 6:
                    func = self.toggle_voltage_28V
                    self.chkbx_28V = checkbox
                    self.qw_28V = w
                l = QHBoxLayout()
                l.setAlignment(Qt.AlignCenter)
                checkbox.setChecked(True)
                checkbox.stateChanged.connect(func)
                l.addWidget(checkbox)
                w.setLayout(l)
                self.power_table.setCellWidget(row, 1, w)
            elif self.power_table.cellWidget(row, 1) is None and row in specialRows and row != 7:
                checkbox = QCheckBox()
                if row == 0:
                    self.master_power_checkbox = checkbox
                checkbox.setChecked(True)
                checkbox.setEnabled(False)
                w = QWidget()
                if row == 2:
                    self.qw_3p3V = w
                    self.chkbx_3p3V = checkbox
                l = QHBoxLayout()
                l.setAlignment(Qt.AlignCenter)
                l.addWidget(checkbox)
                w.setLayout(l)
                self.power_table.setCellWidget(row, 1, w)

            if self.power_table.cellWidget(row, 2) is None and row != 7:
                checkbox = QCheckBox()
                w = QWidget()
                func = self.record_toggle_3V
                if row == 0:
                    func = self.record_toggle_master
                    self.chkbx_master_readonly = checkbox
                    self.qw_master_readonly = w
                if row == 1:
                    func = self.record_toggle_3V
                    self.chkbx_3V_readonly = checkbox
                    self.qw_3V_readonly = w
                if row == 2:
                    func = self.record_toggle_3p3V
                    self.chkbx_3p3V_readonly = checkbox
                    self.qw_3p3V_readonly = w
                if row == 3:
                    func = self.record_toggle_5V
                    self.chkbx_5V_readonly = checkbox
                    self.qw_5V_readonly = w
                elif row == 4:
                    func = self.record_toggle_7p4V
                    self.chkbx_7p4V_readonly = checkbox
                    self.qw_7p4V_readonly = w
                elif row == 5:
                    func = self.record_toggle_8p4V
                    self.chkbx_8p4V_readonly = checkbox
                    self.qw_8p4V_readonly = w
                elif row == 6:
                    func = self.record_toggle_28V
                    self.chkbx_28V_readonly = checkbox
                    self.qw_28V_readonly = w
                l = QHBoxLayout()
                l.setAlignment(Qt.AlignCenter)
                checkbox.setChecked(True)
                checkbox.setEnabled(False)
                checkbox.stateChanged.connect(func)
                l.addWidget(checkbox)
                w.setLayout(l)
                self.power_table.setCellWidget(row, 2, w)

            if row == 7:
                item = QTableWidgetItem("-")
                item.setTextAlignment(Qt.AlignCenter)
                self.power_table.setItem(7,1,item)
                item = QTableWidgetItem("-")
                item.setTextAlignment(Qt.AlignCenter)
                self.power_table.setItem(7,2,item)

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
            if row == 7:
                try:
                    temp = float(dat[1])
                    #print("[UI] [Power] Measured temperature {}".format(temp))
                    err = temp - REASONABLE_TEMP
                    #print("[UI] [Power] Abnormality in temperature {}".format(err))  
                    if err < 5:
                        color = Qt.darkGreen
                    elif err < 10:
                        color = QColor(Qt.yellow)
                        color.setRgb(208, 219, 2) # orange
                    elif err < 20:
                        color = QColor(Qt.red)
                except:
                    print("[UI] [Power] Failed conversion of temperature to number: {}".format(dat[1]))
                    color = QColor(Qt.red)

            item = QTableWidgetItem(str(dat[1])) # voltage
            item.setBackground(QColor(color))
            item2 = QTableWidgetItem(str(dat[2])) # current
            item.setTextAlignment(Qt.AlignCenter)
            item2.setTextAlignment(Qt.AlignCenter)
            self.power_table.setItem(row, 3, item)
            self.power_table.setItem(row, 4, item2)

        # --- Pyros table ---
        try:
            pyros = getattr(self.rocket, "pyros", [None]*8)
            armed = getattr(self.rocket, "armed", [0]*8)
            fired = getattr(self.rocket, "fired", [0]*8)
            servos = getattr(self.rocket, "servos_deg", [0]*4)
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






        if not self.is_gndgps_frozen:
            self.latValL.setText(str(getattr(self.rocket,"gnd_lat","-")))
            self.lonValL.setText(str(getattr(self.rocket,"gnd_lon","-")))
            self.altValL.setText(str(getattr(self.rocket,"gnd_alt","-")))
            self.fixValL.setText(str(getattr(self.rocket,"gnd_fix","-")))
            if self.fixValL.text() == "False":
                self.fixValL.setStyleSheet("color: rgba(250, 0, 0,1); font-weight: bold;")
            elif self.fixValL.text() == "True":
                self.fixValL.setStyleSheet("color: rgba(0, 152, 199,1); font-weight: bold;")


        # power protection

        protec_enbl = getattr(self.rocket, "bms_protections_enabled", 0x00) #0x08
        protec_trig = getattr(self.rocket, "bms_protection_status", 0x00) #0x04
        enbl_vals = [(protec_enbl >> i) & 1 for i in range(8)]
        trig_vals = [(protec_trig >> i) & 1 for i in range(8)]
        arrs = np.array([enbl_vals,trig_vals])
        # enbl
        for i in range(8):
            val = int(arrs[0,i])
            color = Qt.darkGreen if val == 1 else Qt.red
            item = QTableWidgetItem(str(val))
            item.setTextAlignment(Qt.AlignCenter)
            item.setBackground(QColor(color))
            self.power_protec_table.setItem(0,7-i,item)
        for i in range(8):
            val = int(arrs[1,i])
            color = Qt.darkGreen if val == 0 else Qt.red
            item = QTableWidgetItem(str(val))
            item.setTextAlignment(Qt.AlignCenter)
            item.setBackground(QColor(color))
            self.power_protec_table.setItem(1,7-i,item)



        statuses = getattr(self.rocket,"enabled_status",[0]*6)
        for i,chkbx in enumerate(
                [self.chkbx_3V_readonly,self.chkbx_3p3V_readonly,self.chkbx_5V_readonly,self.chkbx_7p4V_readonly,self.chkbx_8p4V_readonly,self.chkbx_28V_readonly]
                ):
            self.switched_from_polled_telemetry = True
            chkbx.setChecked(statuses[i])


        requesting_checkboxes = [
            self.chkbx_3V,
            self.chkbx_3p3V,
            self.chkbx_5V,
            self.chkbx_7p4V,
            self.chkbx_8p4V,
            self.chkbx_28V
        ]
        readonly_checkboxes = [
            self.chkbx_3V_readonly,
            self.chkbx_3p3V_readonly,
            self.chkbx_5V_readonly,
            self.chkbx_7p4V_readonly,
            self.chkbx_8p4V_readonly,
            self.chkbx_28V_readonly
        ]
        requesting_cells = [
            self.qw_3V,
            self.qw_3p3V,
            self.qw_5V,
            self.qw_7p4V,
            self.qw_8p4V,
            self.qw_28V
        ]
        readonly_cells = [
            self.qw_3V_readonly,
            self.qw_3p3V_readonly,
            self.qw_5V_readonly,
            self.qw_7p4V_readonly,
            self.qw_8p4V_readonly,
            self.qw_28V_readonly
        ]
        for reqChk,readChk,reqCell,readCell in zip(requesting_checkboxes,readonly_checkboxes,requesting_cells,readonly_cells):
            # Blue if both off
            # Green if both on
            # Red if mismatch
            color = "red"
            if reqChk.isChecked() == readChk.isChecked():
                if reqChk.isChecked():
                    color = "lightGreen"
                else:
                    color = "#3C78D8"
            else:
                color = "#AB0000"
            
            [cell.setStyleSheet("background-color: {};".format(color)) for cell in [readCell,reqCell]]


        self.updateMasterToggles()






        now = datetime.now()
        seconds = now.second + now.microsecond / 1e6
        self.status_label2.setText("Telemetry last updated at time {}".format(now.strftime("%Y-%m-%d %H:%M:") + f"{seconds:0.2f}"))
        return True
    

    def update_time(self):
        now = datetime.now()
        seconds = now.second + now.microsecond / 1e6
        self.status_label3.setText("Current date/time is {}".format(now.strftime("%Y-%m-%d %H:%M:") + f"{seconds:0.2f}"))


    # -------------------------
    # EMERGENCY command wrappers
    # -------------------------

    def EMERG_DEPLOY_PISTON(self):
        confirm = QMessageBox.critical(
            self, "Confirm Deploy Piston Recovery Method ?",
            "Confirm Deploy Piston Recovery Method ?\n\nTHIS WILL FIRE THE PISTON.\nDO NOT FIRE IN PROXIMITY OF PEOPLE !! \n\nAre you sure ?",
            QMessageBox.Yes | QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            try:
                if hasattr(self.rocket, "EMERG_DEPLOY_PISTON"): self.rocket.EMERG_DEPLOY_PISTON()
                self.status_label.setText("Sent EMERGENCY DEPLOY PISTON")
            except Exception as e:
                QMessageBox.critical(self, "Command Error", f"emergency_deploy_piston failed: {e}")
        else:
            self.status_label.setText("No command sent.") 

    def EMERG_DEPLOY_BP_WELLS(self):
        confirm = QMessageBox.critical(
            self, "Confirm Deploy BP Wells Recovery Method ?",
            "Confirm Deploy BP Wells Recovery Method ?\n\nTHIS WILL FIRE BLACK POWDER WELLS.\nDO NOT FIRE IN PROXIMITY OF PEOPLE !! \n\nAre you sure ?",
            QMessageBox.Yes | QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            try:
                if hasattr(self.rocket, "EMERG_DEPLOY_BP_WELLS"): self.rocket.EMERG_DEPLOY_BP_WELLS()
                self.status_label.setText("Sent EMERGENCY DEPLOY BP WELLS")
            except Exception as e:
                QMessageBox.critical(self, "Command Error", f"emergency_deploy_bp_wells failed: {e}")
        else:
            self.status_label.setText("No command sent.")

    def EMERG_DEPLOY_TD(self):
        confirm = QMessageBox.critical(
            self, "Confirm Deploy TD Recovery Method ?",
            "Confirm Deploy TD Recovery Methods ?\n\nTHIS WILL FIRE THE TENDER DESCENDER.\nDO NOT FIRE IN PROXIMITY OF PEOPLE !! \n\nAre you sure ?",
            QMessageBox.Yes | QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            try:
                if hasattr(self.rocket, "EMERG_DEPLOY_TD"): self.rocket.EMERG_DEPLOY_TD()
                self.status_label.setText("Sent EMERGENCY DEPLOY ALL")
            except Exception as e:
                QMessageBox.critical(self, "Command Error", f"emergency_deploy_all failed: {e}")
        else:
            self.status_label.setText("No command sent.")

    def EMERG_DEPLOY_ALL(self):
        confirm = QMessageBox.critical(
            self, "Confirm Deploy All Recovery Methods ?",
            "Confirm Deploy All Recovery Methods ?\n\nTHIS WILL FIRE PISTON, BLACK POWDER WELLS, AND THE TENDER DESCENDER.\nDO NOT FIRE IN PROXIMITY OF PEOPLE !! \n\nAre you sure ?",
            QMessageBox.Yes | QMessageBox.No
        )
        if confirm == QMessageBox.Yes:
            try:
                if hasattr(self.rocket, "EMERG_DEPLOY_ALL"): self.rocket.EMERG_DEPLOY_ALL()
                self.status_label.setText("Sent EMERGENCY DEPLOY ALL")
            except Exception as e:
                QMessageBox.critical(self, "Command Error", f"emergency_deploy_all failed: {e}")
        else:
            self.status_label.setText("No command sent.")











    # -------------------------
    # Safe command wrappers
    # -------------------------


    def send_calc_angles_req(self):
        connected = getattr(self.rocket, "ser", None) is not None
        connected_antenna = getattr(self.pointer, "isConnected", False)

        rocketFixExists = getattr(self.rocket, "gps_fix", False)
        rocketLat = 0
        rocketLon = 0
        rocketAlt = 0
        if rocketFixExists:
            rocketLat = getattr(self.rocket, "lat", 0)
            rocketLon = getattr(self.rocket, "lon", 0)
            rocketAlt = getattr(self.rocket, "barofilteredalt", 0)

        if connected & connected_antenna:
            if self.is_gndgps_frozen:
                if rocketFixExists:
                    if self.tracking_enabled:
                        # Good to go
                        if hasattr(self.pointer,"refreshPointerState"):
                            self.pointer.refreshPointerState(rocketLat,rocketLon,rocketAlt)
                        else:
                            print("[UI] [AntennaPtr] FAILED TO RUN UPDATE ROUTINE - FATAL !!")
        else:
            # unconnected, cannot perform action.
            pass



    def hold_gnd_gps_fixed(self):
        if not self.is_gndgps_frozen:
            print("[UI] [AntennaPtr] Req holding the Gnd Station GPS coords fixed.")
            if not getattr(self.rocket,"gnd_fix",False):
                if (getattr(self.rocket,"gnd_lat",0) == 0) & (getattr(self.rocket,"gnd_lon",0) == 0):
                    print("[UI] [AntennaPtr] WARNING Attempting to hold to nonexistent coordinates. Not saving....")
                    self.status_label.setText("WARNING Attempting to hold to nonexistent coordinates. Not sent.")
                    self.hold_gnd_gps_fixed_btn.setChecked(False)
                    return
            else:
                print("[UI] [AntennaPtr] INFO Attempting to hold without a fix. Continuing....")
            try:
                
                tmp = getattr(self.rocket, 'gnd_lat', "-")
                if tmp != "-":
                    self.frozen_lat = tmp
                else:
                    raise Exception()
                
                tmp = getattr(self.rocket, 'gnd_lon', "-")
                if tmp != "-":
                    self.frozen_lon = tmp
                else:
                    raise Exception()

                tmp = getattr(self.rocket, 'gnd_alt', "-")
                if tmp != "-":
                    self.frozen_alt = tmp
                else:
                    raise Exception()

                self.is_gndgps_frozen = True
                print("[UI] [AntennaPtr] Successfully saved new fix; lat {} lon {} alt {}".format(self.frozen_lat,self.frozen_lon,self.frozen_alt))
                self.hold_gnd_gps_fixed_btn.setText("Fixed for AntPtr")
                if hasattr(self.pointer,"updateGPS"):
                    self.pointer.updateGPS(self.frozen_lat,self.frozen_lon,self.frozen_alt)
                    print("[UI] [AntennaPtr] Successfully sent new GPS coords to Antenna Pointer on {}.".format(self.pointer.port))
                    self.status_label.setText("Successfully sent new GPS coords to Antenna Pointer on {}.".format(self.pointer.port))
                else:
                    print("[UI] [AntennaPtr] WARNING Antenna pointer did not receive new GPS coords !!")
                    self.status_label.setText("FAILED TO SEND new GPS coords to Antenna Pointer on {}.".format(self.pointer.port))
            except Exception as e:
                print("[UI] [AntennaPtr] Failed to obtain Gnd Gps Fix !")
                print(e)
        else:
            print("[UI] [AntennaPtr] Unfreezing Gnd Station GPS coords.")
            self.status_label.setText("Unfroze Ground Station GPS coords")
            self.is_gndgps_frozen = False
            self.hold_gnd_gps_fixed_btn.setText("Send to AntPtr")
            self.frozen_lat = 0
            self.frozen_lon = 0
            self.frozen_alt = 0


    def update_vtx_power(self, index):
        print("[UI] [VTX] New Power Req")
        a = index,["1 W", "3 W","5 W", "8 W"][index]   
        if hasattr(self.rocket, "set_vtx_power"): 
            self.rocket.set_vtx_power(index)
            self.status_label.setText("new vtx power sent (index {} value {})".format(*a))
            print("Sent new vtx power {} <=> {}".format(*a))
            print("[UI] [VTX] Success !")
        else:
            self.status_label.setText("*Failed* to send vtx power (index {} value {})".format(*a))



    def zero_pitchYawRoll(self):
        if hasattr(self.rocket, "zero_pitchYawRoll"): self.rocket.zero_pitchYawRoll()
        self.status_label.setText("zero_pitchYawRoll sent")

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
        self.gndgpsbox.setEnabled(connected)
        self.hold_gnd_gps_fixed_btn.setEnabled(connected_antenna & connected & self.has_polled_at_least_once)
        self.poll_btn.setEnabled(connected)
        if not connected:
            self.poll_btn.setStyleSheet("color: rgba(29, 112, 245,0.5); font-weight:bold;")
        self.log_btn.setEnabled(connected)
        self.zero_pitchYawRoll_btn.setEnabled(connected)
        self.zero_alt_btn.setEnabled(connected)
        self.zero_velo_btn.setEnabled(connected)
        self.zero_servos_btn.setEnabled(connected)
        self.advance_state_btn.setEnabled(connected)
        self.servo_group.setEnabled(connected)
        self.pd_activate_btn.setEnabled(connected)
        self.pyro_arm_btn.setEnabled(connected)
        self.pyro_fire_btn.setEnabled(connected)
        self.emerg_group.setEnabled(connected)
        for btn in [self.EMERG_all_btn, self.EMERG_td_btn, self.EMERG_bp_wells_btn, self.EMERG_piston_btn, self.pyro_fire_btn, self.pyro_arm_btn]:
            btn.setStyleSheet(
                "color: rgba(250, 0, 0,0.5); font-weight:bold;" if not connected else
                "color: rgba(250, 0, 0,1); font-weight:bold;"
                )
        self.vtxPower_btn.setEnabled(connected)
        self.safe_group.setEnabled(connected)
        self.pwrprotecgrp.setEnabled(connected)





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
    font = QFont()
    font.setPointSize(10)
    app.setFont(font)
    try:
        app.setFont(QFont("Lucida Grande", 12 if IS_MACOS else 10)) # Mac
    except Exception as e:
        try:
            app.setFont(QFont("Segoe UI", 10)) # Windows
        except Exception as e:
            print(e)
            print("Set font failed :(")
        print(e)
        print("Set font failed :(")

    win = RocketUI()

    win.resize(1600, 1000)
    win.showMaximized()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()