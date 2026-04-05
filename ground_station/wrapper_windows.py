import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTextEdit, QLabel, QLineEdit, QFileDialog
)
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QFont, QKeySequence



# 🔧 Worker thread to run process without freezing UI
class ProcessThread(QThread):
    output_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, script_path, working_dir):
        super().__init__()
        self.script_path = script_path
        self.working_dir = working_dir
        self.process = None
        self._running = True  # 🔑 control flag

    def run(self):
        try:
            self.process = subprocess.Popen(
                ["python", "-u", self.script_path],
                cwd=self.working_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                universal_newlines=True
            )

            while self._running:
                line = self.process.stdout.readline()
                if not line and self.process.poll() is not None:
                    break
                if line:
                    self.output_signal.emit(line.rstrip())

        except Exception as e:
            self.output_signal.emit(f"[ERROR] {e}")

        self.finished_signal.emit()

    def stop(self):
        self._running = False
        if self.process:
            try:
                self.process.kill()
            except:
                pass


# 🖥️ Main UI
class TelemetryUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UI Controller")
        self.setGeometry(300, 200, 600, 400)

        self.thread = None
        self.is_running = False
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # --- Script path ---
        script_layout = QHBoxLayout()
        self.script_input = QLineEdit("UI.py")
        browse_script = QPushButton("Browse Script")
        browse_script.clicked.connect(self.browse_script)
        script_layout.addWidget(QLabel("Script:"))
        script_layout.addWidget(self.script_input)
        script_layout.addWidget(browse_script)

        # --- Working directory ---
        wd_layout = QHBoxLayout()
        self.wd_input = QLineEdit(".")
        browse_wd = QPushButton("Browse Dir")
        browse_wd.clicked.connect(self.browse_wd)
        wd_layout.addWidget(QLabel("Working Dir:"))
        wd_layout.addWidget(self.wd_input)
        wd_layout.addWidget(browse_wd)

        # --- Buttons ---
        button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start Telemetry UI")
        self.start_button.clicked.connect(self.toggle_process)

        self.save_button = QPushButton("Save Log")
        self.save_button.clicked.connect(self.save_log)

        # 🔴 Status indicator
        self.status_label = QLabel("●")
        self.status_label.setStyleSheet("color: red; font-size: 18px;")

        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.status_label)

        # --- Log ---
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setStyleSheet("background-color: #1e1e1e; color: white;")
        try:
            font = QFont("Operator Mono")  # Windows default monospace
        except:
            try:
                font = QFont("Consolas")
            except:
                try: 
                    font = QFont("Courier New")
                except:
                    font = None
        if font:
            font.setStyleHint(QFont.Monospace)
            font.setPointSize(10)
            self.log.setFont(font)

        layout.addLayout(script_layout)
        layout.addLayout(wd_layout)
        layout.addLayout(button_layout)
        layout.addWidget(QLabel("Log:"))
        layout.addWidget(self.log)

        self.setLayout(layout)

    # 📂 Browse script
    def browse_script(self):
        file, _ = QFileDialog.getOpenFileName(self, "Select Python Script", "", "Python Files (*.py)")
        if file:
            self.script_input.setText(file)

    # 📂 Browse working dir
    def browse_wd(self):
        directory = QFileDialog.getExistingDirectory(self, "Select Working Directory")
        if directory:
            self.wd_input.setText(directory)

    # ▶️ Start process
    def toggle_process(self):
        # 🔁 If already running → STOP
        self.script_input.setEnabled(not self.is_running)
        self.wd_input.setEnabled(not self.is_running)
        if self.is_running:
            if self.thread:
                self.thread.stop()
                self.append_log("[INFO] Stopping process...")
            return

        # ▶️ Otherwise → START
        script = self.script_input.text()
        wd = self.wd_input.text()

        if not script or not wd:
            self.append_log("[ERROR] Missing script or working directory")
            return

        self.thread = ProcessThread(script, wd)
        self.thread.output_signal.connect(self.append_log)
        self.thread.finished_signal.connect(self.process_finished)
        self.thread.finished.connect(self.thread.deleteLater)  # 🔑 prevents crash
        self.thread.start()

        self.is_running = True
        self.start_button.setText("Stop Telemetry UI")
        self.status_label.setStyleSheet("color: green; font-size: 18px;")

        self.append_log("[INFO] Process started")

    # 🛑 Process finished
    def process_finished(self):
        self.is_running = False
        self.start_button.setText("Start Telemetry UI")
        self.status_label.setStyleSheet("color: red; font-size: 18px;")
        self.append_log("[INFO] Process finished")

    # 🧾 Append log
    def append_log(self, text):
        self.log.append(text)

    # 💾 Save log
    def save_log(self):
        file, _ = QFileDialog.getSaveFileName(self, "Save Log", "", "Text Files (*.txt)")
        if file:
            with open(file, "w") as f:
                f.write(self.log.toPlainText())


    def closeEvent(self, event):
        if self.thread and self.is_running:
            self.append_log("[INFO] Closing app, terminating process...")
            self.thread.stop()
            self.thread = None  # let Qt clean it up safely

        event.accept()


# 🚀 Run app
if __name__ == "__main__":
    app = QApplication(sys.argv)
    try:
        app.setFont(QFont("Lucida Grande", 12)) # Mac
    except Exception as e:
        try:
            app.setFont(QFont("Segoe UI", 12)) # Windows
        except Exception as e:
            print(e)
            print("Set font failed :(")
        print(e)
        print("Set font failed :(")
    window = TelemetryUI()
    app.aboutToQuit.connect(window.close)
    window.toggle_process()
    window.show()
    sys.exit(app.exec_())