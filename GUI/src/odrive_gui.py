import sys
import time
from PyQt5 import QtWidgets, QtCore
import odrive
from odrive.enums import *
import threading


class ODriveGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ODrive Legacy GUI")
        self.odrv = None

        layout = QtWidgets.QVBoxLayout()

        self.status_label = QtWidgets.QLabel("Connecting to ODrive...")
        layout.addWidget(self.status_label)

        self.connect_btn = QtWidgets.QPushButton("Connect to ODrive")
        self.connect_btn.clicked.connect(self.connect)
        layout.addWidget(self.connect_btn)

        self.calibrate_btn = QtWidgets.QPushButton("Full Calibration")
        self.calibrate_btn.clicked.connect(self.run_calibration)
        self.calibrate_btn.setEnabled(False)
        layout.addWidget(self.calibrate_btn)

        self.closed_loop_btn = QtWidgets.QPushButton("Enter Closed Loop")
        self.closed_loop_btn.clicked.connect(self.enter_closed_loop)
        self.closed_loop_btn.setEnabled(False)
        layout.addWidget(self.closed_loop_btn)

        self.idle_btn = QtWidgets.QPushButton("Enter Idle")
        self.idle_btn.clicked.connect(self.enter_idle)
        self.idle_btn.setEnabled(False)
        layout.addWidget(self.idle_btn)

        self.readout = QtWidgets.QTextEdit()
        self.readout.setReadOnly(True)
        layout.addWidget(self.readout)

        self.setLayout(layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_info)

    def connect(self):
        self.status_label.setText("Connecting...")
        self.connect_btn.setEnabled(False)

        def worker():
            try:
                self.odrv = odrive.find_any()
                self.status_label.setText("Connected!")
                self.calibrate_btn.setEnabled(True)
                self.closed_loop_btn.setEnabled(True)
                self.idle_btn.setEnabled(True)
                self.timer.start(500)
            except Exception as e:
                self.status_label.setText(f"Failed: {e}")
                self.connect_btn.setEnabled(True)

        threading.Thread(target=worker, daemon=True).start()

    def run_calibration(self):
        if self.odrv:
            self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def enter_closed_loop(self):
        if self.odrv:
            self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def enter_idle(self):
        if self.odrv:
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE

    def update_info(self):
        if self.odrv:
            try:
                pos = self.odrv.axis0.encoder.pos_estimate
                vel = self.odrv.axis0.encoder.vel_estimate
                iq = self.odrv.axis0.motor.current_control.Iq_measured
                state = self.odrv.axis0.current_state
                text = f"State: {state}\nPosition: {pos:.2f}\nVelocity: {vel:.2f}\nIq_measured: {iq:.2f}"
                self.readout.setText(text)
            except Exception as e:
                self.readout.setText(f"Error: {e}")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = ODriveGUI()
    win.resize(300, 300)
    win.show()
    sys.exit(app.exec_())
