import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGroupBox
)


class DroneWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone GUI")
        self.resize(600, 500)

        main = QVBoxLayout()
        main.addWidget(self.make_telemetry())
        main.addWidget(self.make_controls())
        main.addWidget(self.make_logs())
        self.setLayout(main)

    def make_telemetry(self):
        box = QGroupBox("Telemetry")
        self.battery = QLabel("Battery: --")
        self.pose = QLabel("Pose: --")

        layout = QVBoxLayout()
        layout.addWidget(self.battery)
        layout.addWidget(self.pose)
        box.setLayout(layout)
        return box

    def make_controls(self):
        box = QGroupBox("Controls")
        takeoff = QPushButton("Takeoff")
        land = QPushButton("Land")
        land.clicked.connect(self.on_land)

        layout = QHBoxLayout()
        layout.addWidget(takeoff)
        layout.addWidget(land)
        layout.addStretch()
        box.setLayout(layout)
        return box

    def make_logs(self):
        box = QGroupBox("Logs")
        self.log = QTextEdit()
        self.log.setReadOnly(True)

        layout = QVBoxLayout()
        layout.addWidget(self.log)
        box.setLayout(layout)
        return box

    def on_land(self):
        self.battery.setText("Battery: 87%")
        self.log.append("Land command sent")


app = QApplication(sys.argv)
window = DroneWindow()
window.show()
sys.exit(app.exec_())