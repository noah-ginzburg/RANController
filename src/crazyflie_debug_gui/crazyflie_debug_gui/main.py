import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLineEdit,
    QProgressBar, QSizePolicy
)


class DroneWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone GUI")

        main = QVBoxLayout()
        top_grid = QGridLayout()
        top_grid.addWidget(self.make_telemetry(), 0, 0)
        top_grid.addWidget(self.make_pose(), 0, 1)
        top_grid.addWidget(self.make_vicon_link(), 1, 0)
        top_grid.addWidget(self.make_motors(), 1, 1)
        # estop sits in its own column, spanning both pane rows (row 0, col 2, 2 rows tall)
        top_grid.addWidget(self.make_estop(), 0, 2, 2, 1)
        top_grid.setColumnStretch(0, 3)
        top_grid.setColumnStretch(1, 3)
        top_grid.setColumnStretch(2, 1)
        top_grid.setRowStretch(0, 1)
        top_grid.setRowStretch(1, 1)

        main.addLayout(top_grid, 3)
        main.addWidget(self.make_controls(), 0)
        main.addWidget(self.make_logs(), 1)
        self.setLayout(main)
        # content needs ~1330px tall before the bars start getting squished
        self.setMinimumSize(770, 1330)
        self.resize(1970, 1430)

    def make_telemetry(self):
        box = QGroupBox("Telemetry")

        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 500)  # hundredths of a volt, 0.00-5.00 V
        self.battery_bar.setTextVisible(True)
        self.set_battery_voltage(0.0)

        self.armed_label = QLabel("Armed: --")
        self.connected_label = QLabel("Connected: --")
        self.tumbled_label = QLabel("Tumbled: --")

        layout = QVBoxLayout()
        layout.addWidget(self.battery_bar)
        layout.addWidget(self.armed_label)
        layout.addWidget(self.connected_label)
        layout.addWidget(self.tumbled_label)
        box.setLayout(layout)
        return box

    def set_battery_voltage(self, voltage):
        low = voltage < 3.8
        self.battery_bar.setValue(int(voltage * 100))
        self.battery_bar.setFormat(f"{voltage:.2f} V / 5.00 V  (Min: 3.80 V)")
        chunk_color = "#cc4444" if low else "#4caf50"
        self.battery_bar.setStyleSheet(
            f"QProgressBar::chunk {{ background-color: {chunk_color}; }}"
        )

    def make_pose(self):
        box = QGroupBox("Pose")
        self.setpoint_label = QLabel("Setpoint: --")
        self.estimated_label = QLabel("Estimated: --")
        self.vicon_label = QLabel("Vicon: --")

        self.position_error_bar = QProgressBar()
        self.position_error_bar.setTextVisible(True)
        self.set_position_error(0.0)
        self.attitude_error_bar = QProgressBar()
        self.attitude_error_bar.setTextVisible(True)
        self.set_attitude_error(0.0)
        self.rpy_error_bar = QProgressBar()
        self.rpy_error_bar.setTextVisible(True)
        self.set_rpy_error(0.0)

        layout = QVBoxLayout()
        layout.addWidget(self.setpoint_label)
        layout.addWidget(self.estimated_label)
        layout.addWidget(self.vicon_label)
        layout.addWidget(self.position_error_bar)
        layout.addWidget(self.attitude_error_bar)
        layout.addWidget(self.rpy_error_bar)
        box.setLayout(layout)
        return box

    def set_position_error(self, error_percent):
        self.position_error_bar.setValue(int(error_percent))
        self.position_error_bar.setFormat(f"Position error: {error_percent:.1f}%")

    def set_attitude_error(self, error_percent):
        self.attitude_error_bar.setValue(int(error_percent))
        self.attitude_error_bar.setFormat(f"Attitude error: {error_percent:.1f}%")

    def set_rpy_error(self, error_percent):
        self.rpy_error_bar.setValue(int(error_percent))
        self.rpy_error_bar.setFormat(f"RPY error: {error_percent:.1f}%")

    def make_vicon_link(self):
        box = QGroupBox("Vicon Link")
        self.dropped_frames_label = QLabel("Dropped frames: --")
        self.occluded_frames_label = QLabel("Occluded frames: --")
        self.staleness_label = QLabel("Staleness: --")

        layout = QVBoxLayout()
        layout.addWidget(self.dropped_frames_label)
        layout.addWidget(self.occluded_frames_label)
        layout.addWidget(self.staleness_label)
        box.setLayout(layout)
        return box

    def make_motors(self):
        box = QGroupBox("Motors & PWMs")
        self.m1_label = QLabel("M1: --")
        self.m2_label = QLabel("M2: --")
        self.m3_label = QLabel("M3: --")
        self.m4_label = QLabel("M4: --")

        motor_row = QHBoxLayout()
        motor_row.addWidget(self.m1_label)
        motor_row.addWidget(self.m2_label)
        motor_row.addWidget(self.m3_label)
        motor_row.addWidget(self.m4_label)

        self.m1_m3_bar = QProgressBar()
        self.m1_m3_bar.setTextVisible(True)
        self.set_m1_m3_split(0.0)
        self.m2_m4_bar = QProgressBar()
        self.m2_m4_bar.setTextVisible(True)
        self.set_m2_m4_split(0.0)
        self.all_motor_bar = QProgressBar()
        self.all_motor_bar.setTextVisible(True)
        self.set_all_motor_split(0.0)

        layout = QVBoxLayout()
        layout.addLayout(motor_row)
        layout.addWidget(self.m1_m3_bar)
        layout.addWidget(self.m2_m4_bar)
        layout.addWidget(self.all_motor_bar)
        box.setLayout(layout)
        return box

    def set_m1_m3_split(self, split_percent):
        self.m1_m3_bar.setValue(int(split_percent))
        self.m1_m3_bar.setFormat(f"M1/M3 split: {split_percent:.1f}%")

    def set_m2_m4_split(self, split_percent):
        self.m2_m4_bar.setValue(int(split_percent))
        self.m2_m4_bar.setFormat(f"M2/M4 split: {split_percent:.1f}%")

    def set_all_motor_split(self, split_percent):
        self.all_motor_bar.setValue(int(split_percent))
        self.all_motor_bar.setFormat(f"All-motor split: {split_percent:.1f}%")


    def make_controls(self):
        box = QGroupBox("Controls")

        takeoff = QPushButton("Takeoff")
        takeoff.setMinimumHeight(80)
        takeoff.setStyleSheet("background-color: #4caf50; color: white; font-weight: bold;")

        land = QPushButton("Land")
        land.setMinimumHeight(80)
        land.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold;")
        land.clicked.connect(self.on_land)

        button_row = QHBoxLayout()
        button_row.addWidget(takeoff)
        button_row.addWidget(land)

        self.goto_x = QLineEdit()
        self.goto_y = QLineEdit()
        self.goto_z = QLineEdit()
        self.goto_theta = QLineEdit()
        goto_btn = QPushButton("Go To")
        goto_btn.setStyleSheet("background-color: #2196f3; color: white; font-weight: bold;")
        goto_btn.setMinimumWidth(400)
        goto_btn.clicked.connect(self.on_goto)

        goto_row = QHBoxLayout()
        goto_row.addWidget(QLabel("x"))
        goto_row.addWidget(self.goto_x)
        goto_row.addWidget(QLabel("y"))
        goto_row.addWidget(self.goto_y)
        goto_row.addWidget(QLabel("z"))
        goto_row.addWidget(self.goto_z)
        goto_row.addWidget(QLabel("θ"))
        goto_row.addWidget(self.goto_theta)
        goto_row.addWidget(goto_btn)

        pos_row = QHBoxLayout()
        current_xyz_button = QPushButton("Input Current XYZ")
        current_fullpose_button = QPushButton("Input Current Full Pose")
        pos_row.addWidget(current_xyz_button)
        pos_row.addWidget(current_fullpose_button)

        layout = QVBoxLayout()
        layout.addLayout(button_row)
        layout.addLayout(goto_row)
        layout.addLayout(pos_row)
        box.setLayout(layout)
        return box

    def make_estop(self):
        # QSizePolicy.Expanding lets it stretch to the full height of the pane grid
        estop_button = QPushButton("EMERGENCY\nSTOP")
        estop_button.setStyleSheet(
            "background-color: #cc4444; color: white; font-weight: bold; font-size: 32px;"
        )
        estop_button.setMinimumWidth(560)
        estop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        return estop_button

    def on_land(self):
        self.set_battery_voltage(3.87)
        self.log.append("Land command sent")

    def on_goto(self):
        x = float(self.goto_x.text())
        self.log.append(f"Goto {x}, ...")

    def make_logs(self):
        box = QGroupBox("GUI Logs")
        box.setMinimumHeight(300)
        self.log = QTextEdit()
        self.log.setReadOnly(True)

        layout = QVBoxLayout()
        layout.addWidget(self.log)
        box.setLayout(layout)
        return box




app = QApplication(sys.argv)
window = DroneWindow()
window.show()
sys.exit(app.exec_())