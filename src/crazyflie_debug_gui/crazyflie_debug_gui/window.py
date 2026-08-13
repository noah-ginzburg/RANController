"""Qt side of the debug GUI. No ROS imports on purpose.

main() in debug_gui.py creates the ROS node (TelemetryModel) and hands it to
this window as `node`. The window only ever calls named methods on it - it
never imports rclpy or touches a message type itself. That also means you can
run this file directly with no node at all to iterate on the layout without
starting ROS:  python3 window.py
"""

import math
import sys
from collections import deque

from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLineEdit,
    QProgressBar, QSizePolicy
)

# --- palette -----------------------------------------------------------
# Diverging pair for signed values: two poles that read as opposites, with a
# neutral (not coloured) midpoint. Sequential blue for unsigned magnitudes.
SURFACE   = "#fcfcfb"
INK       = "#0b0b0b"
INK_MUTED = "#898781"
GRIDLINE  = "#e1e0d9"
BASELINE  = "#c3c2b7"
POLE_POS  = "#e34948"   # signed value > 0
POLE_NEG  = "#2a78d6"   # signed value < 0
MAGNITUDE = "#2a78d6"   # unsigned magnitude
GOOD      = "#0ca30c"
WARNING   = "#fab219"
SERIOUS   = "#ec835a"
CRITICAL  = "#d03b3b"

# Point sizes, not pixels: pt scales with screen DPI, px does not, and this
# runs on a 192-DPI display where hardcoded pixels come out half-size.
SMALL_PANE_PT = 10      # Telemetry - sparse text, shrink it
MOTOR_VALUE_PT = 13  # readable at a glance without eating the pane
SPARKLINE_ROWS = 3.75   # sparkline height

# Vicon Link is deliberately the loudest pane in the window. Losing mocap is
# the one fault where the right response is "stop now", and it has to be
# readable from across the room without looking for it - at 10pt it was a grey
# line of text indistinguishable from the battery readout next to it.
VICON_BANNER_PT = 22    # the one-line verdict
VICON_VALUE_PT = 13     # the counters underneath

# When the Vicon pane escalates. Rate is in Hz against the nominal 100.
VICON_RATE_WARN_HZ = 80.0
VICON_RATE_CRITICAL_HZ = 50.0
# Must match VICON_DROP_WINDOW_SEC / VICON_OCCLUSION_WINDOW_SEC in
# telemetry_model.py - the model counts over those windows and this only labels
# them. Duplicated rather than imported so this file stays ROS-free and still
# runs standalone.
VICON_DROP_WINDOW_SEC = 3.0
VICON_OCCLUSION_WINDOW_SEC = 5.0
# Frames lost/unusable within the model's trailing window. Calibrated against
# bags/flight_20260813_143119, where a HEALTHY in-flight feed still showed a
# scattered ~2-12 dropped frames per 5 s (single frames, 20-30 ms apart) but
# exactly ZERO occlusions until the drone actually crashed. So the two get
# different sensitivities: a dropped frame is routine until it adds up to real
# blind time, an occluded frame never is.
# 25 frames at 100 Hz = 0.25 s, the same blind interval vicon_bridge's own
# staleness watchdog warns at.
VICON_DROP_WARN = 25
VICON_DROP_CRITICAL = 50
VICON_OCCLUSION_WARN = 1
VICON_OCCLUSION_CRITICAL = 25

# Full-scale values: what "100% of the bar" means. These are a display choice,
# so they live here rather than in the model, which deals in real units.
MOTOR_SPLIT_FULL_SCALE = 20000.0   # counts; <2k healthy, ~20k was the M1 crash
POSITION_ERROR_FULL_SCALE_M = 0.20
ANGLE_ERROR_FULL_SCALE_DEG = 30.0


class MeterBar(QWidget):
    """One value against a fixed scale.

    signed=True draws a centre-zero diverging bar: zero is the middle, the bar
    grows left (blue) or right (red) so direction is visible at a glance.
    signed=False is an ordinary left-to-right magnitude bar.
    Always shows the raw value in real units - a percentage alone is unreadable.
    """

    def __init__(self, label, full_scale, unit="", signed=False, decimals=0):
        super().__init__()
        self.label = label
        self.full_scale = float(full_scale)
        self.unit = unit
        self.signed = signed
        self.decimals = decimals
        self.value = None          # None = no data, drawn as '--'
        # Size from the font, not fixed pixels: on a HiDPI screen the system
        # font is ~2x and a hardcoded height clips its own labels.
        self.track_h = 10
        self.setMinimumHeight(self.fontMetrics().height() + self.track_h + 8)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_value(self, value):
        self.value = value
        self.update()              # schedules a repaint

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        text_h = self.fontMetrics().height()
        text_rect = QRectF(0, 0, w, text_h)
        track = QRectF(0, text_h + 4, w, self.track_h)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(GRIDLINE))
        p.drawRoundedRect(track, 3, 3)

        if self.value is not None:
            frac = max(-1.0, min(1.0, self.value / self.full_scale))
            if self.signed:
                mid = w / 2.0
                span = abs(frac) * (w / 2.0)
                x = mid if frac >= 0 else mid - span
                colour = POLE_POS if frac >= 0 else POLE_NEG
                p.setBrush(QColor(colour))
                p.drawRoundedRect(QRectF(x, track.top(), span, track.height()), 3, 3)
            else:
                p.setBrush(QColor(MAGNITUDE))
                p.drawRoundedRect(
                    QRectF(0, track.top(), abs(frac) * w, track.height()), 3, 3)

        if self.signed:                      # zero reference line
            p.setPen(QPen(QColor(BASELINE), 1))
            p.drawLine(int(w / 2), int(track.top()), int(w / 2), int(track.bottom()))

        p.setPen(QColor(INK))
        p.drawText(text_rect, Qt.AlignLeft | Qt.AlignVCenter, self.label)
        if self.value is None:
            p.setPen(QColor(INK_MUTED))
            text = "--"
        else:
            sign = "+" if (self.signed and self.value > 0) else ""
            text = f"{sign}{self.value:,.{self.decimals}f} {self.unit}".strip()
        p.drawText(text_rect, Qt.AlignRight | Qt.AlignVCenter, text)
        p.end()


class Sparkline(QWidget):
    """Last N samples as a thin line - the shape over time, which a bar cannot
    show. Most of the flight diagnoses were 'this is GROWING', not 'this is X'."""

    def __init__(self, full_scale, signed=False, capacity=150, colour=MAGNITUDE):
        super().__init__()
        self.full_scale = float(full_scale)
        self.signed = signed
        self.colour = colour
        self.samples = deque(maxlen=capacity)
        self.setMinimumHeight(int(self.fontMetrics().height() * SPARKLINE_ROWS))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def add(self, value):
        self.samples.append(value)   # None is allowed: it breaks the line
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        pad = 3.0
        usable = h - 2 * pad

        def y_for(v):
            frac = max(-1.0, min(1.0, v / self.full_scale))
            if self.signed:
                return pad + usable / 2.0 - frac * (usable / 2.0)
            return pad + usable - abs(frac) * usable

        # Full-scale edges, so the line's height is readable without a y-axis.
        p.setPen(QPen(QColor(GRIDLINE), 1))
        p.drawLine(0, int(pad), w, int(pad))
        if self.signed:
            p.drawLine(0, int(h - pad), w, int(h - pad))

        p.setPen(QPen(QColor(BASELINE), 1, Qt.DashLine))
        zero_y = y_for(0.0)
        p.drawLine(0, int(zero_y), w, int(zero_y))

        if len(self.samples) >= 2:
            step = w / float(max(1, self.samples.maxlen - 1))
            offset = w - step * (len(self.samples) - 1)   # newest pinned right
            pen = QPen(QColor(self.colour), 2)
            pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen)
            prev = None
            last = None
            for i, v in enumerate(self.samples):
                if v is None:
                    prev = None
                    continue
                pt = (offset + i * step, y_for(v))
                if prev is not None:
                    p.drawLine(int(prev[0]), int(prev[1]), int(pt[0]), int(pt[1]))
                prev = last = pt
            if last is not None:      # mark the newest sample
                p.setBrush(QColor(self.colour))
                p.setPen(Qt.NoPen)
                p.drawEllipse(QRectF(last[0] - 3, last[1] - 3, 6, 6))

        # Scale caption, so a line without an axis still means something.
        p.setPen(QColor(INK_MUTED))
        cap = f"±{self.full_scale:g}" if self.signed else f"0–{self.full_scale:g}"
        p.drawText(QRectF(2, 0, w, h), Qt.AlignLeft | Qt.AlignTop, cap)
        p.end()


class DroneWindow(QWidget):

    def __init__(self, node=None):
        super().__init__()
        self.node = node   # TelemetryModel; None when run standalone for layout work
        name = getattr(node, 'drone_name', None)
        self.setWindowTitle(f"Drone GUI - {name}" if name else "Drone GUI")

        main = QVBoxLayout()
        top_grid = QGridLayout()
        # Left column: the two sparse status panes packed tight at the top (0 =
        # take only what you need), estop below them with all the leftover space
        # (1) so there is no white gap.
        left_col = QVBoxLayout()
        left_col.addWidget(self.make_telemetry(), 0)
        left_col.addWidget(self.make_vicon_link(), 0)
        left_col.addWidget(self.make_estop(), 1)
        left_col.addWidget(self.make_controls(), 0)
        left_col.addWidget(self.make_logs(), 1)
        top_grid.addLayout(left_col, 0, 0, 2, 1)

        top_grid.addWidget(self.make_pose(), 0, 1)
        top_grid.addWidget(self.make_motors(), 1, 1)
        top_grid.setColumnStretch(0, 2)
        top_grid.setColumnStretch(1, 3)
        top_grid.setRowStretch(0, 1)
        top_grid.setRowStretch(1, 1)

        main.addLayout(top_grid, 1)
        self.setLayout(main)
        # No setMinimumSize(): Qt derives the minimum from the layout itself, so
        # the window can never be shrunk to the point where bars and plots
        # overlap. Forcing a smaller minimum is what caused that.
        # Height is close to the layout minimum (~2030) because the graphs are
        # tall; anything more won't fit a 2076px-tall screen with decorations.
        self.resize(1970, 1830)

    def make_telemetry(self):
        box = QGroupBox("Telemetry")
        box.setStyleSheet(f"QLabel {{ font-size: {SMALL_PANE_PT}pt; }}")

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
        layout.addStretch()      # pack to the top instead of spreading out
        box.setLayout(layout)
        return box

    def set_battery_voltage(self, voltage):
        # Thresholds match voltage_warning / voltage_critical in crazyflies.yaml.
        if voltage < 3.7:
            chunk_color = CRITICAL
        elif voltage < 3.8:
            chunk_color = WARNING
        else:
            chunk_color = GOOD
        self.battery_bar.setValue(int(voltage * 100))
        # Short text: the column is narrow, and the chunk colour already carries
        # the warn/critical state.
        self.battery_bar.setFormat(f"{voltage:.2f} V")
        self.battery_bar.setToolTip("warn below 3.80 V, critical below 3.70 V")
        self.battery_bar.setStyleSheet(
            f"QProgressBar::chunk {{ background-color: {chunk_color}; }}"
        )

    def make_pose(self):
        box = QGroupBox("Pose  —  estimate vs Vicon")
        self.setpoint_label = QLabel("Setpoint: --")
        self.estimated_label = QLabel("Estimated: --")
        self.vicon_label = QLabel("Vicon: --")
        for lbl in (self.setpoint_label, self.estimated_label, self.vicon_label):
            lbl.setStyleSheet("font-family: monospace;")

        # Unsigned: a distance has no direction.
        self.position_error_bar = MeterBar(
            "Position error", POSITION_ERROR_FULL_SCALE_M, "m", decimals=3)
        self.position_error_plot = Sparkline(POSITION_ERROR_FULL_SCALE_M)

        # Signed, one per axis. Collapsing these to a single max() hid WHICH
        # axis was wrong - and yaw was the axis that caused the runaways.
        self.roll_error_bar = MeterBar(
            "Roll error", ANGLE_ERROR_FULL_SCALE_DEG, "deg", signed=True, decimals=1)
        self.pitch_error_bar = MeterBar(
            "Pitch error", ANGLE_ERROR_FULL_SCALE_DEG, "deg", signed=True, decimals=1)
        self.yaw_error_bar = MeterBar(
            "Yaw error", ANGLE_ERROR_FULL_SCALE_DEG, "deg", signed=True, decimals=1)
        self.yaw_error_plot = Sparkline(
            ANGLE_ERROR_FULL_SCALE_DEG, signed=True, colour=POLE_POS)

        layout = QVBoxLayout()
        layout.addWidget(self.setpoint_label)
        layout.addWidget(self.estimated_label)
        layout.addWidget(self.vicon_label)
        layout.addWidget(self.position_error_bar)
        layout.addWidget(self.position_error_plot)
        layout.addWidget(self.roll_error_bar)
        layout.addWidget(self.pitch_error_bar)
        layout.addWidget(self.yaw_error_bar)
        layout.addWidget(self.yaw_error_plot)
        layout.addStretch()
        box.setLayout(layout)
        return box

    def make_vicon_link(self):
        box = QGroupBox("Vicon Link")
        box.setStyleSheet(f"QLabel {{ font-size: {VICON_VALUE_PT}pt; }}")

        # The verdict, big enough to catch the eye on its own. Everything below
        # it is the evidence for whatever it says.
        self.vicon_banner = QLabel("VICON: --")
        self.vicon_banner.setAlignment(Qt.AlignCenter)
        self.vicon_banner.setWordWrap(True)
        self._set_vicon_banner("VICON: NO DATA", None)

        self.vicon_rate_label = QLabel("Frame rate: --")
        self.dropped_frames_label = QLabel("Dropped frames: --")
        self.occluded_frames_label = QLabel("Occluded frames: --")
        self.staleness_label = QLabel("Pose to drone: --")
        # The counters are unlabelled numbers without these - "12" means nothing
        # unless you know whether it is a rate, a total, or a window.
        self.dropped_frames_label.setToolTip(
            "Frames Vicon captured that never reached this machine (gaps in "
            "frame_number). Counted for the whole session, and over the last "
            "few seconds - the second number is the one that matters in flight.")
        self.occluded_frames_label.setToolTip(
            "Frames that arrived but were unusable, so vicon_bridge discarded "
            "them and the drone's estimator got nothing: zero poses, broken "
            "quaternions, or the last pose re-served verbatim when Tracker "
            "loses the rigid body.")
        self.vicon_rate_label.setToolTip("Nominal is 100 Hz.")

        # Why the last frame was thrown away. Blank when there is nothing to say.
        self.vicon_fault_label = QLabel("")
        self.vicon_fault_label.setWordWrap(True)

        layout = QVBoxLayout()
        layout.addWidget(self.vicon_banner)
        layout.addWidget(self.vicon_rate_label)
        layout.addWidget(self.dropped_frames_label)
        layout.addWidget(self.occluded_frames_label)
        layout.addWidget(self.staleness_label)
        layout.addWidget(self.vicon_fault_label)
        layout.addStretch()      # pack to the top instead of spreading out
        box.setLayout(layout)
        return box

    def _set_vicon_banner(self, text, colour):
        """Banner text plus a filled background. colour=None means 'unknown',
        drawn in muted ink on no fill so it can't be mistaken for OK."""
        self.vicon_banner.setText(text)
        if colour is None:
            self.vicon_banner.setStyleSheet(
                f"font-size: {VICON_BANNER_PT}pt; font-weight: bold; "
                f"color: {INK_MUTED}; border: 2px solid {GRIDLINE}; "
                "border-radius: 6px; padding: 10px;")
        else:
            self.vicon_banner.setStyleSheet(
                f"font-size: {VICON_BANNER_PT}pt; font-weight: bold; "
                f"color: white; background-color: {colour}; "
                "border-radius: 6px; padding: 10px;")

    def make_motors(self):
        # && because a single & is a Qt keyboard-shortcut marker, not a literal
        box = QGroupBox("Motors && PWMs")
        self.m1_label = QLabel("M1\n--")
        self.m2_label = QLabel("M2\n--")
        self.m3_label = QLabel("M3\n--")
        self.m4_label = QLabel("M4\n--")

        motor_row = QHBoxLayout()
        for lbl in (self.m1_label, self.m2_label, self.m3_label, self.m4_label):
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(
                f"font-family: monospace; font-size: {MOTOR_VALUE_PT}pt; font-weight: bold;")
            motor_row.addWidget(lbl)

        # Signed: which motor is working harder IS the diagnosis. The bent M1
        # shaft showed as a persistent one-way split, invisible to abs().
        self.m1_m3_bar = MeterBar(
            "M1 − M3", MOTOR_SPLIT_FULL_SCALE, "counts", signed=True)
        self.m1_m3_plot = Sparkline(MOTOR_SPLIT_FULL_SCALE, signed=True)
        self.m2_m4_bar = MeterBar(
            "M2 − M4", MOTOR_SPLIT_FULL_SCALE, "counts", signed=True)
        self.m2_m4_plot = Sparkline(MOTOR_SPLIT_FULL_SCALE, signed=True)
        # Unsigned: a spread is a range, there is no direction to show.
        self.all_motor_bar = MeterBar(
            "Spread (max − min)", MOTOR_SPLIT_FULL_SCALE, "counts")

        self.saturation_label = QLabel("")
        self.saturation_label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout()
        layout.addLayout(motor_row)
        layout.addWidget(self.saturation_label)
        layout.addWidget(self.m1_m3_bar)
        layout.addWidget(self.m1_m3_plot)
        layout.addWidget(self.m2_m4_bar)
        layout.addWidget(self.m2_m4_plot)
        layout.addWidget(self.all_motor_bar)
        layout.addStretch()
        box.setLayout(layout)
        return box


    def make_controls(self):
        box = QGroupBox("Controls")

        takeoff = QPushButton("Takeoff")
        takeoff.setMinimumHeight(60)
        takeoff.setStyleSheet("background-color: #4caf50; color: white; font-weight: bold;")
        takeoff.clicked.connect(self.on_takeoff)

        land = QPushButton("Land")
        land.setMinimumHeight(60)
        land.setStyleSheet("background-color: #ff9800; color: white; font-weight: bold;")
        land.clicked.connect(self.on_land)

        button_row = QHBoxLayout()
        button_row.addWidget(takeoff)
        button_row.addWidget(land)

        self.goto_x = QLineEdit("0.0")
        self.goto_y = QLineEdit("0.0")
        self.goto_z = QLineEdit("0.0")
        self.goto_theta = QLineEdit("0.0")
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
        goto_row.addWidget(QLabel("θ°"))     # degrees; converted to rad on send
        goto_row.addWidget(self.goto_theta)
        goto_row.addWidget(goto_btn)

        pos_row = QHBoxLayout()
        current_xyz_button = QPushButton("Input Current XYZ")
        current_xyz_button.clicked.connect(self.on_input_current_xyz)
        current_fullpose_button = QPushButton("Input Current Full Pose")
        current_fullpose_button.clicked.connect(self.on_input_current_full_pose)
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
        estop_button.setMinimumWidth(200)
        estop_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        estop_button.clicked.connect(self.on_estop)
        return estop_button

    # --- command handlers ---------------------------------------------
    # Each publishes a DebugFlags message and returns immediately. No service
    # calls here: those block, and a blocked GUI is a dead emergency stop.

    # Refuse to even send a takeoff from this high up. The controller server
    # enforces the same limit independently and is the authority - this copy
    # only exists so the operator gets an instant reason in the log pane
    # instead of silence.
    TAKEOFF_MAX_GROUND_HEIGHT = 0.1  # m

    def _current_position(self):
        """Vicon truth if we have it, else the drone's own estimate, else TF."""
        if self.node is None:
            return None
        return (self.node.vicon_position() or self.node.estimated_position()
                or self.node.tf_position())

    def _current_yaw_deg(self):
        if self.node is None:
            return None
        rpy = (self.node.vicon_rpy() or self.node.estimated_rpy()
               or self.node.tf_rpy())
        return None if rpy is None else rpy[2]

    def on_takeoff(self):
        if self.node is None:
            return
        pos = self._current_position()
        if pos is not None and pos[2] > self.TAKEOFF_MAX_GROUND_HEIGHT:
            self.log.append(
                f"TAKEOFF not sent: drone is at z={pos[2]:.3f} m, above the "
                f"{self.TAKEOFF_MAX_GROUND_HEIGHT:.2f} m ground limit")
            return
        self.node.send_takeoff()
        self.log.append("TAKEOFF sent")

    def on_input_current_xyz(self):
        pos = self._current_position()
        if pos is None:
            self.log.append("No position yet - nothing to copy")
            return
        self.goto_x.setText(f"{pos[0]:.3f}")
        self.goto_y.setText(f"{pos[1]:.3f}")
        self.goto_z.setText(f"{pos[2]:.3f}")
        self.log.append(f"Copied current XYZ: {pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f}")

    def on_input_current_full_pose(self):
        pos = self._current_position()
        yaw = self._current_yaw_deg()
        if pos is None:
            self.log.append("No position yet - nothing to copy")
            return
        self.goto_x.setText(f"{pos[0]:.3f}")
        self.goto_y.setText(f"{pos[1]:.3f}")
        self.goto_z.setText(f"{pos[2]:.3f}")
        if yaw is None:
            # position-only mode: the Vicon quaternion is NaN, so there is no
            # yaw to copy. Leave the box alone rather than writing a fake 0.
            self.log.append(
                f"Copied current XYZ: {pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f} "
                "(no yaw available - position-only mode)")
            return
        self.goto_theta.setText(f"{yaw:.2f}")
        self.log.append(
            f"Copied current pose: {pos[0]:.3f} {pos[1]:.3f} {pos[2]:.3f} yaw={yaw:.2f}deg")

    def on_land(self):
        if self.node is None:
            return
        self.node.send_land()
        self.log.append("LAND sent")

    def on_estop(self):
        if self.node is None:
            return
        self.node.send_estop()
        self.log.append("EMERGENCY STOP sent")

    def on_goto(self):
        if self.node is None:
            return
        try:
            x = float(self.goto_x.text())
            y = float(self.goto_y.text())
            z = float(self.goto_z.text())
            yaw_deg = float(self.goto_theta.text() or 0.0)
        except ValueError:
            # Don't let a typo raise inside a click handler - it would surface
            # as a traceback in the terminal and nothing at all in the GUI.
            self.log.append("Go To ignored: x/y/z/θ must be numbers")
            return
        # The box is degrees (readable); GoTo.yaw is radians.
        self.node.send_goto(x, y, z, math.radians(yaw_deg))
        self.log.append(f"GOTO sent: x={x} y={y} z={z} yaw={yaw_deg}deg")

    def make_logs(self):
        box = QGroupBox("GUI Logs")
        # An explicit small minimum, deliberately BELOW QTextEdit's own ~217px
        # natural minimum (Qt honours setMinimumHeight over minimumSizeHint).
        # Without it the log alone pushes the window taller than the screen; it
        # still gets plenty of room from its stretch factor in the main layout.
        box.setMinimumHeight(40)
        self.log = QTextEdit()
        self.log.setReadOnly(True)

        layout = QVBoxLayout()
        layout.addWidget(self.log)
        box.setLayout(layout)
        return box

    @staticmethod
    def _fmt_xyz(xyz):
        if xyz is None:
            return "--"
        return f"[{xyz[0]:+.3f} {xyz[1]:+.3f} {xyz[2]:+.3f}]"

    def refresh(self):
        """Called at REFRESH_HZ from debug_gui.main(). Pulls from the node and
        pushes into the widgets. One direction only - the node never calls us.

        Every getter can return None, meaning 'no data'. That has to look
        different from zero: a 0% error bar reads as PERFECT TRACKING, which
        would be a lie at exactly the moment you're diagnosing a crash.
        """
        if self.node is None:
            return
        n = self.node

        # --- Telemetry -------------------------------------------------
        volts = n.battery_voltage()
        if volts is None or not n.status_is_fresh():
            self.battery_bar.setValue(0)
            self.battery_bar.setFormat("Battery: --")
        else:
            self.set_battery_voltage(volts)

        connected = n.is_connected()
        self._status(
            self.connected_label,
            f"Connected: {'YES' if connected else 'NO'}"
            + (f"  (RSSI {n.rssi()} dBm)" if connected and n.rssi() is not None else ""),
            GOOD if connected else CRITICAL)

        armed = n.is_armed()
        self._status(self.armed_label, f"Armed: {self._tri(armed)}",
                     None if armed is None else (WARNING if armed else GOOD))

        tumbled = n.is_tumbled()
        self._status(self.tumbled_label, f"Tumbled: {self._tri(tumbled)}",
                     None if tumbled is None else (CRITICAL if tumbled else GOOD))

        # --- Pose ------------------------------------------------------
        self.estimated_label.setText("Estimated: " + self._fmt_xyz(n.estimated_position()))
        self.vicon_label.setText("Vicon:     " + self._fmt_xyz(n.vicon_position()))

        pos_err = n.position_error_m()
        self.position_error_bar.set_value(pos_err)
        self.position_error_plot.add(pos_err)

        rpy = n.rpy_error_deg()          # None in position-only mode (NaN quat)
        self.roll_error_bar.set_value(None if rpy is None else rpy[0])
        self.pitch_error_bar.set_value(None if rpy is None else rpy[1])
        self.yaw_error_bar.set_value(None if rpy is None else rpy[2])
        self.yaw_error_plot.add(None if rpy is None else rpy[2])

        # --- Motors ----------------------------------------------------
        motors = n.motors()
        mean = sum(motors) / 4.0 if motors is not None else None
        for lbl, name, value in zip(
                (self.m1_label, self.m2_label, self.m3_label, self.m4_label),
                ("M1", "M2", "M3", "M4"),
                motors if motors is not None else (None,) * 4):
            lbl.setText(f"{name}\n{'--' if value is None else format(int(value), ',')}")
            # Colour the outlier: the motor furthest from the group mean is the
            # one to suspect. The number is still there, so colour is a hint,
            # never the only channel.
            colour = None
            if value is not None:
                if value >= 65535:
                    colour = CRITICAL
                elif abs(value - mean) > MOTOR_SPLIT_FULL_SCALE * 0.25:
                    colour = SERIOUS
                elif abs(value - mean) > MOTOR_SPLIT_FULL_SCALE * 0.10:
                    colour = WARNING
            lbl.setStyleSheet(
                f"font-family: monospace; font-size: {MOTOR_VALUE_PT}pt; font-weight: bold;"
                + (f" color: {colour};" if colour else ""))

        saturated = n.motors_saturated()
        self.saturation_label.setText("SATURATED — no attitude headroom" if saturated else "")
        self.saturation_label.setStyleSheet(
            f"color: {CRITICAL}; font-weight: bold;" if saturated else "")

        m1_m3, m2_m4 = n.m1_m3_split(), n.m2_m4_split()
        self.m1_m3_bar.set_value(m1_m3)
        self.m1_m3_plot.add(m1_m3)
        self.m2_m4_bar.set_value(m2_m4)
        self.m2_m4_plot.add(m2_m4)
        self.all_motor_bar.set_value(n.all_motor_spread())

        # --- Vicon link ------------------------------------------------
        self._refresh_vicon(n)

    def _refresh_vicon(self, n):
        """The Vicon pane. Worst condition wins the banner; the lines below it
        are the evidence, so the operator can tell WHICH failure this is."""
        pose_ok = n.vicon_is_fresh()
        raw_fresh = n.vicon_raw_is_fresh()
        dropped = n.vicon_dropped_frames()
        occluded = n.vicon_occluded_frames()
        rate = n.vicon_frame_rate_hz()

        # --- counters --------------------------------------------------
        def loss_colour(recent, warn, critical):
            if recent >= critical:
                return CRITICAL
            if recent >= warn:
                return WARNING
            return GOOD

        if dropped is None:
            self._status(self.dropped_frames_label,
                         "Dropped frames: n/a (no raw feed)", None, VICON_VALUE_PT)
        else:
            total, recent = dropped
            self._status(self.dropped_frames_label,
                         f"Dropped frames: {recent} in {int(VICON_DROP_WINDOW_SEC)}s "
                         f"({total:,} total)",
                         loss_colour(recent, VICON_DROP_WARN, VICON_DROP_CRITICAL),
                         VICON_VALUE_PT)

        if occluded is None:
            self._status(self.occluded_frames_label,
                         "Occluded frames: n/a (no raw feed)", None, VICON_VALUE_PT)
        else:
            total, recent = occluded
            self._status(self.occluded_frames_label,
                         f"Occluded frames: {recent} in "
                         f"{int(VICON_OCCLUSION_WINDOW_SEC)}s ({total:,} total)",
                         loss_colour(recent, VICON_OCCLUSION_WARN,
                                     VICON_OCCLUSION_CRITICAL),
                         VICON_VALUE_PT)

        if rate is None:
            self._status(self.vicon_rate_label, "Frame rate: --", None, VICON_VALUE_PT)
        else:
            rate_colour = (CRITICAL if rate < VICON_RATE_CRITICAL_HZ
                           else WARNING if rate < VICON_RATE_WARN_HZ else GOOD)
            self._status(self.vicon_rate_label,
                         f"Frame rate: {rate:.0f} Hz (nominal 100)",
                         rate_colour, VICON_VALUE_PT)

        self._status(self.staleness_label,
                     f"Pose to drone: {'flowing' if pose_ok else 'STOPPED'}",
                     GOOD if pose_ok else CRITICAL, VICON_VALUE_PT)

        fault = n.vicon_last_fault()
        if fault is None or fault[1] > VICON_OCCLUSION_WINDOW_SEC:
            self.vicon_fault_label.setText("")
        else:
            reason, age = fault
            self.vicon_fault_label.setText(f"last bad frame: {reason} ({age:.1f}s ago)")
            self.vicon_fault_label.setStyleSheet(
                f"font-size: {VICON_VALUE_PT}pt; color: {SERIOUS};")

        # --- the verdict -----------------------------------------------
        # Ordered worst-first: the first condition that matches is the one
        # shown, so a real outage is never masked by a milder warning.
        drop_recent = dropped[1] if dropped else 0
        occl_recent = occluded[1] if occluded else 0
        if not pose_ok and not raw_fresh:
            self._set_vicon_banner("VICON LOST\nNO DATA AT ALL", CRITICAL)
        elif not pose_ok:
            # Raw frames are arriving but none survive - the drone is getting
            # nothing, which looks identical to a dead feed from onboard.
            self._set_vicon_banner("VICON LOST\nEVERY FRAME UNUSABLE", CRITICAL)
        elif occl_recent >= VICON_OCCLUSION_CRITICAL:
            self._set_vicon_banner("VICON DEGRADED\nMARKERS OCCLUDED", CRITICAL)
        elif drop_recent >= VICON_DROP_CRITICAL:
            self._set_vicon_banner("VICON DEGRADED\nLOSING FRAMES", CRITICAL)
        elif rate is not None and rate < VICON_RATE_CRITICAL_HZ:
            self._set_vicon_banner("VICON DEGRADED\nFRAME RATE COLLAPSED", CRITICAL)
        elif (occl_recent >= VICON_OCCLUSION_WARN
                or drop_recent >= VICON_DROP_WARN
                or (rate is not None and rate < VICON_RATE_WARN_HZ)):
            self._set_vicon_banner("VICON UNSTEADY", WARNING)
        elif dropped is None:
            # Poses are flowing, so tracking is fine; we just cannot see the
            # raw feed to count against it. Not a fault, but not a clean bill.
            self._set_vicon_banner("VICON OK\n(frame counts unavailable)", GOOD)
        else:
            self._set_vicon_banner("VICON OK", GOOD)

    def _status(self, label, text, colour, pt=SMALL_PANE_PT):
        """Set text plus a status colour. colour=None leaves it in default ink -
        used for '--', because unknown is not a status."""
        label.setText(text)
        label.setStyleSheet(
            f"font-size: {pt}pt; font-weight: bold; color: {colour};"
            if colour else f"font-size: {pt}pt;")

    @staticmethod
    def _tri(value):
        """None -> '--' so 'no data' never masquerades as 'No'."""
        return "--" if value is None else ("YES" if value else "no")


class _FakeNode:
    """Stand-in for TelemetryModel so this file runs with no ROS at all.
    Feeds plausible drifting values so the bars and plots actually move."""

    def __init__(self):
        self.t = 0.0

    def tick(self):
        self.t += 0.1

    import math as _m

    def battery_voltage(self):  return 3.9 - 0.02 * self.t
    def rssi(self):             return 55
    def status_is_fresh(self):  return True
    def is_connected(self):     return True
    def is_armed(self):         return True
    def is_tumbled(self):       return False

    # --- Vicon link. Cycles through the states every 20 s so the banner and
    # the counter colours can all be seen without a mocap rig.
    def _phase(self):
        return int(self.t / 5.0) % 4      # 0 ok, 1 unsteady, 2 degraded, 3 lost

    def vicon_is_fresh(self):     return self._phase() != 3
    def vicon_raw_is_fresh(self): return True
    def vicon_raw_connected(self): return True

    def vicon_frame_rate_hz(self):
        return {0: 100.0, 1: 74.0, 2: 96.0, 3: 40.0}[self._phase()]

    def vicon_dropped_frames(self):
        recent = {0: 0, 1: 3, 2: 61, 3: 140}[self._phase()]
        return (int(self.t * 4) + recent, recent)   # total includes the window

    def vicon_occluded_frames(self):
        recent = {0: 0, 1: 1, 2: 12, 3: 90}[self._phase()]
        return (int(self.t * 2) + recent, recent)

    def vicon_last_fault(self):
        if self._phase() == 0:
            return None
        return ("frozen frame", 0.4)

    MAX_PWM = 65535   # 16-bit; a real motor cannot be commanded past this

    def motors(self):
        import math
        base = 45000 + 3000 * math.sin(self.t / 3.0)
        drift = 120 * self.t          # M1 slowly running away, like the bent shaft
        return tuple(min(self.MAX_PWM, v) for v in
                     (base + drift, base - 200, base - drift * 0.2, base + 150))

    def motors_saturated(self):
        return max(self.motors()) >= self.MAX_PWM

    def m1_m3_split(self):    m = self.motors(); return m[0] - m[2]
    def m2_m4_split(self):    m = self.motors(); return m[1] - m[3]
    def all_motor_spread(self): m = self.motors(); return max(m) - min(m)

    def estimated_position(self):
        import math
        return (0.02 * math.sin(self.t), 0.01 * math.cos(self.t), 1.0)

    def vicon_position(self):
        return (0.0, 0.0, 1.0)

    def estimated_rpy(self):
        import math
        return (1.5 * math.sin(self.t), -2.0, 12.0)

    def vicon_rpy(self):
        return (0.0, 0.0, 11.2)

    # Command stubs. The real TelemetryModel publishes DebugFlags here; standalone
    # there is no ROS, so these just report so the buttons stay clickable.
    def send_takeoff(self, height=0.5):
        print(f"[fake] takeoff height={height}")

    def send_land(self):
        print("[fake] land")

    def send_estop(self):
        print("[fake] estop")

    def send_goto(self, x, y, z, yaw=0.0, duration=2.0):
        print(f"[fake] goto x={x} y={y} z={z} yaw={yaw}")

    def position_error_m(self):
        import math
        return math.dist(self.estimated_position(), self.vicon_position())

    def rpy_error_deg(self):
        import math
        return (1.5 * math.sin(self.t), -2.0 * math.cos(self.t / 2), 0.8 * self.t)


def main(args=None):
    """Layout-only run: no ROS at all, driven by fake data. For iterating on
    sizes/colours/plots. The real entry point is debug_gui:main."""
    import signal
    from PyQt5.QtCore import QTimer

    # app.exec_() blocks inside C++, so Python never sees SIGINT and Ctrl-C is
    # ignored. Restoring the default OS handler lets it kill the process.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    fake = _FakeNode()
    window = DroneWindow(fake)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: (fake.tick(), window.refresh()))
    timer.start(100)

    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
