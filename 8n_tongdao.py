import math
import random
import re
import sys
import time

import numpy as np
import pyqtgraph as pg
import serial
from serial.tools import list_ports

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QApplication,QCheckBox,QComboBox,QDoubleSpinBox,QFrame,QHBoxLayout,QLabel,
    QMainWindow,QPushButton,QSpinBox,QVBoxLayout,QWidget,QTextEdit,
)

# === 配置区 ===
DEFAULT_SERIAL_PORT = "COM3"
BAUD_RATE = 921600
DEFAULT_WINDOW_SIZE = 500
UI_REFRESH_MS = 20
DEFAULT_MANUAL_RANGE_V = 0.002
MAX_LINES_PER_TICK = 80
MAX_RX_BUFFER_BYTES = 65536
CHANNELS_PER_ADS = 8
STATUS_BYTES_PER_ADS = 3
BYTES_PER_CHANNEL = 3
BYTES_PER_ADS_FRAME = STATUS_BYTES_PER_ADS + CHANNELS_PER_ADS * BYTES_PER_CHANNEL
SUPPORTED_CHANNEL_COUNTS = (8, 16, 24, 32)
V_PER_LSB = 4.5 / (8.0 * 8388607.0)
BINARY_SYNC = b"\xA5\x5A"
BINARY_PROTOCOL_VERSION = 0x01
BINARY_HEADER_LEN = 8
BINARY_CHECKSUM_LEN = 1

# 兼容旧固件：如果 Arduino 仍输出十六进制文本帧，打开这个开关也能解析。
PARSE_RAW_HEX_LINES = False
CHANNEL_PAYLOAD_RE = re.compile(r"^[\s,0-9+\-.eE]+$")


class ADS1299Visualizer(QMainWindow):
    def __init__(self):
        super().__init__()

        self.ser = None
        self.window_size = DEFAULT_WINDOW_SIZE
        self.channel_count = 0
        self.data_stack = np.zeros((0, self.window_size), dtype=np.float32)
        self.plots = []
        self.zero_curves = []
        self.curves = []
        self.packet_counter = 0
        self.last_packet_counter = 0
        self.last_rate_time = time.time()
        self.is_paused = False
        self.manual_range_v = DEFAULT_MANUAL_RANGE_V

        self.rx_buffer = bytearray()
        self.last_status_text = ""
        self.last_frame_seq = None
        self.dropped_frame_counter = 0
        self.debug_frame_count = 0  # 用于调试前几帧

        self._setup_window()
        self._setup_ui()
        self.refresh_ports()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(UI_REFRESH_MS)

    def _setup_window(self):
        self.setWindowTitle("ADS1299 实时可视化")
        self.resize(1320, 860)
        pg.setConfigOptions(antialias=False)

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(6, 6, 6, 6)   # 【调节外边距】(左, 上, 右, 下)，控制整个窗口里的所有组件距离边缘距离
        root_layout.setSpacing(6)

        top_bar = QFrame()
        top_layout = QVBoxLayout(top_bar)
        top_layout.setContentsMargins(8, 6, 8, 6)
        top_layout.setSpacing(5)

        row_conn = QHBoxLayout()
        row_conn.setSpacing(8)
        row_ops = QHBoxLayout()
        row_ops.setSpacing(8)
        row_status = QHBoxLayout()
        row_status.setSpacing(8)

        title = QLabel("ADS1299 Monitor")

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(240)

        self.refresh_btn = QPushButton("刷新端口")
        self.refresh_btn.setMinimumWidth(90)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setMinimumWidth(95)
        self.connect_btn.setCheckable(True)
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.setMinimumWidth(80)
        self.pause_btn.setCheckable(True)
        self.clear_btn = QPushButton("清空缓冲")
        self.clear_btn.setMinimumWidth(95)

        self.serial_lines = []

        self.mode1_btn = QPushButton("连续")
        self.mode1_btn.setMinimumWidth(70)
        self.mode2_btn = QPushButton("阻抗")
        self.mode2_btn.setMinimumWidth(70)
        self.mode3_btn = QPushButton("自检")
        self.mode3_btn.setMinimumWidth(70)

        self.autoscale_checkbox = QCheckBox("Y轴自适应")
        self.autoscale_checkbox.setChecked(True)

        self.window_spin = QSpinBox()
        self.window_spin.setRange(200, 8000)
        self.window_spin.setValue(self.window_size)
        self.window_spin.setSingleStep(250)
        self.window_spin.setSuffix(" 点")
        self.window_spin.setMinimumWidth(100)

        self.range_spin = QDoubleSpinBox()
        self.range_spin.setRange(0.000001, 10000.0)
        self.range_spin.setDecimals(6)
        self.range_spin.setSingleStep(0.0005)
        self.range_spin.setValue(self.manual_range_v)
        self.range_spin.setSuffix(" V")
        self.range_spin.setMinimumWidth(120)

        self.conn_label = QLabel("连接: 未连接")
        self.channel_label = QLabel("通道: 等待数据")
        self.rate_label = QLabel("包速: 0.0 pkt/s")

        row_conn.addWidget(title)
        row_conn.addSpacing(12)
        row_conn.addWidget(QLabel("串口"))
        row_conn.addWidget(self.port_combo)
        row_conn.addWidget(self.refresh_btn)
        row_conn.addWidget(self.connect_btn)
        row_conn.addStretch(1)

        row_ops.addWidget(self.pause_btn)
        row_ops.addWidget(self.clear_btn)
        row_ops.addSpacing(8)
        row_ops.addWidget(QLabel("模式"))
        row_ops.addWidget(self.mode1_btn)
        row_ops.addWidget(self.mode2_btn)
        row_ops.addWidget(self.mode3_btn)
        row_ops.addSpacing(8)
        row_ops.addWidget(self.autoscale_checkbox)
        row_ops.addWidget(QLabel("量程(±)"))
        row_ops.addWidget(self.range_spin)
        row_ops.addWidget(QLabel("窗口"))
        row_ops.addWidget(self.window_spin)
        row_ops.addStretch(1)

        row_status.addWidget(self.conn_label)
        row_status.addWidget(self.channel_label)
        row_status.addWidget(self.rate_label)
        row_status.addStretch(1)

        top_layout.addLayout(row_conn)
        top_layout.addLayout(row_ops)
        top_layout.addLayout(row_status)

        root_layout.addWidget(top_bar)
        
        self.serial_output = QTextEdit()
        self.serial_output.setReadOnly(True)
        self.serial_output.setMaximumHeight(56)
        self.serial_output.setStyleSheet("font-family: Consolas, monospace; background-color: #f0f0f0;")
        root_layout.addWidget(self.serial_output)

        self.win = pg.GraphicsLayoutWidget()
        try:
            self.win.ci.setSpacing(2)
            self.win.ci.setContentsMargins(1, 1, 1, 1)
        except Exception:
            pass
        root_layout.addWidget(self.win, stretch=1)


        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.pause_btn.toggled.connect(self.on_pause_toggled)
        self.clear_btn.clicked.connect(self.clear_plot_data)
        self.autoscale_checkbox.toggled.connect(self.on_autoscale_toggled)
        self.range_spin.valueChanged.connect(self.on_manual_range_changed)
        self.window_spin.valueChanged.connect(self.on_window_size_changed)


        self.mode1_btn.clicked.connect(lambda: self.send_mode_command("1"))
        self.mode2_btn.clicked.connect(lambda: self.send_mode_command("2"))
        self.mode3_btn.clicked.connect(lambda: self.send_mode_command("3"))



    def refresh_ports(self):
        current = self.port_combo.currentData()
        ports = sorted(list_ports.comports(), key=lambda p: p.device)

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for p in ports:
            self.port_combo.addItem(f"{p.device} | {p.description}", p.device)

        if not ports:
            self.port_combo.addItem(DEFAULT_SERIAL_PORT, DEFAULT_SERIAL_PORT)

        if current is not None:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        self.port_combo.blockSignals(False)

    def _selected_port(self):
        data = self.port_combo.currentData()
        if data:
            return str(data)
        text = self.port_combo.currentText().strip()
        if "|" in text:
            return text.split("|", 1)[0].strip()
        return text or DEFAULT_SERIAL_PORT

    def toggle_connection(self):
        if self.ser is not None and self.ser.is_open:
            self._disconnect_serial()
            return

        self.refresh_ports()
        port = self._selected_port()
        if not port:
            self.conn_label.setText("连接失败: 未选择串口")
            return

        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.02)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.rx_buffer.clear()
            self.last_frame_seq = None
            self.dropped_frame_counter = 0
            self.conn_label.setText(f"连接: {port} @ {BAUD_RATE}")
            self.connect_btn.setText("断开")
        except Exception as exc:
            self.ser = None
            self.connect_btn.setChecked(False)
            reason = str(exc).strip() or exc.__class__.__name__
            if isinstance(exc, PermissionError) or "拒绝访问" in reason:
                self.conn_label.setText("连接失败: 串口被占用(可能已有另一个本程序在运行)")
            else:
                self.conn_label.setText(f"连接失败: {reason}")
            available = ", ".join(p.device for p in list_ports.comports()) or "无"
            print(f"无法打开串口 {port}: {reason}")
            print(f"当前可用串口: {available}")

    def _disconnect_serial(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.rx_buffer.clear()
        self.last_frame_seq = None
        self.connect_btn.setChecked(False)
        self.connect_btn.setText("连接")
        self.conn_label.setText("连接: 未连接")

    def on_pause_toggled(self, checked):
        self.is_paused = checked
        self.pause_btn.setText("继续" if checked else "暂停")

    def on_window_size_changed(self, value):
        if value == self.window_size:
            return
        self.window_size = value
        if self.channel_count > 0:
            self.data_stack = np.zeros((self.channel_count, self.window_size), dtype=np.float32)
            self.clear_plot_data()

    def _apply_manual_y_range(self):
        y = max(float(self.manual_range_v), 1e-9)
        for p in self.plots:
            p.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
            p.setYRange(-y, y, padding=0.0)

    def on_manual_range_changed(self, value):
        self.manual_range_v = max(float(value), 1e-9)
        # 用户输入量程后，默认切到固定量程模式以立即生效
        if self.autoscale_checkbox.isChecked():
            self.autoscale_checkbox.setChecked(False)
            return
        self._apply_manual_y_range()

    def on_autoscale_toggled(self, checked):
        for p in self.plots:
            p.enableAutoRange(axis=pg.ViewBox.YAxis, enable=checked)
        if not checked:
            self._apply_manual_y_range()

    def clear_plot_data(self):
        if self.channel_count <= 0:
            return
        self.data_stack.fill(0.0)
        x = np.arange(self.window_size, dtype=np.int32)
        zero = np.zeros(self.window_size, dtype=np.float32)
        for zc in self.zero_curves:
            zc.setData(x, zero)
        for i, curve in enumerate(self.curves):
            curve.setData(x, self.data_stack[i])
        self.packet_counter = 0
        self.last_packet_counter = 0
        self.dropped_frame_counter = 0
        self.last_frame_seq = None
        self.last_rate_time = time.time()
        self.rate_label.setText("包速: 0.0 pkt/s")

    def _append_packet_values(self, values):
        packet_channels = int(values.size)
        if packet_channels not in SUPPORTED_CHANNEL_COUNTS:
            self.channel_label.setText(f"通道: 非法帧({packet_channels})")
            return

        if packet_channels != self.channel_count:
            self._configure_plots(packet_channels)

        self.data_stack = np.roll(self.data_stack, -1, axis=1)
        self.data_stack[:, -1] = values
        self.packet_counter += 1
        if self.last_status_text:
            self.channel_label.setText(
                f"通道: {packet_channels} ({packet_channels // 8} 片ADS)  FLAG:{self.last_status_text}"
            )

    def send_mode_command(self, mode_char):
        mode_map = {
            "1": "continuous",
            "2": "impedance",
            "3": "selftest",
        }

        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write(mode_char.encode("ascii"))
        except Exception:
            self._disconnect_serial()

    def _configure_plots(self, channel_count):
        self.win.clear()
        self.plots.clear()
        self.zero_curves.clear()
        self.curves.clear()

        if channel_count <= 0:
            self.channel_count = 0
            self.channel_label.setText("通道: 等待数据")
            return

        # 8通道使用2列，16/24/32通道使用4列，避免32通道时纵向过长。
        cols = 2 if channel_count <= 8 else 4
        rows = math.ceil(channel_count / cols)
        x = np.arange(self.window_size, dtype=np.int32)
        zero = np.zeros(self.window_size, dtype=np.float32)
        left_axis_width = 48 if channel_count <= 16 else 40

        for idx in range(channel_count):
            row = idx // cols
            col = idx % cols

            p = self.win.addPlot(row=row, col=col, title=f"CH {idx + 1}")
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setMenuEnabled(False)
            p.setMouseEnabled(x=False, y=False)
            p.getAxis("left").setWidth(left_axis_width)

            if row < rows - 1:
                p.getAxis("bottom").setStyle(showValues=False)

            # 始终显示 y=0 参考线，便于观察基线偏移
            zero_curve = p.plot(x, zero, pen=pg.mkPen(color=(255, 180, 0), width=1))
            curve = p.plot(pen=pg.mkPen(color=(95, 245, 190), width=1))
            self.plots.append(p)
            self.zero_curves.append(zero_curve)
            self.curves.append(curve)

        self.channel_count = channel_count
        self.channel_label.setText(f"通道: {channel_count} ({channel_count // 8} 片ADS)")
        self.data_stack = np.zeros((self.channel_count, self.window_size), dtype=np.float32)
        self.on_autoscale_toggled(self.autoscale_checkbox.isChecked())

    @staticmethod
    def _parse_channel_line(line):
        channel_pos = line.find("channel:")
        if channel_pos < 0:
            return None
        payload = line[channel_pos + len("channel:"):].strip()
        if not payload:
            return None
        if not CHANNEL_PAYLOAD_RE.fullmatch(payload):
            return None
        try:
            values = np.fromstring(payload, sep=",", dtype=np.float32)
        except ValueError:
            return None
        if values.size not in SUPPORTED_CHANNEL_COUNTS:
            return None
        return values

    def _parse_channel_bytes(self, data_bytes: bytes, debug=False):
        """Parse one daisy-chain ADS1299 frame into channel voltages.

        Frame format:
        per chip: [0..2] STATUS, then 8 channels * 3 bytes, 24-bit two's complement.
        Returns numpy array of 8/16/24/32 float32 voltages (V) or None on error.
        """
        if not data_bytes or len(data_bytes) % BYTES_PER_ADS_FRAME != 0:
            return None
        chip_count = len(data_bytes) // BYTES_PER_ADS_FRAME
        channel_count = chip_count * CHANNELS_PER_ADS
        if channel_count not in SUPPORTED_CHANNEL_COUNTS:
            return None

        try:
            ch_vals = []
            status_vals = []
            
            if debug:
                hex_str = ' '.join(f'{b:02X}' for b in data_bytes)
                print(f"[RAW BYTES] {hex_str}")
            
            for chip in range(chip_count):
                chip_base = chip * BYTES_PER_ADS_FRAME
                status_val = int.from_bytes(data_bytes[chip_base:chip_base + STATUS_BYTES_PER_ADS], byteorder='big', signed=False)
                status_vals.append(status_val >> 12)
                if debug:
                    print(f"[STATUS] chip {chip + 1}: 0x{status_val:06X}")

                for ch in range(CHANNELS_PER_ADS):
                    base = chip_base + STATUS_BYTES_PER_ADS + ch * BYTES_PER_CHANNEL
                    raw = int.from_bytes(data_bytes[base:base + BYTES_PER_CHANNEL], byteorder='big', signed=True)
                    voltage = raw * V_PER_LSB
                    ch_vals.append(voltage)

                    if debug:
                        global_ch = chip * CHANNELS_PER_ADS + ch + 1
                        print(f"[PARSE] CH{global_ch} | bytes: {data_bytes[base]:02X} {data_bytes[base+1]:02X} {data_bytes[base+2]:02X} | raw: {raw:9d} | volts: {voltage:10.6f} V")

            self.last_status_text = ",".join(f"{v:X}" for v in status_vals)
            return np.asarray(ch_vals, dtype=np.float32)
        except Exception as e:
            print(f"[ERROR] Parse failed: {e}")
            return None

    def _append_serial_text(self, text):
        lines = [line.strip() for line in text.replace("\r", "\n").split("\n") if line.strip()]
        if not lines:
            return

        for line in lines:
            values = self._parse_channel_line(line)
            if values is None and PARSE_RAW_HEX_LINES:
                values = self._parse_hex_string(line)
            if values is not None:
                self._append_packet_values(values)
            else:
                self.serial_lines.append(line)

        if len(self.serial_lines) > 4:
            self.serial_lines = self.serial_lines[-4:]
        self.serial_output.setPlainText("\n".join(self.serial_lines))

    def _consume_text_bytes(self, raw):
        if not raw:
            return
        text = raw.decode("utf-8", errors="ignore")
        self._append_serial_text(text)

    def _consume_complete_text_lines(self):
        newline_idx = max(self.rx_buffer.rfind(b"\n"), self.rx_buffer.rfind(b"\r"))
        if newline_idx >= 0:
            raw = bytes(self.rx_buffer[:newline_idx + 1])
            del self.rx_buffer[:newline_idx + 1]
            self._consume_text_bytes(raw)
        elif len(self.rx_buffer) > MAX_RX_BUFFER_BYTES:
            raw = bytes(self.rx_buffer)
            self.rx_buffer.clear()
            self._consume_text_bytes(raw)

    def _binary_payload_is_valid(self, chip_count, payload_len):
        channel_count = chip_count * CHANNELS_PER_ADS
        return (
            channel_count in SUPPORTED_CHANNEL_COUNTS
            and payload_len == chip_count * BYTES_PER_ADS_FRAME
        )

    def _process_rx_buffer(self):
        processed = 0

        while processed < MAX_LINES_PER_TICK:
            sync_idx = self.rx_buffer.find(BINARY_SYNC)

            if sync_idx < 0:
                self._consume_complete_text_lines()
                break

            if sync_idx > 0:
                raw = bytes(self.rx_buffer[:sync_idx])
                del self.rx_buffer[:sync_idx]
                self._consume_text_bytes(raw)
                continue

            if len(self.rx_buffer) < BINARY_HEADER_LEN:
                break

            version = self.rx_buffer[2]
            chip_count = self.rx_buffer[3]
            payload_len = self.rx_buffer[4] | (self.rx_buffer[5] << 8)
            seq = self.rx_buffer[6] | (self.rx_buffer[7] << 8)

            if version != BINARY_PROTOCOL_VERSION or not self._binary_payload_is_valid(chip_count, payload_len):
                del self.rx_buffer[0]
                continue

            frame_len = BINARY_HEADER_LEN + payload_len + BINARY_CHECKSUM_LEN
            if len(self.rx_buffer) < frame_len:
                break

            checksum = self.rx_buffer[frame_len - 1]
            calc_checksum = sum(self.rx_buffer[2:frame_len - 1]) & 0xFF
            if checksum != calc_checksum:
                del self.rx_buffer[0]
                continue

            payload_start = BINARY_HEADER_LEN
            payload_end = payload_start + payload_len
            payload = bytes(self.rx_buffer[payload_start:payload_end])
            del self.rx_buffer[:frame_len]

            if self.last_frame_seq is not None:
                missed = (seq - self.last_frame_seq - 1) & 0xFFFF
                if missed < 0x8000:
                    self.dropped_frame_counter += missed
            self.last_frame_seq = seq

            values = self._parse_channel_bytes(payload)
            if values is not None:
                self._append_packet_values(values)
                processed += 1

        if len(self.rx_buffer) > MAX_RX_BUFFER_BYTES:
            del self.rx_buffer[:-MAX_RX_BUFFER_BYTES]

    def _update_rate_label(self):
        now = time.time()
        dt = now - self.last_rate_time
        if dt < 0.5:
            return
        pps = (self.packet_counter - self.last_packet_counter) / dt
        drop_text = f"  丢帧:{self.dropped_frame_counter}" if self.dropped_frame_counter else ""
        self.rate_label.setText(f"包速: {pps:.1f} pkt/s{drop_text}")
        self.last_packet_counter = self.packet_counter
        self.last_rate_time = now

    def _parse_hex_string(self, text: str):
        """Parse 27*N-byte ADS1299 frame from hex string into channel voltages."""
        parts = text.split()
        if len(parts) % BYTES_PER_ADS_FRAME != 0:
            return None
        channel_count = (len(parts) // BYTES_PER_ADS_FRAME) * CHANNELS_PER_ADS
        if channel_count not in SUPPORTED_CHANNEL_COUNTS:
            return None
        try:
            data_bytes = bytes(int(p, 16) for p in parts)
        except ValueError:
            return None
        return self._parse_channel_bytes(data_bytes)

    def update_plot(self):
        if self.is_paused:
            # 暂停时清空串口缓存，避免恢复后一次性堆积大量旧数据
            if self.ser is not None and self.ser.is_open and self.ser.in_waiting > 0:
                self.ser.reset_input_buffer()
                self.rx_buffer.clear()
            return

        if self.ser is None or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                self.rx_buffer.extend(self.ser.read(waiting))
        except Exception:
            self._disconnect_serial()
            return

        if self.rx_buffer:
            self._process_rx_buffer()

        if self.channel_count > 0:
            x = np.arange(self.window_size, dtype=np.int32)
            for i, curve in enumerate(self.curves):
                curve.setData(x, self.data_stack[i])

        self._update_rate_label()

    def closeEvent(self, event):
        self._disconnect_serial()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = ADS1299Visualizer()
    viewer.show()
    sys.exit(app.exec())
