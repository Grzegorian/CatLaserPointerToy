import sys
import serial
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QComboBox, QWidget, QSpinBox)
from PyQt5.QtCore import Qt


class ServoController:
    def __init__(self, port='COM3', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        time.sleep(0.1)

    def close(self):
        self.ser.close()

    def calculate_checksum(self, packet):
        checksum = 0
        for byte in packet[2:]:
            checksum += byte
        checksum = (~checksum) & 0xFF
        return checksum

    def send_packet(self, packet):
        self.ser.write(bytes(packet))
        time.sleep(0.01)

    def receive_packet(self, timeout=1):
        start_time = time.time()
        response = []

        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                byte = ord(self.ser.read(1))
                if len(response) == 0 and byte == 0xFF:
                    response.append(byte)
                elif len(response) == 1 and byte == 0xFF:
                    response.append(byte)
                    break
                else:
                    response = []

        if len(response) < 2:
            return None

        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                byte = ord(self.ser.read(1))
                response.append(byte)
                if len(response) >= 6 and len(response) == response[3] + 4:
                    break

        return response if len(response) >= 6 else None

    def ping_servo(self, servo_id):
        packet = [0xFF, 0xFF, servo_id, 0x02, 0x01]
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)

        self.ser.flushInput()
        self.send_packet(packet)
        response = self.receive_packet()

        if response is None:
            return False
        return response[4] == 0x00  # True if no error

    def set_servo_position(self, servo_id, position, speed=100):
        # Position is 0-1023 for SC servos (as per ST3215 manual)
        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF

        # WRITE DATA instruction to address 0x2A (position control)
        packet = [
            0xFF, 0xFF,  # Header
            servo_id,  # Servo ID
            0x09,  # Length (N+3 where N=6)
            0x03,  # Instruction (WRITE DATA)
            0x2A,  # Address (position control)
            pos_low, pos_high,  # Position (little-endian)
            0x00, 0x00,  # Time (0 for immediate)
            speed_low, speed_high  # Speed
        ]
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)

        self.ser.flushInput()
        self.send_packet(packet)
        response = self.receive_packet()

        return response is not None and response[4] == 0x00


class ServoControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.controller = None
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Servo Controller')
        self.setGeometry(100, 100, 400, 300)

        # Main widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # Connection controls
        self.port_combo = QComboBox()
        self.port_combo.addItems(['COM3', 'COM4', 'COM5', 'COM6'])
        self.connect_btn = QPushButton('Connect')
        self.connect_btn.clicked.connect(self.toggle_connection)

        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel('Port:'))
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(self.connect_btn)
        layout.addLayout(conn_layout)

        # Servo selection
        self.servo_combo = QComboBox()
        self.servo_combo.addItems(['2', '3'])
        self.ping_btn = QPushButton('Ping Servo')
        self.ping_btn.clicked.connect(self.ping_servo)
        self.ping_btn.setEnabled(False)

        servo_select_layout = QHBoxLayout()
        servo_select_layout.addWidget(QLabel('Servo ID:'))
        servo_select_layout.addWidget(self.servo_combo)
        servo_select_layout.addWidget(self.ping_btn)
        layout.addLayout(servo_select_layout)

        # Position control
        self.position_label = QLabel('Position (0-1023):')
        self.position_spin = QSpinBox()
        self.position_spin.setRange(0, 1023)
        self.position_spin.setValue(512)

        # Arrow buttons
        btn_up = QPushButton('↑')
        btn_up.setFixedSize(50, 30)
        btn_up.clicked.connect(lambda: self.adjust_position(10))

        btn_down = QPushButton('↓')
        btn_down.setFixedSize(50, 30)
        btn_down.clicked.connect(lambda: self.adjust_position(-10))

        btn_left = QPushButton('←')
        btn_left.setFixedSize(50, 30)
        btn_left.clicked.connect(lambda: self.adjust_position(-1))

        btn_right = QPushButton('→')
        btn_right.setFixedSize(50, 30)
        btn_right.clicked.connect(lambda: self.adjust_position(1))

        btn_home = QPushButton('Home (512)')
        btn_home.clicked.connect(lambda: self.set_position(512))

        # Position control layouts
        pos_ctrl_layout = QVBoxLayout()

        spin_layout = QHBoxLayout()
        spin_layout.addWidget(self.position_label)
        spin_layout.addWidget(self.position_spin)
        pos_ctrl_layout.addLayout(spin_layout)

        arrow_layout = QHBoxLayout()
        arrow_layout.addStretch()
        arrow_layout.addWidget(btn_up)
        arrow_layout.addStretch()
        pos_ctrl_layout.addLayout(arrow_layout)

        mid_arrow_layout = QHBoxLayout()
        mid_arrow_layout.addWidget(btn_left)
        mid_arrow_layout.addStretch()
        mid_arrow_layout.addWidget(btn_right)
        pos_ctrl_layout.addLayout(mid_arrow_layout)

        bottom_arrow_layout = QHBoxLayout()
        bottom_arrow_layout.addStretch()
        bottom_arrow_layout.addWidget(btn_down)
        bottom_arrow_layout.addStretch()
        pos_ctrl_layout.addLayout(bottom_arrow_layout)

        pos_ctrl_layout.addWidget(btn_home)

        layout.addLayout(pos_ctrl_layout)

        # Status label
        self.status_label = QLabel('Status: Disconnected')
        layout.addWidget(self.status_label)

        # Enable/disable controls based on connection
        self.update_controls(False)

    def toggle_connection(self):
        if self.controller is None:
            try:
                port = self.port_combo.currentText()
                self.controller = ServoController(port=port)
                self.status_label.setText(f'Status: Connected to {port}')
                self.connect_btn.setText('Disconnect')
                self.update_controls(True)
            except serial.SerialException as e:
                self.status_label.setText(f'Error: {str(e)}')
        else:
            self.controller.close()
            self.controller = None
            self.status_label.setText('Status: Disconnected')
            self.connect_btn.setText('Connect')
            self.update_controls(False)

    def update_controls(self, connected):
        self.servo_combo.setEnabled(connected)
        self.ping_btn.setEnabled(connected)
        self.position_spin.setEnabled(connected)

    def ping_servo(self):
        servo_id = int(self.servo_combo.currentText())
        if self.controller.ping_servo(servo_id):
            self.status_label.setText(f'Status: Servo {servo_id} responded successfully')
        else:
            self.status_label.setText(f'Status: Servo {servo_id} did not respond')

    def adjust_position(self, delta):
        new_pos = self.position_spin.value() + delta
        self.position_spin.setValue(max(0, min(1023, new_pos)))
        self.set_position(self.position_spin.value())

    def set_position(self, position):
        if self.controller is None:
            return

        servo_id = int(self.servo_combo.currentText())
        if self.controller.set_servo_position(servo_id, position):
            self.status_label.setText(f'Status: Servo {servo_id} set to position {position}')
        else:
            self.status_label.setText(f'Status: Failed to set servo {servo_id} position')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ServoControlGUI()
    window.show()
    sys.exit(app.exec_())