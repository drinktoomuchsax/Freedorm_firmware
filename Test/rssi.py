import sys
import serial
import serial.tools.list_ports
import re
import threading
from PyQt5 import QtWidgets, QtCore
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import time  # Ensure you import the time module


class RSSIMonitor(QtWidgets.QMainWindow):
    def __init__(self):
        
        super().__init__()
        self.window_size = 8  # Initialize the moving average window size
        self.rssi_values = []  # Initialize the RSSI values list
        self.init_ui()
        self.timestamps = []  # To store the timestamps for each RSSI value
        self.rssi_slope = []  # To store RSSI slope values
        self.rssi_trend = []  # To store RSSI trend values



    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("RSSI Monitor")
        self.setGeometry(100, 100, 800, 600)

        # Main widget
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        # Layouts
        layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(layout)

        # Serial port selection
        top_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(top_layout)

        self.port_label = QtWidgets.QLabel("Select Port:")
        top_layout.addWidget(self.port_label)

        self.port_combobox = QtWidgets.QComboBox()
        self.port_combobox.addItems(self.get_serial_ports())
        self.port_combobox.setPlaceholderText("Choose a port")
        top_layout.addWidget(self.port_combobox)

        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self.start_monitoring)
        top_layout.addWidget(self.start_button)

        self.stop_button = QtWidgets.QPushButton("Stop")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_monitoring)
        top_layout.addWidget(self.stop_button)

        # Moving average window size adjustment
        self.window_label = QtWidgets.QLabel("Window Size:")
        top_layout.addWidget(self.window_label)

        self.window_spinbox = QtWidgets.QSpinBox()
        self.window_spinbox.setRange(1, 100)  # Window size range
        self.window_spinbox.setValue(self.window_size)
        self.window_spinbox.valueChanged.connect(self.update_window_size)
        top_layout.addWidget(self.window_spinbox)

        # Plot area
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Real-Time RSSI Data")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("RSSI (dBm)")
        self.ax.grid()

        # Timer for plot updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)

    def get_serial_ports(self):
        """Get available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def start_monitoring(self):
        """Start monitoring RSSI data"""
        port = self.port_combobox.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, "Error", "Please select a valid serial port")
            return

        try:
            self.serial_port = serial.Serial(port, baudrate=115200, timeout=1)
            self.running = True
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.rssi_values = []

            # Start thread for reading RSSI data
            self.thread = threading.Thread(target=self.read_rssi)
            self.thread.daemon = True
            self.thread.start()

            # Start timer for plot updates
            self.timer.start(20)  # Refresh every 500ms

        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open serial port: {e}")

    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        # Stop the timer
        self.timer.stop()

    def read_rssi(self):
        """Read RSSI data, slope, and trend from serial"""
        rssi_pattern = re.compile(r"rssi\s(-\d+)")
        slope_pattern = re.compile(r"slope:\s([+-]?\d*\.?\d+)")
        trend_pattern = re.compile(r"trend:\s(\d+)")

        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    # Read and decode serial data
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Line: {line}")  # Debug

                    # Match and store RSSI
                    rssi_match = rssi_pattern.search(line)
                    if rssi_match:
                        rssi_value = int(rssi_match.group(1))
                        self.rssi_values.append(rssi_value)
                        self.timestamps.append(time.time())  # Record timestamp
                        print(f"Parsed RSSI: {rssi_value}")  # Debug

                    # Match and store RSSI slope
                    slope_match = slope_pattern.search(line)
                    if slope_match:
                        slope_value = float(slope_match.group(1))
                        self.rssi_slope.append(slope_value)
                        print(f"Parsed Slope: {slope_value}")  # Debug

                    # Match and store RSSI trend
                    trend_match = trend_pattern.search(line)
                    if trend_match:
                        trend_value = int(trend_match.group(1))
                        self.rssi_trend.append(trend_value)
                        print(f"Parsed Trend: {trend_value}")  # Debug
            except Exception as e:
                print(f"Error reading from serial: {e}")
                self.running = False
                break


    def get_elapsed_time(self):
        """Convert timestamps to elapsed time in seconds"""
        if not self.timestamps:
            return []
        start_time = self.timestamps[0]
        return [timestamp - start_time for timestamp in self.timestamps]

    def update_window_size(self, value):
        """Update the moving average window size"""
        self.window_size = value
        print(f"Updated window size: {self.window_size}")


    def update_plot(self):
        """Update the plot in real-time"""
        if not self.rssi_values:
            print("No RSSI data available to plot.")
            return

        self.ax.clear()
        self.ax.set_title("Real-Time RSSI Data")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Value")
        self.ax.grid()

        # Get elapsed time for the x-axis
        elapsed_time = self.get_elapsed_time()

        # 修正 elapsed_time 和 rssi_values 的长度
        if len(elapsed_time) > len(self.rssi_values):
            elapsed_time = elapsed_time[:len(self.rssi_values)]
        elif len(elapsed_time) < len(self.rssi_values):
            self.rssi_values = self.rssi_values[:len(elapsed_time)]

        # Scaling factors for slope and trend
        slope_scale_factor = 1  # Scale slope to make it more visible
        trend_scale_factor = -10   # Scale trend to make it more visible

        # Plot raw RSSI data
        self.ax.plot(elapsed_time, self.rssi_values, label="Raw RSSI", marker='o', linewidth=1)

        # Plot RSSI slope (scaled)
        if len(self.rssi_slope) > 0:
            slope_time = elapsed_time[-len(self.rssi_slope):]
            if len(slope_time) > len(self.rssi_slope):
                slope_time = slope_time[:len(self.rssi_slope)]
            elif len(self.rssi_slope) > len(slope_time):
                self.rssi_slope = self.rssi_slope[:len(slope_time)]

            scaled_slope = [value * slope_scale_factor for value in self.rssi_slope]
            self.ax.plot(slope_time, scaled_slope, label=f"RSSI Slope (x{slope_scale_factor})", color='orange', linestyle='--', linewidth=1.5)

        # Plot RSSI trend (scaled)
        if len(self.rssi_trend) > 0:
            trend_time = elapsed_time[-len(self.rssi_trend):]
            if len(trend_time) > len(self.rssi_trend):
                trend_time = trend_time[:len(self.rssi_trend)]
            elif len(self.rssi_trend) > len(trend_time):
                self.rssi_trend = self.rssi_trend[:len(trend_time)]

            scaled_trend = [value * trend_scale_factor for value in self.rssi_trend]
            self.ax.plot(trend_time, scaled_trend, label=f"RSSI Trend (x{trend_scale_factor})", color='green', linestyle='-.', linewidth=1.5)

        # Plot moving average for RSSI
        if len(self.rssi_values) >= self.window_size:
            smoothed_values = self.calculate_moving_average(self.rssi_values, self.window_size)
            smoothed_time = elapsed_time[-len(smoothed_values):]  # Align time with smoothed values
            self.ax.plot(smoothed_time, smoothed_values,
                        label=f"Moving Average (Window Size: {self.window_size})", color='red', linewidth=2)

        self.ax.legend()
        self.canvas.draw()



    def calculate_moving_average(self, data, window_size):
        """Calculate moving average"""
        return [sum(data[i:i + window_size]) / window_size for i in range(len(data) - window_size + 1)]

    def update_window_size(self, value):
        """Update the moving average window size"""
        self.window_size = value

    def closeEvent(self, event):
        """Clean up resources when closing the application"""
        self.stop_monitoring()
        event.accept()




def main():
    app = QtWidgets.QApplication(sys.argv)
    main_window = RSSIMonitor()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
