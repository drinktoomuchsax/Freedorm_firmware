import sys
import serial
import serial.tools.list_ports
import re
import threading
from PyQt5 import QtWidgets, QtCore
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import time
import numpy as np

class RSSIMonitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.window_size = 8  # Moving average window size
        self.rssi_values = []  # Raw RSSI values list
        self.smoothed_rssi_values = []  # Kalman-filtered RSSI values
        self.timestamps = []  # To store timestamps for each RSSI value
        self.rssi_slope = []  # To store RSSI slope values
        self.rssi_trend = []  # To store RSSI trend values
        self.device_id = 0  # Default device ID
        self.init_ui()

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
        
        # Add controls layout for display elements
        controls_layout = QtWidgets.QHBoxLayout()
        layout.addLayout(controls_layout)
        
        # Checkboxes for controlling display elements
        self.chk_raw_rssi = QtWidgets.QCheckBox("Show Raw RSSI")
        self.chk_raw_rssi.setChecked(True)
        controls_layout.addWidget(self.chk_raw_rssi)
        
        self.chk_kalman_rssi = QtWidgets.QCheckBox("Show Kalman Filtered RSSI")
        self.chk_kalman_rssi.setChecked(True)
        controls_layout.addWidget(self.chk_kalman_rssi)
        
        self.chk_moving_avg = QtWidgets.QCheckBox("Show Moving Average")
        self.chk_moving_avg.setChecked(False)
        controls_layout.addWidget(self.chk_moving_avg)
        
        self.chk_slope = QtWidgets.QCheckBox("Show Slope")
        self.chk_slope.setChecked(False)
        controls_layout.addWidget(self.chk_slope)
        
        self.chk_trend = QtWidgets.QCheckBox("Show Trend")
        self.chk_trend.setChecked(False)
        controls_layout.addWidget(self.chk_trend)
        
        self.chk_quality = QtWidgets.QCheckBox("Show Signal Quality")
        self.chk_quality.setChecked(False)
        controls_layout.addWidget(self.chk_quality)

        # Plot area
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Real-Time RSSI Data")
        self.ax.set_xlabel("Time (s)")
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
            self.smoothed_rssi_values = []
            self.timestamps = []
            self.rssi_slope = []
            self.rssi_trend = []

            # Start thread for reading RSSI data
            self.thread = threading.Thread(target=self.read_rssi)
            self.thread.daemon = True
            self.thread.start()

            # Start timer for plot updates
            self.timer.start(100)  # Refresh every 100ms

        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to open serial port: {e}")

    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        # Stop the timer
        self.timer.stop()

    def read_rssi(self):
        """Read RSSI data, slope, and trend from serial"""
        
        # Updated regular expressions
        raw_rssi_pattern = re.compile(r"RSSI of the remote device (\d+): (-?\d+)")  # Match device ID and raw RSSI
        trend_pattern = re.compile(r"RSSI trend: (-?\d+)")  # Match trend value
        smoothed_pattern = re.compile(r"smoothed RSSI of the remote device (\d+): (-?\d+)")  # Match smoothed RSSI

        # Variables to store temporarily before adding to arrays
        current_device_id = None
        current_raw_rssi = None
        current_smoothed_rssi = None
        current_trend = None
        
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    # Read and decode serial data
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Line: {line}")  # Debug output

                    # Match raw RSSI
                    raw_rssi_match = raw_rssi_pattern.search(line)
                    if raw_rssi_match:
                        device_id = int(raw_rssi_match.group(1))
                        rssi_value = int(raw_rssi_match.group(2))
                        current_device_id = device_id
                        current_raw_rssi = rssi_value
                        print(f"Parsed Raw RSSI: Device {device_id}, Value {rssi_value}")
                        
                        # Check for trend in the same line
                        trend_match = trend_pattern.search(line)
                        if trend_match:
                            current_trend = int(trend_match.group(1))
                            print(f"Parsed trend: {current_trend}")
                    
                    # Match smoothed RSSI
                    smoothed_match = smoothed_pattern.search(line)
                    if smoothed_match:
                        device_id = int(smoothed_match.group(1))
                        smoothed_value = int(smoothed_match.group(2))
                        # Only update if same device or no device set yet
                        if current_device_id is None or current_device_id == device_id:
                            current_device_id = device_id
                            current_smoothed_rssi = smoothed_value
                            print(f"Parsed smoothed RSSI: Device {device_id}, Value {smoothed_value}")
                    
                    # If we have both values, add them as a pair
                    if current_raw_rssi is not None and current_smoothed_rssi is not None:
                        # Add to data arrays
                        self.device_id = current_device_id
                        self.rssi_values.append(current_raw_rssi)
                        self.smoothed_rssi_values.append(current_smoothed_rssi)
                        self.timestamps.append(time.time())
                        
                        if current_trend is not None:
                            self.rssi_trend.append(current_trend)
                        
                        # Calculate slope if enough data
                        if len(self.rssi_values) >= 3:
                            recent_values = self.rssi_values[-3:]
                            x = np.array(range(len(recent_values)))
                            slope = np.polyfit(x, recent_values, 1)[0]
                            self.rssi_slope.append(slope)
                            print(f"Calculated slope: {slope}")
                        
                        # Reset temporary variables
                        current_raw_rssi = None
                        current_smoothed_rssi = None
                        current_trend = None
                        
                    # If we have data from different logging messages that belong together
                    elif (current_raw_rssi is not None or current_smoothed_rssi is not None) and \
                        "I (" in line and "FREEDORM_BLE_GAP" in line:
                        # This appears to be a new logging message, so save what we have
                        if current_raw_rssi is not None:
                            self.device_id = current_device_id
                            self.rssi_values.append(current_raw_rssi)
                            # Use the last known smoothed value or a placeholder
                            if len(self.smoothed_rssi_values) > 0:
                                # Repeat the last smoothed value
                                self.smoothed_rssi_values.append(self.smoothed_rssi_values[-1])
                            else:
                                # First data point, just use raw as initial smoothed
                                self.smoothed_rssi_values.append(current_raw_rssi)
                            self.timestamps.append(time.time())
                            
                            if current_trend is not None:
                                self.rssi_trend.append(current_trend)
                        
                        elif current_smoothed_rssi is not None:
                            self.device_id = current_device_id
                            # Use the last known raw value or a placeholder
                            if len(self.rssi_values) > 0:
                                # Repeat the last raw value
                                self.rssi_values.append(self.rssi_values[-1])
                            else:
                                # First data point, just use smoothed as initial raw
                                self.rssi_values.append(current_smoothed_rssi)
                            self.smoothed_rssi_values.append(current_smoothed_rssi)
                            self.timestamps.append(time.time())
                        
                        # Reset temporary variables
                        current_raw_rssi = None
                        current_smoothed_rssi = None
                        current_trend = None

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
        self.ax.set_title(f"Real-time RSSI Data - Device {self.device_id}")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("RSSI (dBm)")
        self.ax.grid()

        # Get elapsed time for the x-axis
        elapsed_time = self.get_elapsed_time()

        # Fix length matching issues for raw RSSI
        min_len = min(len(elapsed_time), len(self.rssi_values))
        plot_time = elapsed_time[:min_len]
        plot_rssi = self.rssi_values[:min_len]

        # Scaling factors
        slope_scale_factor = 2  # Scale slope for better visualization
        trend_scale_factor = -5  # Scale trend for better visualization

        # Plot raw RSSI data (if checkbox is checked)
        if self.chk_raw_rssi.isChecked():
            self.ax.plot(plot_time, plot_rssi, label="Raw RSSI", marker='o', linewidth=1, color='blue')

        # Plot Kalman filtered RSSI (if checkbox is checked and data available)
        if self.chk_kalman_rssi.isChecked() and self.smoothed_rssi_values:
            smoothed_len = min(len(plot_time), len(self.smoothed_rssi_values))
            smoothed_time = plot_time[:smoothed_len]
            plot_smoothed = self.smoothed_rssi_values[:smoothed_len]
            self.ax.plot(smoothed_time, plot_smoothed, label="Kalman Filtered RSSI", 
                       marker='s', linewidth=1.5, color='purple')

        # Plot moving average (if checkbox is checked)
        if self.chk_moving_avg.isChecked() and len(self.rssi_values) >= self.window_size:
            smoothed_values = self.calculate_moving_average(plot_rssi, self.window_size)
            smoothed_time = plot_time[self.window_size-1:]
            self.ax.plot(smoothed_time, smoothed_values,
                       label=f"Moving Average (Window: {self.window_size})", color='red', linewidth=2)

        # Plot RSSI slope (scaled) (if checkbox is checked)
        if self.chk_slope.isChecked() and len(self.rssi_slope) > 0:
            slope_time = plot_time[-len(self.rssi_slope):]
            plot_slope = self.rssi_slope[:len(slope_time)]
            scaled_slope = [value * slope_scale_factor for value in plot_slope]
            self.ax.plot(slope_time, scaled_slope, label=f"RSSI Slope (x{slope_scale_factor})", 
                       color='orange', linestyle='--', linewidth=1.5)

        # Plot RSSI trend (scaled) (if checkbox is checked)
        if self.chk_trend.isChecked() and len(self.rssi_trend) > 0:
            trend_time = plot_time[-len(self.rssi_trend):]
            plot_trend = self.rssi_trend[:len(trend_time)]
            scaled_trend = [value * trend_scale_factor for value in plot_trend]
            self.ax.plot(trend_time, scaled_trend, label=f"RSSI Trend (x{trend_scale_factor})", 
                       color='green', linestyle='-.', linewidth=1.5)

        # Add signal quality indicator (if checkbox is checked)
        if self.chk_quality.isChecked() and len(plot_rssi) > 0:
            latest_rssi = plot_rssi[-1]
            quality_text = self.get_signal_quality(latest_rssi)
            self.ax.text(0.02, 0.05, f"Signal Quality: {quality_text}", transform=self.ax.transAxes, 
                      bbox=dict(facecolor='white', alpha=0.7))

        self.ax.legend(loc='upper right')
        self.canvas.draw()

    def calculate_moving_average(self, data, window_size):
        """Calculate moving average"""
        return [sum(data[i:i + window_size]) / window_size for i in range(len(data) - window_size + 1)]

    def get_signal_quality(self, rssi):
        """Evaluate signal quality based on RSSI value"""
        if rssi >= -50:
            return "Excellent"
        elif rssi >= -65:
            return "Very Good"
        elif rssi >= -75:
            return "Good"
        elif rssi >= -85:
            return "Fair"
        elif rssi >= -95:
            return "Poor"
        else:
            return "Very Poor"

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
