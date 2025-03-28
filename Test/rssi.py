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
        self.rssi_speed = []  # To store RSSI speed values
        self.speed_timestamps = []  # Timestamps for speed values
        self.rssi_trend = []  # To store RSSI trend values
        self.trend_timestamps = []  # Timestamps for trend values
        self.distance_values = []  # To store distance values
        self.is_approaching = []  # To store approaching status values
        self.approaching_timestamps = []  # Timestamps for approaching status
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
        self.chk_kalman_rssi.setChecked(False)
        controls_layout.addWidget(self.chk_kalman_rssi)
        
        self.chk_moving_avg = QtWidgets.QCheckBox("Show Moving Average")
        self.chk_moving_avg.setChecked(False)
        controls_layout.addWidget(self.chk_moving_avg)
        
        self.chk_speed = QtWidgets.QCheckBox("Show Speed")
        self.chk_speed.setChecked(False)
        controls_layout.addWidget(self.chk_speed)
        
        self.chk_trend = QtWidgets.QCheckBox("Show Trend")
        self.chk_trend.setChecked(False)
        controls_layout.addWidget(self.chk_trend)
        
        self.chk_quality = QtWidgets.QCheckBox("Show Signal Quality")
        self.chk_quality.setChecked(False)
        controls_layout.addWidget(self.chk_quality)
        
        # Add checkbox for distance display
        self.chk_distance = QtWidgets.QCheckBox("Show Distance")
        self.chk_distance.setChecked(True)
        controls_layout.addWidget(self.chk_distance)
        
        # Add checkbox for approaching status
        self.chk_approaching = QtWidgets.QCheckBox("Show Approaching")
        self.chk_approaching.setChecked(True)
        controls_layout.addWidget(self.chk_approaching)

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
            
            # Reset all data arrays
            self.rssi_values = []
            self.smoothed_rssi_values = []
            self.timestamps = []
            self.rssi_speed = []
            self.speed_timestamps = []  # Reset speed timestamps
            self.rssi_trend = []
            self.trend_timestamps = []  # Reset trend timestamps
            self.distance_values = []
            self.is_approaching = []  # Reset approaching status
            self.approaching_timestamps = []  # Reset approaching timestamps

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
        """Read RSSI data, speed, trend, and distance from serial"""
        
        # Updated regular expressions
        raw_rssi_pattern = re.compile(r"RSSI of the remote device (\d+): (-?\d+)")  # Match device ID and raw RSSI
        trend_pattern = re.compile(r"RSSI trend: (-?\d+)")  # Match trend value
        smoothed_pattern = re.compile(r"smoothed RSSI of the remote device (\d+): (-?\d+)")  # Match smoothed RSSI
        distance_pattern = re.compile(r"distance of the remote device (\d+): ([\d.]+)")  # Match device ID and distance
        approaching_pattern = re.compile(r"Is approaching: (\d+)")  # Match approaching status

        # Variables to store temporarily before adding to arrays
        current_device_id = None
        current_raw_rssi = None
        current_smoothed_rssi = None
        current_trend = None
        current_distance = None
        current_approaching = None
        current_timestamp = None
        
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    # Read and decode serial data
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Line: {line}")  # Debug output
                    
                    # Generate timestamp for this line
                    current_timestamp = time.time()

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
                    
                    # Match distance
                    distance_match = distance_pattern.search(line)
                    if distance_match:
                        device_id = int(distance_match.group(1))
                        distance_value = float(distance_match.group(2))
                        # Only update if same device or no device set yet
                        if current_device_id is None or current_device_id == device_id:
                            current_device_id = device_id
                            current_distance = distance_value
                            print(f"Parsed distance: Device {device_id}, Value {distance_value}")
                    
                    # Match approaching status
                    approaching_match = approaching_pattern.search(line)
                    if approaching_match:
                        approaching_value = int(approaching_match.group(1))
                        current_approaching = approaching_value
                        print(f"Parsed approaching status: {approaching_value}")
                    
                    # If we have both values, add them as a pair
                    if current_raw_rssi is not None and current_smoothed_rssi is not None:
                        # Add to data arrays
                        self.device_id = current_device_id
                        self.rssi_values.append(current_raw_rssi)
                        self.smoothed_rssi_values.append(current_smoothed_rssi)
                        self.timestamps.append(current_timestamp)
                        
                        if current_trend is not None:
                            self.rssi_trend.append(current_trend)
                            self.trend_timestamps.append(current_timestamp)  # Store timestamp for trend
                        
                        if current_distance is not None:
                            self.distance_values.append(current_distance)
                        elif len(self.distance_values) > 0:
                            # Use previous distance value if available
                            self.distance_values.append(self.distance_values[-1])
                        
                        if current_approaching is not None:
                            self.is_approaching.append(current_approaching)
                            self.approaching_timestamps.append(current_timestamp)
                        elif len(self.is_approaching) > 0:
                            # Use previous approaching value if available
                            self.is_approaching.append(self.is_approaching[-1])
                            self.approaching_timestamps.append(current_timestamp)
                        
                        # Calculate speed if enough data
                        if len(self.rssi_values) >= 3:
                            recent_values = self.rssi_values[-3:]
                            x = np.array(range(len(recent_values)))
                            speed = np.polyfit(x, recent_values, 1)[0]
                            self.rssi_speed.append(speed)
                            self.speed_timestamps.append(current_timestamp)  # Store timestamp for speed
                            print(f"Calculated speed: {speed}")
                        
                        # Reset temporary variables
                        current_raw_rssi = None
                        current_smoothed_rssi = None
                        current_trend = None
                        current_distance = None
                        current_approaching = None
                        
                    # If we have data from different logging messages that belong together
                    elif (current_raw_rssi is not None or current_smoothed_rssi is not None or 
                          current_distance is not None or current_approaching is not None) and \
                        "I (" in line and ("FREEDORM_BLE_GAP" in line or "BLE_GAP_TAG" in line):
                        # This appears to be a new logging message, so save what we have
                        if (current_raw_rssi is not None or current_smoothed_rssi is not None or 
                            current_distance is not None or current_approaching is not None):
                            self.device_id = current_device_id
                            
                            # Handle raw RSSI
                            if current_raw_rssi is not None:
                                self.rssi_values.append(current_raw_rssi)
                            elif len(self.rssi_values) > 0:
                                # Repeat the last raw value
                                self.rssi_values.append(self.rssi_values[-1])
                            
                            # Handle smoothed RSSI
                            if current_smoothed_rssi is not None:
                                self.smoothed_rssi_values.append(current_smoothed_rssi)
                            elif len(self.smoothed_rssi_values) > 0:
                                # Repeat the last smoothed value
                                self.smoothed_rssi_values.append(self.smoothed_rssi_values[-1])
                            elif current_raw_rssi is not None:
                                # First data point, just use raw as initial smoothed
                                self.smoothed_rssi_values.append(current_raw_rssi)
                            
                            # Handle distance
                            if current_distance is not None:
                                self.distance_values.append(current_distance)
                            elif len(self.distance_values) > 0:
                                # Repeat the last distance value
                                self.distance_values.append(self.distance_values[-1])
                            
                            # Handle approaching status
                            if current_approaching is not None:
                                self.is_approaching.append(current_approaching)
                                self.approaching_timestamps.append(current_timestamp)
                            elif len(self.is_approaching) > 0:
                                # Repeat the last approaching value
                                self.is_approaching.append(self.is_approaching[-1])
                                self.approaching_timestamps.append(current_timestamp)
                            
                            # Add timestamp
                            self.timestamps.append(current_timestamp)
                            
                            # Handle trend with its timestamp
                            if current_trend is not None:
                                self.rssi_trend.append(current_trend)
                                self.trend_timestamps.append(current_timestamp)
                            
                            # Calculate speed with its timestamp if enough data
                            if len(self.rssi_values) >= 3 and len(self.rssi_values) > len(self.rssi_speed):
                                recent_values = self.rssi_values[-3:]
                                x = np.array(range(len(recent_values)))
                                speed = np.polyfit(x, recent_values, 1)[0]
                                self.rssi_speed.append(speed)
                                self.speed_timestamps.append(current_timestamp)
                        
                        # Reset temporary variables
                        current_raw_rssi = None
                        current_smoothed_rssi = None
                        current_trend = None
                        current_distance = None
                        current_approaching = None

            except Exception as e:
                print(f"Error reading from serial: {e}")
                import traceback
                traceback.print_exc()
                self.running = False
                break

    def get_elapsed_time(self, timestamps):
        """Convert timestamps to elapsed time in seconds"""
        if not timestamps:
            return []
        start_time = self.timestamps[0]  # Always use the first RSSI timestamp as reference
        return [timestamp - start_time for timestamp in timestamps]

    def update_window_size(self, value):
        """Update the moving average window size"""
        self.window_size = value
        print(f"Updated window size: {self.window_size}")

    def update_plot(self):
        """Update the plot in real-time"""
        if not self.rssi_values:
            print("No RSSI data available to plot.")
            return

        # Clear the entire figure instead of just the primary axis
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)
        
        self.ax.set_title(f"Real-time RSSI Data - Device {self.device_id}")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("RSSI (dBm)")
        self.ax.grid()

        # Get elapsed time for the x-axis of each data series
        elapsed_time = self.get_elapsed_time(self.timestamps)
        speed_elapsed_time = self.get_elapsed_time(self.speed_timestamps)
        trend_elapsed_time = self.get_elapsed_time(self.trend_timestamps)
        approaching_elapsed_time = self.get_elapsed_time(self.approaching_timestamps)

        # Fix length matching issues for raw RSSI
        min_len = min(len(elapsed_time), len(self.rssi_values))
        plot_time = elapsed_time[:min_len]
        plot_rssi = self.rssi_values[:min_len]

        # Scaling factors
        speed_scale_factor = 1  # Scale speed for better visualization
        trend_scale_factor = 1  # Scale trend for better visualization
        
        # Text positions for left upper corner (y-values)
        text_positions = [0.95, 0.90, 0.85, 0.80, 0.75, 0.70, 0.65, 0.60, 0.55, 0.50]
        pos_index = 0

        # Plot raw RSSI data (if checkbox is checked)
        if self.chk_raw_rssi.isChecked():
            self.ax.plot(plot_time, plot_rssi, label="Raw RSSI", marker='o', linewidth=1, color='blue')
            
            # Show latest raw RSSI value
            if len(plot_rssi) > 0:
                latest_rssi = plot_rssi[-1]
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Latest Raw RSSI: {latest_rssi} dBm", 
                    transform=self.ax.transAxes, color='blue',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1

        # Plot Kalman filtered RSSI (if checkbox is checked and data available)
        if self.chk_kalman_rssi.isChecked() and self.smoothed_rssi_values:
            smoothed_len = min(len(plot_time), len(self.smoothed_rssi_values))
            smoothed_time = plot_time[:smoothed_len]
            plot_smoothed = self.smoothed_rssi_values[:smoothed_len]
            self.ax.plot(smoothed_time, plot_smoothed, label="Kalman Filtered RSSI", 
                    marker='s', linewidth=1.5, color='purple')
            
            # Show latest Kalman filtered RSSI value
            if len(plot_smoothed) > 0:
                latest_smoothed = plot_smoothed[-1]
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Latest Kalman RSSI: {latest_smoothed} dBm", 
                    transform=self.ax.transAxes, color='purple',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1

        # Plot moving average (if checkbox is checked)
        if self.chk_moving_avg.isChecked() and len(self.rssi_values) >= self.window_size:
            smoothed_values = self.calculate_moving_average(plot_rssi, self.window_size)
            smoothed_time = plot_time[self.window_size-1:]
            self.ax.plot(smoothed_time, smoothed_values,
                    label=f"Moving Average (Window: {self.window_size})", color='red', linewidth=2)
            
            # Show latest moving average value
            if len(smoothed_values) > 0:
                latest_ma = smoothed_values[-1]
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Latest Moving Avg: {latest_ma:.2f} dBm", 
                    transform=self.ax.transAxes, color='red',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1

        # Plot RSSI speed with correct timestamps (if checkbox is checked)
        if self.chk_speed.isChecked() and len(self.rssi_speed) > 0:
            min_speed_len = min(len(speed_elapsed_time), len(self.rssi_speed))
            plot_speed_time = speed_elapsed_time[:min_speed_len]
            plot_speed = self.rssi_speed[:min_speed_len]
            scaled_speed = [value * speed_scale_factor for value in plot_speed]
            self.ax.plot(plot_speed_time, scaled_speed, label=f"RSSI Speed (x{speed_scale_factor})", 
                    color='orange', linestyle='--', linewidth=1.5)
            
            # Show latest speed value
            if len(plot_speed) > 0:
                latest_speed = plot_speed[-1]
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Latest Speed: {latest_speed:.2f}", 
                    transform=self.ax.transAxes, color='orange',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1

        # Plot RSSI trend with correct timestamps (if checkbox is checked)
        if self.chk_trend.isChecked() and len(self.rssi_trend) > 0:
            min_trend_len = min(len(trend_elapsed_time), len(self.rssi_trend))
            plot_trend_time = trend_elapsed_time[:min_trend_len]
            plot_trend = self.rssi_trend[:min_trend_len]
            scaled_trend = [value * trend_scale_factor for value in plot_trend]
            self.ax.plot(plot_trend_time, scaled_trend, label=f"RSSI Trend (x{trend_scale_factor})", 
                    color='green', linestyle='-.', linewidth=1.5)
            
            # Show latest trend value
            if len(plot_trend) > 0:
                latest_trend = plot_trend[-1]
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Latest Trend: {latest_trend}", 
                    transform=self.ax.transAxes, color='green',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1
        
        # Plot approaching status (if checkbox is checked)
        if self.chk_approaching.isChecked() and len(self.is_approaching) > 0:
            min_approaching_len = min(len(approaching_elapsed_time), len(self.is_approaching))
            plot_approaching_time = approaching_elapsed_time[:min_approaching_len]
            plot_approaching = self.is_approaching[:min_approaching_len]
            
            # Use a separate axis for approaching status (0/1 values)
            ax3 = self.ax.twinx()
            ax3.spines['right'].set_position(('outward', 60))  # Move this axis outward
            ax3.set_ylabel('Is Approaching', color='magenta')
            ax3.tick_params(axis='y', labelcolor='magenta')
            ax3.set_ylim(-0.1, 1.1)  # Only 0 and 1 values with some padding
            ax3.set_yticks([0, 1])
            ax3.set_yticklabels(['No', 'Yes'])
            
            # Plot with step function instead of stem to avoid oscillation
            ax3.step(plot_approaching_time, plot_approaching, where='post', 
                    color='magenta', linewidth=2, label="Is Approaching")
            
            # Add small markers at data points for clarity
            ax3.plot(plot_approaching_time, plot_approaching, 'mo', markersize=4)
            
            # Debug print to verify data
            # print(f"Approaching data points: {plot_approaching}")
            
            # Show latest approaching status value
            if len(plot_approaching) > 0:
                latest_approaching = plot_approaching[-1]
                approaching_text = "Yes" if latest_approaching == 1 else "No"
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Is Approaching: {approaching_text}", 
                    transform=self.ax.transAxes, color='magenta',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1

        # Plot distance data (if checkbox is checked and data available)
        if self.chk_distance.isChecked() and self.distance_values:
            # Create a twin y-axis for distance
            ax2 = self.ax.twinx()
            ax2.set_ylabel('Distance (m)', color='brown')
            ax2.tick_params(axis='y', labelcolor='brown')
            
            # If approaching is also shown, move this axis to avoid overlap
            if self.chk_approaching.isChecked() and len(self.is_approaching) > 0:
                ax2.spines['right'].set_position(('outward', 30))
            
            distance_len = min(len(plot_time), len(self.distance_values))
            distance_time = plot_time[:distance_len]
            plot_distance = self.distance_values[:distance_len]
            
            # Use a distinct line style and marker for distance
            ax2.plot(distance_time, plot_distance, label="Distance", 
                marker='d', linewidth=2, color='brown', linestyle='-')
            
            # Show latest distance value
            if len(plot_distance) > 0:
                latest_distance = plot_distance[-1]
                # Position the text in the upper left corner
                self.ax.text(0.02, text_positions[pos_index], 
                    f"Latest Distance: {latest_distance:.2f}m", 
                    transform=self.ax.transAxes, color='brown',
                    bbox=dict(facecolor='white', alpha=0.7))
                pos_index += 1

        # Add signal quality indicator (if checkbox is checked)
        if self.chk_quality.isChecked() and len(plot_rssi) > 0:
            latest_rssi = plot_rssi[-1]
            quality_text = self.get_signal_quality(latest_rssi)
            self.ax.text(0.02, text_positions[pos_index], 
                    f"Signal Quality: {quality_text}", 
                    transform=self.ax.transAxes, color='black',
                    bbox=dict(facecolor='white', alpha=0.7))
            pos_index += 1

        # Create legend with all elements
        handles = []
        labels = []
        
        # Get handles and labels from main axis
        h1, l1 = self.ax.get_legend_handles_labels()
        handles.extend(h1)
        labels.extend(l1)
        
        # Get handles and labels from distance axis if it exists
        if self.chk_distance.isChecked() and self.distance_values:
            h2, l2 = ax2.get_legend_handles_labels()
            handles.extend(h2)
            labels.extend(l2)
        
        # Get handles and labels from approaching axis if it exists
        if self.chk_approaching.isChecked() and len(self.is_approaching) > 0:
            h3, l3 = ax3.get_legend_handles_labels()
            handles.extend(h3)
            labels.extend(l3)
        
        # Create combined legend in lower right corner
        if handles:
            self.ax.legend(handles, labels, loc='lower right')
            
        # Draw the canvas with the updated figure
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
