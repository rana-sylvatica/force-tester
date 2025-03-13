import sys
import time
import csv
import serial
import serial.tools.list_ports
import numpy as np
import pandas as pd
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QComboBox, QGridLayout, QStatusBar, QMessageBox, QFileDialog
)
import pyqtgraph as pg

# Constants
BAUD_RATE = 115200
INITIAL_GCODE = [
    "M92 Z320",    # Steps/mm for 4-start lead screw (10mm lead)
    "M203 Z50",    # Maximum Z speed (mm/s)
    "M201 Z500",   # Z acceleration (mm/s²)
    "M205 Z2.0",   # Z jerk/junction deviation (mm/s)
    "G92 Z0"       # Set current position as Z zero
]

class SerialReaderThread(QThread):
    """Thread for reading data from load cell"""
    data_received = pyqtSignal(float, float, float, float)  # timestamp, position, force, raw_encoder_position
    error_signal = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = False
        self.serial = None
        self.position = 0.0  # Current position of the system
        self.encoder_position = 1000.0  # Initial encoder position (starts at 1000mm)
        self.position_offset = 1000.0  # Offset to zero out the position
    
    def run(self):
        try:
            self.serial = serial.Serial(self.port, BAUD_RATE, timeout=1)
            self.running = True
            
            while self.running:
                if self.serial.in_waiting:
                    try:
                        line = self.serial.readline().decode('utf-8', errors='replace').strip()
                        # Parse the CSV data from load cell/encoder
                        # Format: timestamp,raw_value,pounds,position_mm
                        parts = line.split(',')
                        if len(parts) >= 4:
                            timestamp = float(parts[0])
                            force = float(parts[2])  # pounds value
                            self.encoder_position = float(parts[3])  # position in mm from encoder
                            # Calculate zeroed position
                            self.position = self.encoder_position - self.position_offset
                            self.data_received.emit(timestamp, self.position, force, self.encoder_position)
                    except (ValueError, IndexError) as e:
                        # Not a valid data line, ignore
                        pass
                    except Exception as e:
                        print(f"Error reading load cell/encoder: {str(e)}")
                else:
                    # Avoid 100% CPU usage
                    time.sleep(0.01)
                    
        except Exception as e:
            self.error_signal.emit(f"Load cell/encoder error: {str(e)}")
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()
    
    def zero_position(self):
        """Set current encoder position as zero"""
        self.position_offset = self.encoder_position
        return self.position_offset
    
    def stop(self):
        self.running = False
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass
        self.wait()


class ControllerThread(QThread):
    """Thread for sending commands to the motion controller"""
    position_updated = pyqtSignal(float)
    cycle_updated = pyqtSignal(int)
    test_complete = pyqtSignal()
    error_signal = pyqtSignal(str)
    
    def __init__(self, port, cycles=0, speed=10, distance=0):
        super().__init__()
        self.port = port
        self.cycles = cycles
        self.speed = speed
        self.distance = distance
        self.running = False
        self.test_active = False
        self.serial = None
        self.current_cycle = 0
    
    def initialize(self):
        """Initialize the serial connection without starting the thread"""
        try:
            self.serial = serial.Serial(self.port, BAUD_RATE, timeout=2)
            
            # Send initial G-code setup
            for command in INITIAL_GCODE:
                self.send_command(command)
                time.sleep(0.1)  # Brief pause between commands
            
            # Send connection test command and wait for response
            response = self.send_command("M115", wait_for_response=True)
            if not response:
                raise ValueError("Failed to connect to control board - no response.")
            elif "FIRMWARE" not in response and "Marlin" not in response:
                raise ValueError(f"Unexpected response from control board: {response[:50]}...")
                
            return True
                
        except Exception as e:
            self.error_signal.emit(f"Controller initialization error: {str(e)}")
            self.cleanup()
            return False
    
    def cleanup(self):
        """Clean up resources without stopping the thread"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass
            self.serial = None
    
    def run(self):
        """Thread main loop - only used during tests"""
        try:
            self.running = True
            
            # Run test cycles if test is active
            if self.test_active:
                if self.cycles == 0:
                    # Non-cyclic test: Move distance at speed, then stop
                    self.run_single_movement()
                else:
                    # Cyclic test: Repeat movement for specified cycles
                    self.run_cyclic_test()
            
        except Exception as e:
            self.error_signal.emit(f"Controller error: {str(e)}")
        finally:
            self.test_active = False
            self.running = False
            self.test_complete.emit()
    
    def start_test(self, cycles, speed, distance):
        """Start a test with the specified parameters"""
        if self.running or self.test_active:
            return False
            
        self.cycles = cycles
        self.speed = speed
        self.distance = distance
        self.current_cycle = 0
        self.cycle_updated.emit(self.current_cycle)
        self.test_active = True
        
        # Start the thread to run the test
        self.start()
        
        return True
    
    def stop_test(self):
        """Stop the current test"""
        if self.serial and self.serial.is_open and self.test_active:
            try:
                # Send emergency stop
                self.serial.write(b"M112\n")
                self.serial.flush()
            except:
                pass
        
        self.test_active = False
        self.running = False
        self.wait()  # Wait for thread to finish
    
    def jog(self, distance, speed=10):
        """Jog the axis by the specified distance at specified speed"""
        if not self.serial or not self.serial.is_open or self.test_active:
            return False
        
        try:
            # Set feedrate (speed in mm/min = mm/s * 60)
            feedrate = speed * 60
            
            # Send move command in relative mode
            # This allows jogging in either direction regardless of position
            self.send_command(f"G91")  # Set to relative positioning mode
            self.send_command(f"G1 Z{distance} F{feedrate}")
            # We'll stay in relative mode since we're not tracking position in the controller
            
            # Calculate approximate movement time
            movement_time = abs(distance) / speed
            # Wait for movement to finish (plus a small buffer)
            time.sleep(movement_time * 1.1)
            
            return True
            
        except Exception as e:
            self.error_signal.emit(f"Jog error: {str(e)}")
            return False
    
    def run_single_movement(self):
        """Run a single non-cyclic test movement"""
        try:
            # Set feedrate (speed in mm/min = mm/s * 60)
            feedrate = self.speed * 60
            
            # Move relative distance
            self.send_command(f"G91")  # Set to relative positioning mode
            self.send_command(f"G1 Z{self.distance} F{feedrate}")
            # We'll stay in relative mode to avoid absolute position limitations
            
            # Wait for movement to complete with a fixed delay based on speed and distance
            sleep_time = abs(self.distance) / self.speed * 1.2  # 20% extra time for acceleration/deceleration
            time.sleep(sleep_time)
                
        except Exception as e:
            self.error_signal.emit(f"Error during movement: {str(e)}")
    
    def run_cyclic_test(self):
        """Run a cyclic test with specified parameters"""
        try:
            # Set feedrate (speed in mm/min = mm/s * 60)
            feedrate = self.speed * 60
            
            # Set to relative positioning mode once at the start
            self.send_command(f"G91")
            
            self.current_cycle = 0
            self.cycle_updated.emit(self.current_cycle)
            
            while self.running and self.current_cycle < self.cycles:
                # Move forward
                self.send_command(f"G1 Z{self.distance} F{feedrate}")
                
                # Wait for movement to complete with a fixed delay
                forward_time = abs(self.distance) / self.speed * 1.2  # 20% extra time
                time.sleep(forward_time)
                
                # Move back to starting position
                self.send_command(f"G1 Z{-self.distance} F{feedrate}")
                
                # Wait for return movement to complete
                backward_time = abs(self.distance) / self.speed * 1.2  # 20% extra time
                time.sleep(backward_time)
                
                # Increment and report cycle count
                self.current_cycle += 1
                self.cycle_updated.emit(self.current_cycle)
                
            if self.running:
                self.test_complete.emit()
                
        except Exception as e:
            self.error_signal.emit(f"Error during cyclic test: {str(e)}")
    
    def send_command(self, command, wait_for_response=False):
        """Send a G-code command to the controller"""
        if not self.serial or not self.serial.is_open:
            return None
            
        # Add newline if needed
        if not command.endswith('\n'):
            command += '\n'
        
        # Clear any existing data in the buffer
        self.serial.reset_input_buffer()
            
        self.serial.write(command.encode('utf-8'))
        self.serial.flush()
        
        if wait_for_response:
            # Wait for response with timeout
            response = ""
            start_time = time.time()
            while time.time() - start_time < 5:  # 5-second timeout
                if self.serial.in_waiting:
                    try:
                        line = self.serial.readline().decode('utf-8', errors='replace').strip()
                        response += line + "\n"
                        # For M115 command, we're looking for the firmware info
                        if command.startswith("M115") and ("FIRMWARE" in line or "Marlin" in line):
                            # But still wait for the "ok"
                            continue
                        if line == "ok" or "ok" in line:
                            break
                    except Exception as e:
                        print(f"Error reading response: {str(e)}")
                        break
                time.sleep(0.05)  # slightly longer pause
            
            print(f"Command: {command.strip()}, Response: {response[:100]}...")
            return response
        
        return None
    
    def stop(self):
        """Stop the test and emergency stop the machine"""
        if self.serial and self.serial.is_open:
            try:
                # Send emergency stop
                self.serial.write(b"M112\n")
                self.serial.flush()
            except:
                pass
        
        self.running = False
        self.wait()


class ForceTestApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Single-Axis Force and Cycle Tester")
        self.setMinimumSize(800, 600)
        
        # Variables
        self.load_cell_thread = None
        self.controller_thread = None
        self.is_connected = False
        self.is_test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.data_buffer = []
        self.max_tension_force = 0
        self.max_compression_force = 0
        
        # Data for plotting
        self.times = []
        self.positions = []
        self.forces = []
        self.peak_tensions = []
        self.peak_compressions = []
        self.cycle_positions = []
        
        # Setup UI
        self.setup_ui()
        
        # Populate serial port dropdowns
        self.refresh_ports()
        
        # Setup status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # Update every second
    
    def setup_ui(self):
        # Main widget and layout
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        
        # Left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        
        # Serial port selection
        port_layout = QHBoxLayout()
        
        # Load Cell Port
        load_cell_layout = QVBoxLayout()
        load_cell_layout.addWidget(QLabel("Load Cell/Encoder Port:"))
        self.load_cell_port_combo = QComboBox()
        self.load_cell_port_combo.setMinimumWidth(150)
        load_cell_layout.addWidget(self.load_cell_port_combo)
        port_layout.addLayout(load_cell_layout)
        
        # Control Board Port
        control_board_layout = QVBoxLayout()
        control_board_layout.addWidget(QLabel("Control Board Port:"))
        self.control_board_port_combo = QComboBox()
        self.control_board_port_combo.setMinimumWidth(150)
        control_board_layout.addWidget(self.control_board_port_combo)
        port_layout.addLayout(control_board_layout)
        
        # Refresh ports button
        self.refresh_ports_btn = QPushButton("Refresh Ports")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_ports_btn)
        
        left_layout.addLayout(port_layout)
        
        # Connect/Disconnect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        left_layout.addWidget(self.connect_button)
        
        # Start/Stop buttons
        button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_test)
        self.start_button.setEnabled(False)  # Disabled until connected
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_test)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        left_layout.addLayout(button_layout)
        
        # Test parameters grid
        params_grid = QGridLayout()
        
        # Number of cycles
        params_grid.addWidget(QLabel("Number of cycles:"), 0, 0)
        self.cycles_input = QLineEdit("10")
        self.cycles_input.setPlaceholderText("Enter 0 for non-cyclic test")
        params_grid.addWidget(self.cycles_input, 0, 1)
        
        # X axis variable
        params_grid.addWidget(QLabel("X axis variable:"), 1, 0)
        self.x_axis_combo = QComboBox()
        self.x_axis_combo.addItems(["Time (seconds)", "Distance (mm)"])
        self.x_axis_combo.currentIndexChanged.connect(self.update_axis_options)
        params_grid.addWidget(self.x_axis_combo, 1, 1)
        
        # Y axis variable
        params_grid.addWidget(QLabel("Y axis variable:"), 2, 0)
        self.y_axis_combo = QComboBox()
        self.y_axis_combo.addItems(["Real-time force", "Peak tension force", "Peak compression force"])
        params_grid.addWidget(self.y_axis_combo, 2, 1)
        
        # Speed
        params_grid.addWidget(QLabel("Speed (mm/s):"), 3, 0)
        self.speed_input = QLineEdit("10")
        params_grid.addWidget(self.speed_input, 3, 1)
        
        # Distance
        params_grid.addWidget(QLabel("Distance (mm):"), 4, 0)
        self.distance_input = QLineEdit("50")
        params_grid.addWidget(self.distance_input, 4, 1)
        
        # Filename for CSV
        params_grid.addWidget(QLabel("Filename for CSV:"), 5, 0)
        self.csv_layout = QHBoxLayout()
        self.filename_input = QLineEdit("test_data.csv")
        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self.browse_csv_file)
        self.csv_layout.addWidget(self.filename_input)
        self.csv_layout.addWidget(self.browse_button)
        params_grid.addLayout(self.csv_layout, 5, 1)
        
        left_layout.addLayout(params_grid)
        
        # Add jog controls
        jog_layout = QVBoxLayout()
        jog_layout.addWidget(QLabel("Jog Controls:"))
        
        # Grid for jog buttons
        jog_grid = QGridLayout()
        
        # Up buttons
        self.jog_up_10_btn = QPushButton("Up 10mm")
        self.jog_up_10_btn.clicked.connect(lambda: self.jog_axis(10))
        self.jog_up_10_btn.setEnabled(False)
        jog_grid.addWidget(self.jog_up_10_btn, 0, 0)
        
        self.jog_up_1_btn = QPushButton("Up 1mm")
        self.jog_up_1_btn.clicked.connect(lambda: self.jog_axis(1))
        self.jog_up_1_btn.setEnabled(False)
        jog_grid.addWidget(self.jog_up_1_btn, 0, 1)
        
        # Zero button
        self.zero_axis_btn = QPushButton("Zero Axis")
        self.zero_axis_btn.clicked.connect(self.zero_axis)
        self.zero_axis_btn.setEnabled(False)
        jog_grid.addWidget(self.zero_axis_btn, 1, 0, 1, 2)
        
        # Down buttons
        self.jog_down_1_btn = QPushButton("Down 1mm")
        self.jog_down_1_btn.clicked.connect(lambda: self.jog_axis(-1))
        self.jog_down_1_btn.setEnabled(False)
        jog_grid.addWidget(self.jog_down_1_btn, 2, 0)
        
        self.jog_down_10_btn = QPushButton("Down 10mm")
        self.jog_down_10_btn.clicked.connect(lambda: self.jog_axis(-10))
        self.jog_down_10_btn.setEnabled(False)
        jog_grid.addWidget(self.jog_down_10_btn, 2, 1)
        
        jog_layout.addLayout(jog_grid)
        left_layout.addLayout(jog_layout)
        
        left_layout.addStretch()
        
        left_panel.setLayout(left_layout)
        main_layout.addWidget(left_panel, 1)
        
        # Right panel for chart and indicators
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        
        # Chart
        self.chart_widget = pg.PlotWidget()
        self.chart_widget.setBackground('w')
        self.chart_widget.setLabel('left', 'Force (lbf)')
        self.chart_widget.setLabel('bottom', 'Time (s)')
        self.chart_widget.showGrid(x=True, y=True)
        self.plot_line = self.chart_widget.plot([], [], pen=pg.mkPen(color='b', width=2))
        right_layout.addWidget(self.chart_widget, 4)
        
        # Indicators
        indicators_layout = QHBoxLayout()
        
        # Current force
        force_layout = QVBoxLayout()
        force_layout.addWidget(QLabel("Current force (lbf)"))
        self.force_indicator = QLabel("0.0")
        self.force_indicator.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.force_indicator.setStyleSheet("border: 1px solid gray; padding: 5px;")
        force_layout.addWidget(self.force_indicator)
        indicators_layout.addLayout(force_layout)
        
        # Current position
        position_layout = QVBoxLayout()
        position_layout.addWidget(QLabel("Current position (mm)"))
        self.position_indicator = QLabel("0.0")
        self.position_indicator.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.position_indicator.setStyleSheet("border: 1px solid gray; padding: 5px;")
        position_layout.addWidget(self.position_indicator)
        indicators_layout.addLayout(position_layout)
        
        # Current cycle count
        cycle_layout = QVBoxLayout()
        cycle_layout.addWidget(QLabel("Current cycle count"))
        self.cycle_indicator = QLabel("0")
        self.cycle_indicator.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cycle_indicator.setStyleSheet("border: 1px solid gray; padding: 5px;")
        cycle_layout.addWidget(self.cycle_indicator)
        indicators_layout.addLayout(cycle_layout)
        
        right_layout.addLayout(indicators_layout, 1)
        
        right_panel.setLayout(right_layout)
        main_layout.addWidget(right_panel, 3)
        
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Status bar for messages
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        self.load_cell_port_combo.clear()
        self.control_board_port_combo.clear()
        
        # Get list of available ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            port_name = f"{port.device} - {port.description}"
            self.load_cell_port_combo.addItem(port_name, port.device)
            self.control_board_port_combo.addItem(port_name, port.device)
        
        if not ports:
            self.status_bar.showMessage("No serial ports found", 3000)
    
    def update_axis_options(self):
        """Update Y-axis options based on X-axis selection and cycle count"""
        cycles = self.get_cycles()
        
        # If distance is selected for X-axis, it's only valid for non-cyclic tests
        if self.x_axis_combo.currentText() == "Distance (mm)" and cycles != 0:
            self.x_axis_combo.setCurrentIndex(0)  # Set back to Time
            QMessageBox.warning(self, "Invalid Selection", 
                                "Distance can only be used as X-axis for non-cyclic tests.")
    
    def get_cycles(self):
        """Get cycles value with validation"""
        try:
            cycles = int(self.cycles_input.text())
            if cycles < 0:
                raise ValueError("Cycles must be a positive integer or zero")
            return cycles
        except ValueError:
            return 0  # Default to non-cyclic test
    
    def get_speed(self):
        """Get speed value with validation"""
        try:
            speed = float(self.speed_input.text())
            if speed <= 0:
                raise ValueError("Speed must be positive")
            return min(speed, 50)  # Cap at 50 mm/s per spec
        except ValueError:
            return 10  # Default speed
    
    def get_distance(self):
        """Get distance value with validation"""
        try:
            distance = float(self.distance_input.text())
            # Apply limits
            if abs(distance) > 1000:
                QMessageBox.warning(self, "Invalid Distance", 
                                   "Distance is limited to +/- 1000mm for safety.")
                distance = 1000 if distance > 0 else -1000
            return distance
        except ValueError:
            return 50  # Default distance
    
    def browse_csv_file(self):
        """Open file dialog to select CSV save location"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save CSV File", "", "CSV Files (*.csv)"
        )
        if filename:
            if not filename.endswith('.csv'):
                filename += '.csv'
            self.filename_input.setText(filename)
    
    def toggle_connection(self):
        """Connect to or disconnect from the load cell and control board"""
        if self.is_connected:
            self.disconnect_devices()
        else:
            self.connect_devices()
    
    def connect_devices(self):
        """Connect to load cell and control board"""
        try:
            # Check serial ports
            load_cell_port = self.load_cell_port_combo.currentData()
            control_board_port = self.control_board_port_combo.currentData()
            
            if not load_cell_port:
                raise ValueError("No load cell port selected")
            if not control_board_port:
                raise ValueError("No control board port selected")
            
            # Start controller thread first
            self.controller_thread = ControllerThread(control_board_port, 0, 10, 0)
            self.controller_thread.cycle_updated.connect(self.on_cycle_updated)
            self.controller_thread.test_complete.connect(self.on_test_complete)
            self.controller_thread.error_signal.connect(self.on_error)
            
            # Initialize only (don't start the thread's main loop)
            self.controller_thread.initialize()
            
            # Give the controller thread a moment to initialize
            time.sleep(0.5)
            
            # Start load cell thread
            self.load_cell_thread = SerialReaderThread(load_cell_port)
            self.load_cell_thread.data_received.connect(self.on_data_received)
            self.load_cell_thread.error_signal.connect(self.on_error)
            self.load_cell_thread.start()
            
            # Zero the axis on connection
            if self.load_cell_thread:
                # Give the load cell thread a moment to start receiving data
                time.sleep(0.5)
                self.zero_axis()
            
            # Update UI
            self.is_connected = True
            self.connect_button.setText("Disconnect")
            self.start_button.setEnabled(True)
            self.toggle_jog_buttons(True)
            self.toggle_port_selection(False)
            self.status_bar.showMessage("Connected to devices")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to connect: {str(e)}")
            self.disconnect_devices()
    
    def disconnect_devices(self):
        """Disconnect from load cell and control board"""
        # Stop any running test
        if self.is_test_running:
            self.stop_test()
        
        # Stop and clean up threads
        if self.load_cell_thread:
            self.load_cell_thread.stop()
            self.load_cell_thread = None
            
        if self.controller_thread:
            self.controller_thread.cleanup()
            self.controller_thread = None
        
        # Update UI
        self.is_connected = False
        self.connect_button.setText("Connect")
        self.start_button.setEnabled(False)
        self.toggle_jog_buttons(False)
        self.toggle_port_selection(True)
        self.status_bar.showMessage("Disconnected from devices")
    
    def toggle_port_selection(self, enabled):
        """Enable or disable port selection controls"""
        self.load_cell_port_combo.setEnabled(enabled)
        self.control_board_port_combo.setEnabled(enabled)
        self.refresh_ports_btn.setEnabled(enabled)
    
    def toggle_jog_buttons(self, enabled):
        """Enable or disable jog control buttons"""
        self.jog_up_10_btn.setEnabled(enabled)
        self.jog_up_1_btn.setEnabled(enabled)
        self.jog_down_1_btn.setEnabled(enabled)
        self.jog_down_10_btn.setEnabled(enabled)
        self.zero_axis_btn.setEnabled(enabled)
    
    def jog_axis(self, distance):
        """Jog the axis by the specified distance"""
        if not self.is_connected or self.is_test_running or not self.controller_thread:
            return
        
        try:
            # Jog at 10mm/s as requested
            self.controller_thread.jog(distance, 10)
            
        except Exception as e:
            self.status_bar.showMessage(f"Jog error: {str(e)}")
    
    def zero_axis(self):
        """Set current position as zero"""
        if not self.is_connected or self.is_test_running or not self.load_cell_thread:
            return
        
        try:
            # Zero the position in the load cell thread (encoder based)
            self.load_cell_thread.zero_position()
            self.position_indicator.setText("0.0")
            self.status_bar.showMessage("Axis zeroed")
            
        except Exception as e:
            self.status_bar.showMessage(f"Zero axis error: {str(e)}")
    
    def start_test(self):
        """Start the test with current parameters"""
        if self.is_test_running or not self.is_connected:
            return
        
        # Validate inputs and get parameters
        try:
            cycles = self.get_cycles()
            speed = self.get_speed()
            distance = self.get_distance()
            
            # Prepare CSV file
            csv_filename = self.filename_input.text()
            if not csv_filename:
                raise ValueError("No CSV filename specified")
                
            self.csv_file = open(csv_filename, 'w', newline='')
            csv_headers = ['Time (ms)', 'Position (mm)', 'Force (lbf)']
            if cycles > 0:
                csv_headers.extend(['Cycle', 'Peak Tension (lbf)', 'Peak Compression (lbf)'])
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(csv_headers)
            
            # Clear data buffers
            self.times = []
            self.positions = []
            self.forces = []
            self.peak_tensions = []
            self.peak_compressions = []
            self.cycle_positions = []
            self.max_tension_force = 0
            self.max_compression_force = 0
            
            # Update chart based on selected variables
            self.update_chart_axes()
            
            # Start the test in the controller thread
            if self.controller_thread:
                self.controller_thread.start_test(cycles, speed, distance)
            
            # Update UI
            self.is_test_running = True
            self.toggle_input_fields(False)
            self.status_bar.showMessage("Test running...")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start test: {str(e)}")
            self.cleanup_test()
            
    def stop_test(self):
        """Stop the current test"""
        if not self.is_test_running:
            return
            
        # Stop test in controller thread
        if self.controller_thread:
            self.controller_thread.stop_test()
            
        self.cleanup_test()
        self.status_bar.showMessage("Test stopped by user")
    
    def cleanup_test(self):
        """Clean up after test completion or stoppage"""
        # Close CSV file
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        # Update UI
        self.is_test_running = False
        self.toggle_input_fields(True)
    
    def toggle_input_fields(self, enabled):
        """Enable or disable input fields during test"""
        self.start_button.setEnabled(enabled and self.is_connected)
        self.stop_button.setEnabled(not enabled and self.is_connected)
        self.connect_button.setEnabled(enabled)  # Can't disconnect during test
        self.cycles_input.setEnabled(enabled)
        self.x_axis_combo.setEnabled(enabled)
        self.y_axis_combo.setEnabled(enabled)
        self.speed_input.setEnabled(enabled)
        self.distance_input.setEnabled(enabled)
        self.filename_input.setEnabled(enabled)
        self.browse_button.setEnabled(enabled)
        self.toggle_jog_buttons(enabled and self.is_connected)
    
    def on_data_received(self, timestamp, position, force, raw_encoder_position):
        """Handle incoming data from load cell/encoder"""
        # Update indicators
        self.force_indicator.setText(f"{force:.2f}")
        self.position_indicator.setText(f"{position:.2f}")
        
        # Store data
        self.times.append(timestamp / 1000.0)  # Convert ms to s
        self.positions.append(position)
        self.forces.append(force)
        
        # Track peak forces for current cycle
        if force > self.max_tension_force:
            self.max_tension_force = force
        if force < self.max_compression_force:
            self.max_compression_force = force
        
        # Write to CSV
        if self.csv_writer:
            cycle = int(self.cycle_indicator.text())
            
            if self.get_cycles() > 0:
                self.csv_writer.writerow([
                    timestamp, position, force, cycle, 
                    self.max_tension_force, self.max_compression_force
                ])
            else:
                self.csv_writer.writerow([timestamp, position, force])
        
        # Update chart
        self.update_chart()
    
    def on_cycle_updated(self, cycle):
        """Handle cycle count updates"""
        self.cycle_indicator.setText(str(cycle))
        
        # Record peak forces for the completed cycle
        if cycle > 0:
            self.peak_tensions.append(self.max_tension_force)
            self.peak_compressions.append(self.max_compression_force)
            self.cycle_positions.append(cycle)
            
            # Reset peak tracking for next cycle
            self.max_tension_force = 0
            self.max_compression_force = 0
            
            # Update chart if showing peak data
            if (self.y_axis_combo.currentText() == "Peak tension force" or 
                self.y_axis_combo.currentText() == "Peak compression force"):
                self.update_chart()
    
    def on_test_complete(self):
        """Handle test completion"""
        self.status_bar.showMessage("Test completed successfully")
        self.cleanup_test()
    
    def on_error(self, message):
        """Handle error messages from threads"""
        self.status_bar.showMessage(message)
        
        if self.is_test_running:
            QMessageBox.critical(self, "Error", message)
            self.cleanup_test()
    
    def update_chart_axes(self):
        """Update chart axes based on selections"""
        # X-axis
        x_selection = self.x_axis_combo.currentText()
        if x_selection == "Time (seconds)":
            self.chart_widget.setLabel('bottom', 'Time (s)')
        else:  # Distance
            self.chart_widget.setLabel('bottom', 'Distance (mm)')
        
        # Y-axis
        y_selection = self.y_axis_combo.currentText()
        if y_selection == "Real-time force":
            self.chart_widget.setLabel('left', 'Force (lbf)')
        elif y_selection == "Peak tension force":
            self.chart_widget.setLabel('left', 'Peak Tension Force (lbf)')
        else:  # Peak compression
            self.chart_widget.setLabel('left', 'Peak Compression Force (lbf)')
        
        # Clear plot
        self.plot_line.setData([], [])
    
    def update_chart(self):
        """Update chart with latest data"""
        if not self.is_test_running or not self.times:
            return
        
        x_data = []
        y_data = []
        
        x_selection = self.x_axis_combo.currentText()
        y_selection = self.y_axis_combo.currentText()
        
        # Determine X-axis data
        if x_selection == "Time (seconds)":
            # Use time data
            if y_selection == "Real-time force":
                x_data = self.times
            else:
                # For peak data, use cycle numbers but scale to match time
                if self.cycle_positions:
                    # Create evenly spaced points based on cycle numbers
                    x_data = self.cycle_positions
        else:  # Distance
            # Use position data
            x_data = self.positions
        
        # Determine Y-axis data
        if y_selection == "Real-time force":
            y_data = self.forces
        elif y_selection == "Peak tension force" and self.peak_tensions:
            y_data = self.peak_tensions
        elif y_selection == "Peak compression force" and self.peak_compressions:
            y_data = self.peak_compressions
        
        # Update plot if we have data
        if x_data and y_data and len(x_data) == len(y_data):
            self.plot_line.setData(x_data, y_data)
            
            # Auto-scale view to fit all data
            self.chart_widget.enableAutoRange()
    
    def update_status(self):
        """Periodic status updates"""
        # Check if threads are still running
        if self.is_test_running:
            if (self.load_cell_thread and not self.load_cell_thread.isRunning() or
                self.controller_thread and not self.controller_thread.isRunning()):
                self.cleanup_test()
                self.status_bar.showMessage("Test stopped unexpectedly")
    
    def closeEvent(self, event):
        """Handle application close"""
        # Disconnect from devices
        self.disconnect_devices()
            
        # Close CSV file if open
        if self.csv_file:
            self.csv_file.close()
            
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ForceTestApp()
    window.show()
    sys.exit(app.exec())