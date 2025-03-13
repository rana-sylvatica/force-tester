import time
from machine import Pin, I2C

# HX711 Load Cell Class
class HX711:
    def __init__(self, dout_pin, sck_pin, gain=128):
        self.pSCK = Pin(sck_pin, Pin.OUT)
        self.pDOUT = Pin(dout_pin, Pin.IN, Pin.PULL_UP)
        self.gain = gain
        
        self.offset = 0
        self.scale = 1
        
        # Initialize the HX711
        self.pSCK.value(0)
        self.power_up()
        self.set_gain(gain)
    
    def set_gain(self, gain):
        if gain == 128:
            self.gain = 1
        elif gain == 64:
            self.gain = 3
        elif gain == 32:
            self.gain = 2
        else:
            self.gain = 1  # Default to 128
        
        self.pSCK.value(0)
        self.read()
    
    def is_ready(self):
        return self.pDOUT.value() == 0
    
    def power_up(self):
        self.pSCK.value(0)
        time.sleep_us(100)
    
    def power_down(self):
        self.pSCK.value(0)
        self.pSCK.value(1)
        time.sleep_us(100)
    
    def read(self):
        # Wait for the sensor to be ready
        while not self.is_ready():
            pass
        
        # Read 24 bits of data
        data = 0
        for i in range(24):
            self.pSCK.value(1)
            time.sleep_us(1)
            data = (data << 1) | self.pDOUT.value()
            self.pSCK.value(0)
            time.sleep_us(1)
        
        # Set the gain for the next reading
        for i in range(self.gain):
            self.pSCK.value(1)
            time.sleep_us(1)
            self.pSCK.value(0)
            time.sleep_us(1)
        
        # Convert 2's complement to signed int
        if data & 0x800000:
            data = data - 0x1000000
        
        return data
    
    def read_average(self, times=5):  # Reduced from 10 to 5 samples
        sum = 0
        for i in range(times):
            sum += self.read()
            # Removed the 5ms delay to speed up averaging
        return sum // times
    
    def get_value(self):
        return self.read_average() - self.offset
    
    def get_units(self):
        return self.get_value() / self.scale
    
    def tare(self, times=10):
        self.offset = self.read_average(times)
    
    def set_scale(self, scale):
        self.scale = scale
    
    def get_scale(self):
        return self.scale
    
    def get_calibrated_value(self, no_load_value, known_weight_value, known_weight, raw=None):
        """
        Convert raw value to calibrated weight units
        
        Parameters:
        no_load_value: Raw value when no load is applied
        known_weight_value: Raw value when known weight is applied
        known_weight: The known weight value (in desired output units)
        raw: Optional raw value to use instead of reading again
        
        Returns:
        Weight in desired units
        """
        # Calculate the scale factor (how many raw units per weight unit)
        scale_factor = abs(known_weight_value - no_load_value) / known_weight
        
        # Get current raw value or use provided one
        if raw is None:
            raw = self.read()
        
        # Determine if it's tension or compression
        if raw < no_load_value:
            # Tension (load cell value decreases under tension)
            calibrated = (no_load_value - raw) / scale_factor
        else:
            # Compression (load cell value increases under compression)
            calibrated = -1 * (raw - no_load_value) / scale_factor
        
        return calibrated

# AS5600 Magnetic Encoder Class
class AS5600:
    def __init__(self, i2c, addr=0x36):
        self.i2c = i2c
        self.addr = addr
        self.REG_ANGLE_H = 0x0E
        self.REG_ANGLE_L = 0x0F
        
        # Motor and leadscrew parameters
        self.STEPS_PER_REV = 200          # Standard 1.8° stepper motor
        self.MICROSTEPS = 16              # 16x microstepping
        self.LEAD_MM_PER_REV = 10         # 10mm lead per revolution (4-start, 2.5mm pitch)
        
        # Initialize starting position (1000mm to avoid negative values)
        self.start_position_mm = 1000
        self.previous_raw_angle = None
        self.position_mm = self.start_position_mm
        self.total_angle_change = 0       # Track cumulative angle change
    
    def is_connected(self):
        devices = self.i2c.scan()
        return self.addr in devices
    
    def read_angle(self):
        # Read high and low bytes of angle
        angle_h = self.i2c.readfrom_mem(self.addr, self.REG_ANGLE_H, 1)[0]
        angle_l = self.i2c.readfrom_mem(self.addr, self.REG_ANGLE_L, 1)[0]
        
        # Combine high and low bytes to get 12-bit angle value (0-4095)
        raw_angle = (angle_h << 8) | angle_l
        
        # Convert raw angle (0-4095) to degrees (0-359.9)
        degrees = (raw_angle * 360) / 4096
        
        return raw_angle, degrees
    
    def update_position(self):
        raw_angle, degrees = self.read_angle()
        
        # Initial reading - just store it
        if self.previous_raw_angle is None:
            self.previous_raw_angle = raw_angle
            return self.position_mm
        
        # Calculate angle change, handling wraparound
        angle_change = raw_angle - self.previous_raw_angle
        
        # Handle wraparound (0 to 4095 transition or 4095 to 0 transition)
        if angle_change > 2048:         # Large positive change means we went backward past zero
            angle_change -= 4096
        elif angle_change < -2048:      # Large negative change means we went forward past zero
            angle_change += 4096
            
        # Update total angle change
        self.total_angle_change += angle_change
        
        # Convert angle change to position change in mm
        # (angle_change / 4096) is the fraction of a revolution
        # multiply by lead screw movement per revolution
        # Negative sign to reverse direction (up becomes down and vice versa)
        position_change_mm = -1 * (angle_change / 4096) * self.LEAD_MM_PER_REV
        
        # Update position
        self.position_mm += position_change_mm
        
        # Update previous angle for next calculation
        self.previous_raw_angle = raw_angle
        
        return self.position_mm


# Main program
def main():
    # Initialize I2C for AS5600 encoder
    i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100000)
    
    # Initialize AS5600 encoder
    encoder = AS5600(i2c)
    
    # Check if AS5600 is connected
    if not encoder.is_connected():
        print(f"Error: AS5600 not found at address 0x{encoder.addr:02X}")
        return
    
    # Initialize HX711 load cell
    # DOUT connected to GPIO 16, SCK connected to GPIO 17
    hx = HX711(dout_pin=16, sck_pin=17, gain=128)  # 10Hz mode
    
    # Calibration values - replace with your actual measured values
    NO_LOAD_VALUE = 308925        # Raw value with no load
    KNOWN_LOAD_VALUE = -127000    # Raw value with 5lb tension
    KNOWN_LOAD_WEIGHT = 5.0       # Known weight in pounds
    
    # Perform auto-tare on startup to account for any fixtures
    print("Starting auto-tare. Please ensure the system is at rest with any fixtures in place.")
    print("Do not apply any force for 3 seconds...")
    
    # Wait for system to stabilize
    time.sleep(3)
    
    # Get average reading with fixture (10 samples for stability)
    fixture_raw_value = hx.read_average(10)
    print(f"Auto-tare complete. Fixture offset: {fixture_raw_value - NO_LOAD_VALUE}")
    
    # Update the no-load value to account for the fixture
    FIXTURE_ADJUSTED_NO_LOAD = fixture_raw_value
    
    # Wait a moment before starting measurements
    time.sleep(1)
    print("Starting measurements...")
    
    # Starting timestamp
    start_time = time.ticks_ms()
    
    # Send a header line for CSV format
    print('timestamp,raw_value,pounds,position_mm')
    
    try:
        while True:
            # Read the current force value
            raw_value = hx.read()
            
            # Calculate calibrated value in pounds - using fixture-adjusted no-load value
            pounds = hx.get_calibrated_value(FIXTURE_ADJUSTED_NO_LOAD, KNOWN_LOAD_VALUE, KNOWN_LOAD_WEIGHT, raw_value)
            
            # Update position from encoder
            position_mm = encoder.update_position()
            
            # Get current timestamp (milliseconds since start)
            current_time = time.ticks_ms()
            elapsed_time = time.ticks_diff(current_time, start_time)
            
            # Send data in CSV format with 3 decimal places for pounds and 2 for position
            print(f'{elapsed_time},{raw_value},{pounds:.3f},{position_mm:.2f}')
            
            # The HX711 at gain 128 has a natural ~10Hz max rate
            # The is_ready() method in read() already waits for the HX711 to complete its conversion
    
    except KeyboardInterrupt:
        print("Stopped by user")

if __name__ == "__main__":
    main()