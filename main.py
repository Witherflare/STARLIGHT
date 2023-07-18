import machine
import utime
import ustruct
import sys
import time
import gc
import struct
import math
import micropython
import uio
from machine import Pin, PWM

###############################################################################
# Constants

# Refresh rate
LOOP_HZ = 1000

# Servo centers
SERVO_X_CENTER = 110;
SERVO_Y_CENTER = 70;

# Pins
LED = Pin(24, Pin.OUT)
SERVO_Y = PWM(Pin(18))
SERVO_Y.freq(50)
SERVO_X = PWM(Pin(11))
SERVO_X.freq(50)
IGNITER = Pin(6, Pin.OUT)
EJECTION = Pin(7, Pin.OUT)
SERVO_OE = Pin(14, Pin.OUT)

# I2C address
BMP388_ADR = 0x76
ICM42605_ADR = 0x68

# Registers
REG_CHIPID = 0x00
REG_TEMP = 0x07

SEA_LEVEL_PRESSURE = 1019

###############################################################################
###############################################################################
# Settings

# Initialize I2C with pins
i2c = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=9600)

###############################################################################
###############################################################################
# Functions

def getTemperature():
    return read_temp_and_pressure(i2c)[0];

def getPressure():
    return read_temp_and_pressure(i2c)[1];

def getAltitude():
    """The altitude in meters based on the currently set sea level pressure."""
    # see https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
    return 44307.7 * (1 - (getPressure() / SEA_LEVEL_PRESSURE) ** 0.190284)

def toInt(data):
    return int.from_bytes(data, 'big')

def toHex(data):
    return hex(toInt(data))

def append_hex(a, b):
    sizeof_b = 0

    # get size of b in bits
    while((b >> sizeof_b) > 0):
        sizeof_b += 1

    # align answer to nearest 4 bits (hex digit)
    sizeof_b += sizeof_b % 4

    return (a << sizeof_b) | b


def read_temp_and_pressure(i2c):
    # See if readings are ready
    status = i2c.readfrom_mem(BMP388_ADR, 0x03, 1)
    #print(toHex(status))
    if (toInt(status) & 0x60 != 0x60):
        print("Not ready")
    else:
        # ** If you want to know how this works, don't ask me. I stole all this code from https://github.com/adafruit/Adafruit_CircuitPython_BMP3XX/
        
        # read and bit shift our readings
        data = i2c.readfrom_mem(BMP388_ADR, 0x04, 6)
        adc_p = data[2] << 16 | data[1] << 8 | data[0]
        adc_t = data[5] << 16 | data[4] << 8 | data[3]
        #print(adc_t)
        
        T1, T2, T3 = temp_calib
        
        pd1 = adc_t - T1
        pd2 = pd1 * T2
        
        temperature = pd2 + (pd1 * pd1) * T3 #TEMPERATURE IN C
                
        # datasheet, sec 9.3 Pressure compensation
        P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11 = pressure_calib

        pd1 = P6 * temperature
        pd2 = P7 * temperature**2.0
        pd3 = P8 * temperature**3.0
        po1 = P5 + pd1 + pd2 + pd3

        pd1 = P2 * temperature
        pd2 = P3 * temperature**2.0
        pd3 = P4 * temperature**3.0
        po2 = adc_p * (P1 + pd1 + pd2 + pd3)

        pd1 = adc_p**2.0
        pd2 = P9 + P10 * temperature
        pd3 = pd1 * pd2
        pd4 = pd3 + P11 * adc_p**3.0

        pressure = (po1 + po2 + pd4)/100 #PRESSURE IN hPa
        
        return(temperature, pressure)
            
def calibrate_bmp388():
    # find our compensation coefficient values (this shit is all copied from adafruit's library, don't ask me how it works)
    coeff = i2c.readfrom_mem(BMP388_ADR, 0x31, 21)
    coeff = struct.unpack("<HHbhhbbHHbbhbb", coeff)
    
    global temp_calib
    global pressure_calib
    temp_calib = (
        coeff[0] / 2**-8.0,  # T1
        coeff[1] / 2**30.0,  # T2
        coeff[2] / 2**48.0,
    )  # T3
    pressure_calib = (
        (coeff[3] - 2**14.0) / 2**20.0,  # P1
        (coeff[4] - 2**14.0) / 2**29.0,  # P2
        coeff[5] / 2**32.0,  # P3
        coeff[6] / 2**37.0,  # P4
        coeff[7] / 2**-3.0,  # P5
        coeff[8] / 2**6.0,  # P6
        coeff[9] / 2**8.0,  # P7
        coeff[10] / 2**15.0,  # P8
        coeff[11] / 2**48.0,  # P9
        coeff[12] / 2**48.0,  # P10
        coeff[13] / 2**65.0,
    )  # P11

def enable_temp_and_pressure(i2c):
    i2c.writeto_mem(BMP388_ADR, 0x1B, b'\x33')

def enable_gyro():
    i2c.writeto_mem(ICM42605_ADR, 0x4E, b'\x0F')
    i2c.writeto_mem(ICM42605_ADR, 0x56, b'\x02')
    i2c.writeto_mem(ICM42605_ADR, 0x4F, b'\x03')

def enable_servos():
    SERVO_OE.value(1)
#     SERVO_X.duty_u16(2000)

def servo_goto(servo, degrees):
    value = (degrees * 35) + 1500
    servo.duty_u16(math.floor(value))
    time.sleep(0.01)
    
def save_data(temp_data):
    with open('data.json', 'a') as f:
        f.write(temp_data)

def calibrate_accel():
    print("CALIBRATING ACCELEROMETER... MAKE SURE YOU KEEP THE ROCKET COMPLETELY STILL")

    precision = 25 # change this number to change precision of calibration. higher number will take longer

    # X
    x_result = []
    for i in range(precision):
        data = i2c.readfrom_mem(ICM42605_ADR, 0x1F, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_value & 0x8000:
            accel_value -= 0x10000
        x_result.append(accel_value * (16/32768))
        time.sleep(0.01)
        
    # Y
    y_result = []
    for i in range(precision):
        data = i2c.readfrom_mem(ICM42605_ADR, 0x21, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_value & 0x8000:
            accel_value -= 0x10000
        y_result.append(accel_value * (16/32768))
        time.sleep(0.01)
        
    # Z
    z_result = []
    for i in range(precision):
        data = i2c.readfrom_mem(ICM42605_ADR, 0x23, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_value & 0x8000:
            accel_value -= 0x10000
        z_result.append(accel_value * (16/32768))
        time.sleep(0.01)
        
    x_err = sum(x_result) / len(x_result)
    y_err = sum(y_result) / len(y_result)
    z_err = sum(z_result) / len(z_result)
    
    return x_err, y_err, z_err



def calibrate_gyros():
    print("CALIBRATING GYROSCOPES... MAKE SURE YOU KEEP THE ROCKET COMPLETELY STILL")
    
    precision = 25 # change this number to change precision of calibration. higher number will take longer
    
    # X
    x_result = []
    for i in range(precision):
        data = i2c.readfrom_mem(ICM42605_ADR, 0x25, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_value & 0x8000:
            accel_value -= 0x10000
        x_result.append(accel_value * (16/32768))
        time.sleep(0.01)
        
    # Y
    y_result = []
    for i in range(precision):
        data = i2c.readfrom_mem(ICM42605_ADR, 0x27, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_value & 0x8000:
            accel_value -= 0x10000
        y_result.append(accel_value * (16/32768))
        time.sleep(0.01)

    # Z
    z_result = []
    for i in range(precision):
        data = i2c.readfrom_mem(ICM42605_ADR, 0x29, 2)
        # Assuming data[0] is the upper byte and data[1] is the lower byte
        accel_value = (data[0] << 8) | data[1]
        # Convert to signed integer
        if accel_value & 0x8000:
            accel_value -= 0x10000
        z_result.append(accel_value * (16/32768))
        time.sleep(0.01)
    
    x_err = sum(x_result) / len(x_result)
    y_err = sum(y_result) / len(y_result)
    z_err = sum(z_result) / len(z_result)
    
    return x_err, y_err, z_err

def accel_x(x_err):
    data = i2c.readfrom_mem(ICM42605_ADR, 0x1F, 2)
    # Assuming data[0] is the upper byte and data[1] is the lower byte
    accel_value = (data[0] << 8) | data[1]
    # Convert to signed integer
    if accel_value & 0x8000:
        accel_value -= 0x10000
    
    return accel_value * (16/32768) - x_err

def accel_y(y_err):
    data = i2c.readfrom_mem(ICM42605_ADR, 0x21, 2)
    # Assuming data[0] is the upper byte and data[1] is the lower byte
    accel_value = (data[0] << 8) | data[1]
    # Convert to signed integer
    if accel_value & 0x8000:
        accel_value -= 0x10000
    
    return accel_value * (16/32768) - y_err

def accel_z(z_err):
    data = i2c.readfrom_mem(ICM42605_ADR, 0x23, 2)
    # Assuming data[0] is the upper byte and data[1] is the lower byte
    accel_value = (data[0] << 8) | data[1]
    # Convert to signed integer
    if accel_value & 0x8000:
        accel_value -= 0x10000
    
    return accel_value * (16/32768) - z_err


def get_x(x_err):
    data = i2c.readfrom_mem(ICM42605_ADR, 0x25, 2)
    # Assuming data[0] is the upper byte and data[1] is the lower byte
    gyro_value = (data[0] << 8) | data[1]
    # Convert to signed integer
    if gyro_value & 0x8000:
        gyro_value -= 0x10000
    
    return gyro_value * (16/32768) - x_err
    
def get_y(y_err):
    data = i2c.readfrom_mem(ICM42605_ADR, 0x27, 2)
    # Assuming data[0] is the upper byte and data[1] is the lower byte
    gyro_value = (data[0] << 8) | data[1]
    # Convert to signed integer
    if gyro_value & 0x8000:
        gyro_value -= 0x10000
    
    return gyro_value * (16/32768) - y_err

def get_z(z_err):
    data = i2c.readfrom_mem(ICM42605_ADR, 0x29, 2)
    # Assuming data[0] is the upper byte and data[1] is the lower byte
    gyro_value = (data[0] << 8) | data[1]
    # Convert to signed integer
    if gyro_value & 0x8000:
        gyro_value -= 0x10000
    
    return gyro_value * (16/32768) - z_err

###############################################################################
###############################################################################
# Main


enable_temp_and_pressure(i2c)
calibrate_bmp388()
enable_servos()
enable_gyro()
x_err, y_err, z_err = calibrate_gyros()
x_accel_err, y_accel_err, z_accel_err = calibrate_accel()


# Reset data.json
with uio.open('data.json', 'w') as f:
    f.write('{ "data": [')
    f.close()

time.sleep(1)


last_tick = time.ticks_ms()
downtime = 0
loops = 0

logging = False
in_flight = False

x = 0
y = 0
z = 0

accel_data = 0

servo_goto(SERVO_X, SERVO_X_CENTER)
servo_goto(SERVO_Y, SERVO_Y_CENTER)


temperature = 0
pressure = 0

print("Done booting")
while True:
    
    if loops % LOOP_HZ == 0:
        temperature = getTemperature()
        pressure = getPressure()
    
    accel_data += accel_z(z_accel_err)
    
    x += get_x(x_err)
    z += get_z(z_err)

    # Set to in flight if movement is detected
    if not -0.5 <= accel_data < 0.5:
        in_flight = True
#         logging = True

    if in_flight:
        print(x*10)
        print(z*10)
        servo_goto(SERVO_X, SERVO_X_CENTER + (x*5))
        servo_goto(SERVO_Y, SERVO_Y_CENTER + (z*5))
    
    # Log data
    if logging:
        save_data(('{"timestamp": %(ts)s, "temperature": %(temp)s, "pressure": %(pressure)s, "x": %(x)s, "y": %(y)s, "z": %(z)s}, ' % {'ts': time.ticks_ms(), 'temp': temperature, 'pressure': pressure, 'x': x, 'y': y, 'z': z}))
    
    # Code to manage our loop, to make sure it fires when it's supposed to
    while time.ticks_ms() < last_tick + (1/LOOP_HZ)*1000:
        downtime += 1
        time.sleep(0.0001)
        pass
    last_tick = time.ticks_ms()
    if downtime < 5:
#         print("System overloaded! Try lowering refresh rate or decreasing load in the loop.")
        pass
    downtime = 0
    loops+=1


