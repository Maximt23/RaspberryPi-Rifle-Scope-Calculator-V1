# Author: Maxim Tsitolovsky
# Date: 2025-01-30
# Description: This script is a Python program that calculates the elevation and windage adjustments for a rifle scope based on the distance to the target, wind speed, and wind angle. The program uses a rangefinder to measure the distance to the target and an OLED display to show the calculated adjustments. It also reads the battery percentage from the Raspberry Pi and displays it on the OLED screen. The program runs continuously, updating the display every 0.5 seconds.
# Comment: Evan is a virgin 

import serial
import time
import board
import smbus
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
from typing import Optional, Dict, Union, NamedTuple
import logging
import math

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WindageData(NamedTuple):
    clicks: int
    direction: str  # LEFT or RIGHT

class ElevationData(NamedTuple):
    clicks: int
    direction: str  # UP or DOWN

class BallisticCalculator:
    MOA_CONVERSION = 1.047
    CLICKS_PER_MOA = 4      # 1/4 MOA scope
    SERIAL_TIMEOUT = 1
    SERIAL_BAUDRATE = 115200
    OLED_WIDTH = 128
    OLED_HEIGHT = 64
    
    # Wind deflection in inches for 10 mph crosswind
    # These values are for the 277 SIG Fury 165gr
    WIND_DRIFT_DATA = {
        100: 0.5,
        200: 2.1,
        300: 4.9,
        400: 9.0,
        500: 14.6,
        600: 21.8,
        700: 30.9,
        800: 42.1,
        900: 55.7,
        1000: 71.9
    }
    
    def __init__(self):
        self.drop_data = {
            100: -1.92,
            200: -9.73,
            300: -24.62,
            400: -48.11,
            500: -81.23,
            600: -125.47,
            700: -182.31,
            800: -253.15,
            900: -339.84,
            1000: -444.27
        }
        
        try:
            self.i2c_bus = smbus.SMBus(1)
            self.oled = adafruit_ssd1306.SSD1306_I2C(
                self.OLED_WIDTH, 
                self.OLED_HEIGHT, 
                self.i2c_bus
            )
            self.ser = serial.Serial(
                "/dev/serial0", 
                self.SERIAL_BAUDRATE, 
                timeout=self.SERIAL_TIMEOUT
            )
        except Exception as e:
            logger.error(f"Initialization error: {str(e)}")
            raise

    def calculate_wind_drift(self, distance_yards: float, wind_speed: float, wind_angle: float) -> Optional[float]:
        """
        Calculate wind drift in inches.
        wind_speed: in mph
        wind_angle: in degrees (0-360, where 90 is full value wind from right)
        """
        # Get base drift for 10 mph wind
        distances = sorted(self.WIND_DRIFT_DATA.keys())
        base_drift = None
        
        if distance_yards in self.WIND_DRIFT_DATA:
            base_drift = self.WIND_DRIFT_DATA[distance_yards]
        else:
            for i in range(len(distances) - 1):
                d1, d2 = distances[i], distances[i + 1]
                if d1 < distance_yards < d2:
                    drift1, drift2 = self.WIND_DRIFT_DATA[d1], self.WIND_DRIFT_DATA[d2]
                    base_drift = drift1 + (drift2 - drift1) * ((distance_yards - d1) / (d2 - d1))
        
        if base_drift is None:
            return None
            
        # Calculate crosswind component
        wind_angle_rad = math.radians(wind_angle)
        crosswind = abs(wind_speed * math.sin(wind_angle_rad))
        
        # Scale drift for actual wind speed
        actual_drift = base_drift * (crosswind / 10.0)
        
        # Determine direction
        if 0 <= wind_angle <= 180:
            return -actual_drift  # Wind from right pushes bullet left
        else:
            return actual_drift   # Wind from left pushes bullet right

    def calculate_adjustments(self, distance_yards: float, wind_speed: float, wind_angle: float) -> tuple[ElevationData, WindageData]:
        """Calculate both elevation and windage adjustments"""
        # Elevation (vertical) adjustment
        drop_inches = self.get_bullet_drop(distance_yards)
        if drop_inches is None:
            return None, None
            
        elevation_moa = drop_inches / (self.MOA_CONVERSION * (distance_yards / 100))
        elevation_clicks = abs(round(elevation_moa * self.CLICKS_PER_MOA))
        elevation_direction = "UP" if drop_inches < 0 else "DOWN"
        
        # Windage (horizontal) adjustment
        drift_inches = self.calculate_wind_drift(distance_yards, wind_speed, wind_angle)
        if drift_inches is None:
            return ElevationData(elevation_clicks, elevation_direction), None
            
        windage_moa = drift_inches / (self.MOA_CONVERSION * (distance_yards / 100))
        windage_clicks = abs(round(windage_moa * self.CLICKS_PER_MOA))
        windage_direction = "LEFT" if drift_inches < 0 else "RIGHT"
        
        return (ElevationData(elevation_clicks, elevation_direction),
                WindageData(windage_clicks, windage_direction))

    def display_oled(self, distance_yards: float, elevation: ElevationData, 
                    windage: Optional[WindageData], battery: Optional[int]) -> None:
        try:
            self.oled.fill(0)
            image = Image.new("1", (self.OLED_WIDTH, self.OLED_HEIGHT))
            draw = ImageDraw.Draw(image)
            font = ImageFont.load_default()

            # Display distance
            draw.text((5, 0), f"{int(distance_yards)} YDS", font=font, fill=255)
            
            # Display elevation adjustment
            draw.text((5, 16), f"ELEV: {elevation.clicks} {elevation.direction}", 
                     font=font, fill=255)
            
            # Display windage adjustment if available
            if windage:
                draw.text((5, 32), f"WIND: {windage.clicks} {windage.direction}", 
                         font=font, fill=255)
            
            # Display MOA setting and battery
            draw.text((5, 48), "1/4 MOA", font=font, fill=255)
            if battery is not None:
                draw.text((80, 48), f"BAT:{battery}%", font=font, fill=255)

            self.oled.image(image)
            self.oled.show()
            
        except Exception as e:
            logger.error(f"Display error: {str(e)}")

    def get_distance(self) -> Optional[float]:
        try:
            self.ser.write(b'\x5A\x04\x04\x62')
            time.sleep(0.1)
            data = self.ser.read(9)
            if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                distance_cm = data[2] + (data[3] << 8)
                distance_yards = distance_cm / 91.44
                return round(distance_yards, 1)
            return None
        except serial.SerialException as e:
            logger.error(f"Serial error: {str(e)}")
            return None

    def get_bullet_drop(self, distance_yards: float) -> Optional[float]:
        distances = sorted(self.drop_data.keys())
        if distance_yards in self.drop_data:
            return self.drop_data[distance_yards]
        if distance_yards < min(distances) or distance_yards > max(distances):
            return None
        for i in range(len(distances) - 1):
            d1, d2 = distances[i], distances[i + 1]
            if d1 < distance_yards < d2:
                drop1, drop2 = self.drop_data[d1], self.drop_data[d2]
                return drop1 + (drop2 - drop1) * ((distance_yards - d1) / (d2 - d1))
        return None

    def get_battery_percentage(self) -> Optional[int]:
        try:
            with open("/sys/class/power_supply/BAT0/capacity", "r") as f:
                return int(f.read().strip())
        except (IOError, ValueError):
            return None

    def run(self):
        logger.info("Starting calculator with wind compensation")
        
        # Default wind values - these could be made adjustable with buttons
        wind_speed = 10  # mph
        wind_angle = 90  # degrees (90 = from right, 270 = from left)
        
        while True:
            try:
                distance_yards = self.get_distance()
                battery = self.get_battery_percentage()

                if distance_yards:
                    elevation, windage = self.calculate_adjustments(
                        distance_yards, wind_speed, wind_angle)
                    
                    if elevation:
                        logger.info(
                            f"Distance: {distance_yards} yds | "
                            f"Elevation: {elevation.clicks} clicks {elevation.direction} | "
                            f"Windage: {windage.clicks if windage else 0} "
                            f"clicks {windage.direction if windage else 'none'}"
                        )
                        self.display_oled(distance_yards, elevation, windage, battery)

                time.sleep(0.5)
                
            except KeyboardInterrupt:
                logger.info("Program terminated by user")
                break
            except Exception as e:
                logger.error(f"Runtime error: {str(e)}")
                time.sleep(1)

if __name__ == "__main__":
    try:
        calculator = BallisticCalculator()
        calculator.run()
    except Exception as e:
        logger.error(f"Fatal error: {str(e)}")