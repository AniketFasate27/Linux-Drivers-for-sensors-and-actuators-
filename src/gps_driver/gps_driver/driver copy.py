import rclpy
from rclpy.node import Node
from gps_interface.msg import Customgps
from std_msgs.msg import Header

import serial
from serial import serialutil
import utm, time, calendar

def convertMinutesToDecimal(dm: float) -> float:
    deg = int(dm // 100)
    mins = dm - deg * 100
    return deg + mins / 60.0

def signConversion(coord: float, dirc: str) -> float:
    return -abs(coord) if dirc in ("S","W") else abs(coord)

def utc_to_utc_epoch(utc):
    utc = float(utc)
    utc_in_secs = (utc // 10000) * 3600 + ((utc % 10000) // 100) * 60 + (utc % 100)
    now = time.time()
    today = time.gmtime(now)
    bod = calendar.timegm((today.tm_year, today.tm_mon, today.tm_mday, 0, 0, 0))
    current = bod + utc_in_secs
    sec = int(current)
    nsec = int(round((current - sec) * 1e9))
    if nsec == 1_000_000_000:
        sec += 1
        nsec = 0
    return [sec, nsec]

class GpsDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 4800)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('serial_url', '')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)
        timeout = float(self.get_parameter('timeout').value)
        serial_url = self.get_parameter('serial_url').get_parameter_value().string_value

        try:
            if serial_url:
                self.ser = serial.serial_for_url(serial_url, baudrate=baud, timeout=timeout)
                self.get_logger().info(f'Opened serial URL "{serial_url}" baud={baud}')
            else:
                self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
                self.get_logger().info(f'Opened serial port "{port}" baud={baud}')
        except serialutil.SerialException as e:
            self.get_logger().fatal(f'Failed to open serial: {e}')
            raise

        self.pub = self.create_publisher(Customgps, '/gps', 10)
        self.timer = self.create_timer(0.5, self.publish_sample)

    def publish_sample(self):
        try:
            line = self.ser.readline()
        except serialutil.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            return
        if not line:
            return

        gpgga = line.decode('utf-8', errors='ignore').strip()
        if not gpgga.startswith("$GPGGA"):
            return
        parts = gpgga.split(',')
        if len(parts) < 10:
            return

        try:
            utc = parts[1]
            lat = float(parts[2]); lat_dir = parts[3]
            lon = float(parts[4]); lon_dir = parts[5]
            hdop = float(parts[8]) if parts[8] else float('nan')
            alt = float(parts[9]) if parts[9] else float('nan')
        except (ValueError, IndexError):
            return

        lat_dec = convertMinutesToDecimal(lat)
        lon_dec = convertMinutesToDecimal(lon)
        lat_signed = signConversion(lat_dec, lat_dir)
        lon_signed = signConversion(lon_dec, lon_dir)
        e, n, zone, letter = utm.from_latlon(lat_signed, lon_signed)
        sec, nsec = utc_to_utc_epoch(utc)

        msg = Customgps()
        msg.header = Header()
        msg.header.frame_id = 'GPS1_Frame'
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.latitude = lat_signed
        msg.longitude = lon_signed
        msg.altitude = alt
        msg.utm_easting = e
        msg.utm_northing = n
        msg.zone = int(zone)
        msg.letter = letter
        msg.hdop = hdop
        msg.gpgga_read = gpgga
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GpsDriver()
    try:
        rclpy.spin(node)
    finally:
        try:
            if hasattr(node, 'ser') and node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
