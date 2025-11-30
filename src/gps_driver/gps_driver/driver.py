import rclpy
from rclpy.node import Node
from gps_interface.msg import Customgps
from std_msgs.msg import Header

import serial
from serial import serialutil
import utm
import time, calendar

def convert_minutes_to_decimal(dm: float) -> float:
    deg = int(dm // 100)
    mins = dm - deg * 100
    return deg + mins / 60.0

def signed(coord: float, hemi: str) -> float:
    return -abs(coord) if hemi in ("S", "W") else abs(coord)

def parse_utc_to_epoch(utc_str: str):
    """
    utc_str like 'hhmmss.sss'. Use current UTC date, preserve fractional seconds.
    """
    if not utc_str:
        # Fallback: current system time (discouraged, but avoids crashes)
        now = time.time()
        sec = int(now); nsec = int((now - sec) * 1e9)
        return sec, nsec

    # Split fractional seconds explicitly
    if '.' in utc_str:
        hhmmss, frac = utc_str.split('.', 1)
        frac_s = float(f"0.{frac}") if frac.isdigit() else 0.0
    else:
        hhmmss, frac_s = utc_str, 0.0

    # Zero-pad to 6 for safety (e.g., '2359' -> '002359' is not valid, but emulator sends full hhmmss)
    hhmmss = hhmmss.zfill(6)
    try:
        h = int(hhmmss[0:2])
        m = int(hhmmss[2:4])
        s = int(hhmmss[4:6])
    except ValueError:
        now = time.time()
        sec = int(now); nsec = int((now - sec) * 1e9)
        return sec, nsec

    today = time.gmtime(time.time())
    bod = calendar.timegm((today.tm_year, today.tm_mon, today.tm_mday, 0, 0, 0))
    current = bod + h * 3600 + m * 60 + s + frac_s
    sec = int(current)
    nsec = int(round((current - sec) * 1e9))
    if nsec == 1_000_000_000:
        sec += 1
        nsec = 0
    return sec, nsec

def nmea_checksum_valid(sentence: str) -> bool:
    """
    Validate NMEA checksum. sentence is entire line like '$GPGGA,...*5A'
    """
    if '*' not in sentence:
        return False
    try:
        body = sentence[sentence.find('$') + 1:sentence.find('*')]
        given = sentence[sentence.find('*') + 1:].strip()
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        return f"{calc:02X}" == given.upper()
    except Exception:
        return False

class GpsDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.audit_printed = False

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 4800)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('serial_url', '')  # e.g., 'tcp://127.0.0.1:9000' or other emulator endpoints

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)
        timeout = float(self.get_parameter('timeout').value)
        serial_url = self.get_parameter('serial_url').get_parameter_value().string_value

        # Open serial (URL for emulator, device for puck)
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
        self.timer = self.create_timer(0.05, self.poll_serial)  # 20 Hz read, gated by line availability
        self.msg_count = 0

    def poll_serial(self):
        try:
            line = self.ser.readline()
        except serialutil.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            return
        if not line:
            return

        raw = line.decode('utf-8', errors='ignore').strip()
        if not raw.startswith('$GPGGA'):
            return
        if not nmea_checksum_valid(raw):
            # Drop corrupted lines silently to keep stream clean
            return

        # Strip trailing checksum part for parsing convenience
        star_idx = raw.find('*')
        core = raw[1:star_idx] if star_idx != -1 else raw[1:]
        parts = core.split(',')
        # GPGGA field map
        # 0=GPGGA,1=UTC,2=lat,3=N/S,4=lon,5=E/W,6=fix,7=nsats,8=hdop,9=alt,10=alt_unit,11=geoid,12=geoid_unit,13=age,14=ref
        if len(parts) < 10:
            return

        try:
            utc_str = parts[1]                 # hhmmss.sss
            lat_str = parts[2]; lat_hemi = parts[3]
            lon_str = parts[4]; lon_hemi = parts[5]
            fix = parts[6] if len(parts) > 6 else ''
            nsats = parts[7] if len(parts) > 7 else ''
            hdop_str = parts[8] if len(parts) > 8 else ''
            alt_str = parts[9] if len(parts) > 9 else ''
        except Exception:
            return

        # Empty guards
        if not lat_str or not lon_str or not lat_hemi or not lon_hemi:
            return

        # Parse numbers
        try:
            lat_dm = float(lat_str)
            lon_dm = float(lon_str)
            lat_dec = convert_minutes_to_decimal(lat_dm)
            lon_dec = convert_minutes_to_decimal(lon_dm)
            lat = signed(lat_dec, lat_hemi)
            lon = signed(lon_dec, lon_hemi)
        except ValueError:
            return

        # UTM conversion
        try:
            easting, northing, zone, letter = utm.from_latlon(lat, lon)
        except Exception as e:
            #ssself.get_logger().warn(f'UTM conversion failed: {e}')
            self.get_logger().warning(f'UTM conversion failed: {e}')

            return

        # Time (epoch sec/nsec)
        sec, nsec = parse_utc_to_epoch(utc_str)

        # Optional fields
        try:
            hdop = float(hdop_str) if hdop_str else float('nan')
        except ValueError:
            hdop = float('nan')

        try:
            alt = float(alt_str) if alt_str else float('nan')
        except ValueError:
            alt = float('nan')

        # Build and publish message
        msg = Customgps()
        msg.header = Header()
        msg.header.frame_id = 'GPS1_Frame'
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nsec)

        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(alt)
        msg.utm_easting = float(easting)
        msg.utm_northing = float(northing)
        msg.zone = int(zone)
        msg.letter = str(letter)
        msg.hdop = float(hdop)
        msg.gpgga_read = raw
        
                # --- Print 5 audit lines once for verification ---
        if not self.audit_printed:
            iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(sec)) + f'.{nsec:09d}Z'

            # 1) GPGGA parsing
            self.get_logger().info(
                f"[1/5 GPGGA] fix={fix or '?'} sats={nsats or '?'} hdop={parts[8] if len(parts)>8 else '?'} "
                f"alt={parts[9] if len(parts)>9 else '?'} raw='{raw}'"
            )

            # 2) Converts lat/lon to decimal
            self.get_logger().info(
                f"[2/5 Lat/Lon] {lat_dm:.5f}{lat_hemi} {lon_dm:.5f}{lon_hemi} -> "
                f"lat={lat:.6f}, lon={lon:.6f}"
            )

            # 3) Converts decimal to UTM
            self.get_logger().info(
                f"[3/5 UTM] easting={easting:.3f} m, northing={northing:.3f} m, zone={zone}, letter={letter}"
            )

            # 4) Correct time handling (UTC->epoch with nsec)
            self.get_logger().info(
                f"[4/5 Time] utc='{utc_str}' -> epoch={sec}.{nsec:09d} ({iso})"
            )

            # 5) Correct ROS message structure (header + key fields)
            self.get_logger().info(
                f"[5/5 ROS] frame_id={msg.header.frame_id}, topic=/gps, type=gps_interface/Customgps, "
                f"alt={msg.altitude}, hdop={msg.hdop}"
            )
            self.audit_printed = True

        # --- Publish every message ---
        self.pub.publish(msg)
        self.msg_count += 1

        # Light periodic log (every 50 msgs)
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f"gps msgs={self.msg_count} fix={fix or '?'} sats={nsats or '?'} "
                f"lat={lat:.6f} lon={lon:.6f} E={easting:.2f} N={northing:.2f}"
            )

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

if __name__ == '__main__':
    main()
