import sys, csv, math, pathlib
from rosbags.highlevel import AnyReader

import math

def quat_to_euler_deg(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw

bag_path = pathlib.Path(sys.argv[1])
out_csv = pathlib.Path(sys.argv[2])

with AnyReader([bag_path]) as reader:
    conns = [c for c in reader.connections if c.topic == '/imu']
    if not conns:
        print("No /imu topic found.")
        sys.exit(1)
    conn = conns[0]

    with out_csv.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'time_s',
            'ax_ms2', 'ay_ms2', 'az_ms2',
            'gx_rad_s', 'gy_rad_s', 'gz_rad_s',
            'mx_T', 'my_T', 'mz_T',
            'qx', 'qy', 'qz', 'qw',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'raw'
        ])
        t0 = None
        for _, t, rawdata in reader.messages(connections=[conn]):
            msg = reader.deserialize(rawdata, conn.msgtype)
            ts = t / 1e9
            if t0 is None:
                t0 = ts
            trel = ts - t0

            imu = msg.imu
            mag = msg.mag_field

            ax = getattr(imu.linear_acceleration, 'x', None)
            ay = getattr(imu.linear_acceleration, 'y', None)
            az = getattr(imu.linear_acceleration, 'z', None)
            gx = getattr(imu.angular_velocity, 'x', None)
            gy = getattr(imu.angular_velocity, 'y', None)
            gz = getattr(imu.angular_velocity, 'z', None)
            qx = getattr(imu.orientation, 'x', None)
            qy = getattr(imu.orientation, 'y', None)
            qz = getattr(imu.orientation, 'z', None)
            qw = getattr(imu.orientation, 'w', None)
            mx = getattr(mag.magnetic_field, 'x', None)
            my = getattr(mag.magnetic_field, 'y', None)
            mz = getattr(mag.magnetic_field, 'z', None)

            if None not in (qx, qy, qz, qw):
                roll, pitch, yaw = quat_to_euler_deg(qx, qy, qz, qw)
            else:
                roll = pitch = yaw = None

            raw_line = getattr(msg, 'raw', '')
            w.writerow([trel, ax, ay, az, gx, gy, gz, mx, my, mz,
                        qx, qy, qz, qw, roll, pitch, yaw, raw_line])
            

            def quat_ok(qx,qy,qz,qw):
                try:
                    n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
                    return n > 0.5  # any reasonable non-zero norm
                except Exception:
                    return False
                
            def quat_to_euler_deg(x,y,z,w):
                # ROS ENU quaternion → roll, pitch, yaw (deg)
                sinr_cosp = 2*(w*x + y*z)
                cosr_cosp = 1 - 2*(x*x + y*y)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
                sinp = 2*(w*y - z*x)
                pitch = math.degrees(math.copysign(math.pi/2, sinp)) if abs(sinp) >= 1 else math.degrees(math.asin(sinp))
                siny_cosp = 2*(w*z + x*y)
                cosy_cosp = 1 - 2*(y*y + z*z)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                return roll, pitch, yaw
            
            def parse_vnymr_angles(raw):
                # $VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyrX,GyrY,GyrZ*CS
                if not isinstance(raw, str) or "$VNYMR" not in raw:
                    return None
                try:
                    parts = raw.split('*')[0].split(',')
                    yaw = float(parts[1]); pitch = float(parts[2]); roll = float(parts[3])
                    return roll, pitch, yaw
                except Exception:
                    return None
                
            if None not in (qx,qy,qz,qw) and quat_ok(qx,qy,qz,qw):
                roll, pitch, yaw = quat_to_euler_deg(qx,qy,qz,qw)
            else:
                # 2) fallback: parse the raw VNYMR string
                e = parse_vnymr_angles(raw_line)
                roll, pitch, yaw = (e if e is not None else (None, None, None))

print(f"Wrote {out_csv}")
