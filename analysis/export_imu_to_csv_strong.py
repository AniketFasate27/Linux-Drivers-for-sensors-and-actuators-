#!/usr/bin/env python3
import sys, csv, math, pathlib, re
from rosbags.highlevel import AnyReader

def quat_to_euler_deg(x,y,z,w):
    # ROS standard
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi/2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw

def nmea_checksum_ok(line: str) -> bool:
    try:
        s = line.strip()
        i = s.index('$') + 1
        j = s.rindex('*')
        data = s[i:j]
        cs = 0
        for ch in data:
            cs ^= ord(ch)
        want = s[j+1:j+3].upper()
        have = f'{cs:02X}'
        return want == have
    except Exception:
        return False

def parse_vnymr(line: str):
    """
    $VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyrX,GyrY,GyrZ*CS
    Units: angles [deg], mag [Gauss], acc [g], gyro [deg/s]
    Returns dict with Euler deg + SI units.
    """
    if not line.startswith('$VNYMR'):
        return None
    if '*' in line and not nmea_checksum_ok(line):
        return None
    parts = line.split('*')[0].split(',')
    if len(parts) < 13:
        return None
    _, yaw, pitch, roll, mx, my, mz, ax, ay, az, gx, gy, gz = parts[:13]
    try:
        yaw   = float(yaw);   pitch = float(pitch); roll = float(roll)
        mx_g  = float(mx);    my_g  = float(my);    mz_g = float(mz)
        ax_g  = float(ax);    ay_g  = float(ay);    az_g = float(az)
        gx_d  = float(gx);    gy_d  = float(gy);    gz_d = float(gz)
    except Exception:
        return None

    g = 9.80665
    d2r = math.pi/180.0
    # convert to SI
    return {
        'roll_deg': roll, 'pitch_deg': pitch, 'yaw_deg': yaw,
        'mx_T': mx_g*1e-4, 'my_T': my_g*1e-4, 'mz_T': mz_g*1e-4,
        'ax_ms2': ax_g*g, 'ay_ms2': ay_g*g, 'az_ms2': az_g*g,
        'gx_rad_s': gx_d*d2r, 'gy_rad_s': gy_d*d2r, 'gz_rad_s': gz_d*d2r,
    }

def is_zero_quat(x,y,z,w):
    return (x,y,z,w) == (0.0,0.0,0.0,0.0)

bag_path = pathlib.Path(sys.argv[1])   # e.g. ~/EECE5554/gnss/data/imu_stationary
out_csv  = pathlib.Path(sys.argv[2])   # e.g. ~/EECE5554/gnss/analysis/imu_stationary.csv

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
            'ax_ms2','ay_ms2','az_ms2',
            'gx_rad_s','gy_rad_s','gz_rad_s',
            'mx_T','my_T','mz_T',
            'qx','qy','qz','qw',
            'roll_deg','pitch_deg','yaw_deg',
            'raw'
        ])
        t0 = None
        for _, t, rawdata in reader.messages(connections=[conn]):
            msg = reader.deserialize(rawdata, conn.msgtype)
            ts = t/1e9
            if t0 is None: t0 = ts
            trel = ts - t0

            imu = msg.imu
            mag = msg.mag_field
            raw = getattr(msg, 'raw', '')

            # defaults
            qx=qy=qz=qw=None
            roll=pitch=yaw=None
            ax=ay=az=gx=gy=gz=mx=my=mz=None

            # try taking quaternion from message
            if hasattr(imu, 'orientation'):
                qx = getattr(imu.orientation, 'x', None)
                qy = getattr(imu.orientation, 'y', None)
                qz = getattr(imu.orientation, 'z', None)
                qw = getattr(imu.orientation, 'w', None)
                if None not in (qx,qy,qz,qw) and not is_zero_quat(qx,qy,qz,qw):
                    roll,pitch,yaw = quat_to_euler_deg(qx,qy,qz,qw)

            # angular vel & linear accel from message (already SI if published correctly)
            if hasattr(imu, 'angular_velocity'):
                gx = getattr(imu.angular_velocity, 'x', None)
                gy = getattr(imu.angular_velocity, 'y', None)
                gz = getattr(imu.angular_velocity, 'z', None)
            if hasattr(imu, 'linear_acceleration'):
                ax = getattr(imu.linear_acceleration, 'x', None)
                ay = getattr(imu.linear_acceleration, 'y', None)
                az = getattr(imu.linear_acceleration, 'z', None)
            if hasattr(mag, 'magnetic_field'):
                mx = getattr(mag.magnetic_field, 'x', None)
                my = getattr(mag.magnetic_field, 'y', None)
                mz = getattr(mag.magnetic_field, 'z', None)

            # If Euler missing or some channels None, try to parse raw VNYMR
            if (roll is None or pitch is None or yaw is None) or \
               (None in (ax,ay,az,gx,gy,gz,mx,my,mz)):
                parsed = parse_vnymr(raw) if isinstance(raw, str) else None
                if parsed:
                    roll   = parsed['roll_deg']   if roll   is None else roll
                    pitch  = parsed['pitch_deg']  if pitch  is None else pitch
                    yaw    = parsed['yaw_deg']    if yaw    is None else yaw
                    ax     = parsed['ax_ms2']     if ax     is None else ax
                    ay     = parsed['ay_ms2']     if ay     is None else ay
                    az     = parsed['az_ms2']     if az     is None else az
                    gx     = parsed['gx_rad_s']   if gx     is None else gx
                    gy     = parsed['gy_rad_s']   if gy     is None else gy
                    gz     = parsed['gz_rad_s']   if gz     is None else gz
                    mx     = parsed['mx_T']       if mx     is None else mx
                    my     = parsed['my_T']       if my     is None else my
                    mz     = parsed['mz_T']       if mz     is None else mz
                    # if we lacked quaternion, synthesize one from Euler
                    if qx is None or qy is None or qz is None or qw is None:
                        # build quaternion from roll,pitch,yaw
                        r = math.radians(roll); p = math.radians(pitch); y = math.radians(yaw)
                        cy, sy = math.cos(y*0.5), math.sin(y*0.5)
                        cp, sp = math.cos(p*0.5), math.sin(p*0.5)
                        cr, sr = math.cos(r*0.5), math.sin(r*0.5)
                        qw = cr*cp*cy + sr*sp*sy
                        qx = sr*cp*cy - cr*sp*sy
                        qy = cr*sp*cy + sr*cp*sy
                        qz = cr*cp*sy - sr*sp*cy
                        n = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw) or 1.0
                        qx,qy,qz,qw = qx/n,qy/n,qz/n,qw/n

            w.writerow([trel, ax, ay, az, gx, gy, gz, mx, my, mz,
                        qx, qy, qz, qw, roll, pitch, yaw, raw])

print(f"Wrote {out_csv}")
